#!/usr/bin/env python3
"""
Navigation to MAVLink Bridge
============================
Converts ROS2 navigation commands to ArduPilot MAVLink messages.
Provides seamless integration between Nav2 stack and ArduPilot flight controller.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from mavros_msgs.msg import ManualControl, State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header

import threading
import time


class NavToMAVLink(Node):
    """Bridge node converting ROS2 nav commands to ArduPilot MAVLink messages."""
    
    def __init__(self):
        super().__init__('nav_to_mavlink_bridge')
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers to MAVROS
        self.manual_control_pub = self.create_publisher(
            ManualControl, 
            '/mavros/manual_control/send', 
            qos_profile
        )
        
        # Subscribers from navigation stack
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            qos_profile
        )
        
        # MAVROS state subscriber
        self.state_sub = self.create_subscription(
            State, 
            '/mavros/state', 
            self.state_callback, 
            qos_profile
        )
        
        # Service clients for vehicle setup
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # State variables
        self.current_state = None
        self.last_cmd_vel = Twist()
        self.vehicle_armed = False
        self.guided_mode_set = False
        
        # Control scaling factors
        self.linear_scale = 1000.0    # Scale linear velocity to throttle
        self.angular_scale = 1000.0   # Scale angular velocity to steering
        
        # Timer for regular MAVLink command transmission
        self.control_timer = self.create_timer(0.1, self.send_control_commands)
        
        # Setup timer (runs once after delay)
        self.setup_timer = self.create_timer(2.0, self.setup_vehicle_once)
        self.setup_completed = False
        
        self.get_logger().info("MAVROS Bridge initialized")
        
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands from navigation stack."""
        self.last_cmd_vel = msg
        
    def state_callback(self, msg):
        """Handle MAVROS state updates."""
        self.current_state = msg
        self.vehicle_armed = msg.armed
        
        # Log state changes
        if hasattr(self, '_last_mode') and self._last_mode != msg.mode:
            self.get_logger().info(f"Vehicle mode changed: {msg.mode}")
        if hasattr(self, '_last_armed') and self._last_armed != msg.armed:
            self.get_logger().info(f"Vehicle armed: {msg.armed}")
            
        self._last_mode = msg.mode
        self._last_armed = msg.armed
        
    def setup_vehicle_once(self):
        """Setup vehicle mode and arming (runs once)."""
        if self.setup_completed:
            self.setup_timer.cancel()
            return
            
        if self.current_state is None:
            self.get_logger().warn("No MAVROS state received yet")
            return
            
        if not self.current_state.connected:
            self.get_logger().warn("MAVROS not connected to flight controller")
            return
            
        # Set up vehicle in separate thread
        setup_thread = threading.Thread(target=self._setup_vehicle_thread)
        setup_thread.daemon = True
        setup_thread.start()
        
        self.setup_completed = True
        self.setup_timer.cancel()
        
    def _setup_vehicle_thread(self):
        """Vehicle setup in separate thread to avoid blocking."""
        try:
            # Set GUIDED mode
            if self.current_state.mode != "GUIDED":
                self.get_logger().info("Setting GUIDED mode...")
                mode_request = SetMode.Request()
                mode_request.custom_mode = "GUIDED"
                
                if self.set_mode_client.wait_for_service(timeout_sec=5.0):
                    future = self.set_mode_client.call_async(mode_request)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                    
                    if future.result() and future.result().mode_sent:
                        self.get_logger().info("GUIDED mode set successfully")
                        self.guided_mode_set = True
                    else:
                        self.get_logger().error("Failed to set GUIDED mode")
                else:
                    self.get_logger().error("Set mode service not available")
                    
            # Wait for mode change
            time.sleep(1.0)
            
            # Arm vehicle
            if not self.current_state.armed:
                self.get_logger().info("Arming vehicle...")
                arm_request = CommandBool.Request()
                arm_request.value = True
                
                if self.arming_client.wait_for_service(timeout_sec=5.0):
                    future = self.arming_client.call_async(arm_request)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                    
                    if future.result() and future.result().success:
                        self.get_logger().info("Vehicle armed successfully")
                    else:
                        self.get_logger().error("Failed to arm vehicle")
                else:
                    self.get_logger().error("Arming service not available")
                    
        except Exception as e:
            self.get_logger().error(f"Vehicle setup error: {str(e)}")
    
    def send_control_commands(self):
        """Send control commands to ArduPilot via MAVROS."""
        if self.current_state is None or not self.current_state.connected:
            return
            
        # Create manual control message
        manual_control = ManualControl()
        manual_control.header = Header()
        manual_control.header.stamp = self.get_clock().now().to_msg()
        
        # Convert cmd_vel to ArduPilot manual control
        # Linear velocity -> throttle (0 to 1000, centered at 500)
        throttle = int(self.last_cmd_vel.linear.x * self.linear_scale/2 + 500)
        throttle = max(0, min(1000, throttle))
        
        # Angular velocity -> steering (-1000 to 1000)
        steering = int(-self.last_cmd_vel.angular.z * self.angular_scale)
        steering = max(-1000, min(1000, steering))
        
        manual_control.x = throttle    # Forward/backward
        manual_control.y = steering    # Left/right steering
        manual_control.z = 500         # Throttle center
        manual_control.r = 0           # Yaw rate
        manual_control.buttons = 0     # Button states
        
        # Publish control command
        self.manual_control_pub.publish(manual_control)


def main(args=None):
    """Main entry point for the bridge node."""
    rclpy.init(args=args)
    
    try:
        bridge_node = NavToMAVLink()
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in MAVROS bridge: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()