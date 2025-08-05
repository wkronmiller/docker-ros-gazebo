# =============================================================================
# Multi-Component AI Rover Image Build System
# =============================================================================
# Build system for the three-container architecture:
# - flight-controller: ArduPilot SITL autopilot
# - flight-computer: ROS2 navigation + MAVROS bridge  
# - simulation: Gazebo physics simulation
# Maintains compatibility with existing layered image system
# =============================================================================

# Get the git tag if exists, else fallback to commit hash
GIT_TAG := $(shell git describe --tags --exact-match 2>/dev/null || git rev-parse --short HEAD)

# Original layered image names (maintained for compatibility)
BASE_NAME := ghcr.io/wkronmiller/docker-ros-base
GAZEBO_NAME := ghcr.io/wkronmiller/docker-ros-gazebo
GUI_NAME := ghcr.io/wkronmiller/docker-ros-gazebo-gui

# New multi-component image names
FLIGHT_CONTROLLER_NAME := ghcr.io/wkronmiller/rover-flight-controller
FLIGHT_COMPUTER_NAME := ghcr.io/wkronmiller/rover-flight-computer
SIMULATION_NAME := ghcr.io/wkronmiller/rover-simulation

.PHONY: all base gazebo gui flight-controller flight-computer simulation
.PHONY: publish publish-multicomponent publish-latest clean help
.PHONY: print-tag print-image-names test-flight-controller test-containers

# =============================================================================
# Main Targets
# =============================================================================

# Default target builds everything
all: multicomponent

# New multi-component build system
multicomponent: flight-controller flight-computer simulation

# Help target
help:
	@echo "==================================================================="
	@echo "Multi-Component AI Rover Image Build System"
	@echo "==================================================================="
	@echo "Main targets:"
	@echo "  all                 - Build all multi-component images (default)"
	@echo "  base                - Build base ROS2 image"
	@echo "  gazebo              - Build Gazebo + navigation image"
	@echo "  gui                 - Build GUI desktop image"
	@echo ""
	@echo "Component targets:"
	@echo "  flight-controller   - Build ArduPilot SITL flight controller"
	@echo "  flight-computer     - Build ROS2 + MAVROS flight computer"
	@echo "  simulation          - Build Gazebo physics simulation"
	@echo ""
	@echo "Publishing targets:"
	@echo "  publish             - Push all images to registry"
	@echo "  publish-latest      - Push and tag as latest"
	@echo "  publish-base        - Push base images only"
	@echo "  publish-multicomponent - Push multi-component images only"
	@echo ""
	@echo "Testing targets:"
	@echo "  test-flight-controller - Test flight controller image"
	@echo "  test-containers        - Test all container images"
	@echo ""
	@echo "Utility targets:"
	@echo "  clean               - Clean up build artifacts"
	@echo "  print-tag           - Print current git tag"
	@echo "  print-image-names   - Print all image names"
	@echo "==================================================================="

# =============================================================================
# Build Targets
# =============================================================================

base:
	@echo "Building base ROS2 image..."
	docker build -t $(BASE_NAME):$(GIT_TAG) -f base/Dockerfile .

gazebo: base
	@echo "Building Gazebo + navigation image..."
	docker build --build-arg BASE_IMAGE=$(BASE_NAME):$(GIT_TAG) -t $(GAZEBO_NAME):$(GIT_TAG) -f gazebo/Dockerfile .

gui: gazebo
	@echo "Building GUI desktop image..."
	docker build --build-arg BASE_IMAGE=$(GAZEBO_NAME):$(GIT_TAG) -t $(GUI_NAME):$(GIT_TAG) -f gui/Dockerfile .

flight-controller:
	@echo "Building flight controller (ArduPilot SITL)..."
	docker build -t $(FLIGHT_CONTROLLER_NAME):$(GIT_TAG) -f flight-controller/Dockerfile flight-controller/

flight-computer: gazebo
	@echo "Building flight computer (ROS2 + MAVROS)..."
	docker build --build-arg BASE_IMAGE=$(GAZEBO_NAME):$(GIT_TAG) -t $(FLIGHT_COMPUTER_NAME):$(GIT_TAG) -f flight-computer/Dockerfile flight-computer/

simulation: gazebo
	@echo "Building simulation (Gazebo physics)..."
	docker build --build-arg BASE_IMAGE=$(GAZEBO_NAME):$(GIT_TAG) -t $(SIMULATION_NAME):$(GIT_TAG) -f simulation/Dockerfile simulation/

# =============================================================================
# Publishing Targets
# =============================================================================

publish-base: base gazebo gui
	@echo "Publishing base images..."
	docker push $(BASE_NAME):$(GIT_TAG)
	docker push $(GAZEBO_NAME):$(GIT_TAG)
	docker push $(GUI_NAME):$(GIT_TAG)

publish-multicomponent: multicomponent
	@echo "Publishing multi-component images..."
	docker push $(FLIGHT_CONTROLLER_NAME):$(GIT_TAG)
	docker push $(FLIGHT_COMPUTER_NAME):$(GIT_TAG)
	docker push $(SIMULATION_NAME):$(GIT_TAG)

publish: publish-base publish-multicomponent

publish-latest: publish
	@echo "Tagging and pushing as latest..."
	# Tag and push base images as latest
	docker tag $(BASE_NAME):$(GIT_TAG) $(BASE_NAME):latest
	docker push $(BASE_NAME):latest
	docker tag $(GAZEBO_NAME):$(GIT_TAG) $(GAZEBO_NAME):latest
	docker push $(GAZEBO_NAME):latest
	docker tag $(GUI_NAME):$(GIT_TAG) $(GUI_NAME):latest
	docker push $(GUI_NAME):latest
	# Tag and push multi-component images as latest
	docker tag $(FLIGHT_CONTROLLER_NAME):$(GIT_TAG) $(FLIGHT_CONTROLLER_NAME):latest
	docker push $(FLIGHT_CONTROLLER_NAME):latest
	docker tag $(FLIGHT_COMPUTER_NAME):$(GIT_TAG) $(FLIGHT_COMPUTER_NAME):latest
	docker push $(FLIGHT_COMPUTER_NAME):latest
	docker tag $(SIMULATION_NAME):$(GIT_TAG) $(SIMULATION_NAME):latest
	docker push $(SIMULATION_NAME):latest

# =============================================================================
# Testing Targets
# =============================================================================

test-flight-controller: flight-controller
	@echo "Testing flight controller image..."
	docker run --rm -d --name test-fc $(FLIGHT_CONTROLLER_NAME):$(GIT_TAG)
	sleep 10
	docker exec test-fc pgrep -f "ardupilot" || (echo "ArduPilot not running"; exit 1)
	docker stop test-fc
	@echo "Flight controller test passed!"

test-containers: test-flight-controller
	@echo "All container tests passed!"

# =============================================================================
# Utility Targets
# =============================================================================

clean:
	@echo "Cleaning up Docker build artifacts..."
	docker system prune -f
	docker builder prune -f

clean-all: clean
	@echo "Removing all rover-related Docker images..."
	docker rmi $(shell docker images --filter=reference="*rover*" -q) 2>/dev/null || true
	docker rmi $(shell docker images --filter=reference="$(BASE_NAME)*" -q) 2>/dev/null || true
	docker rmi $(shell docker images --filter=reference="$(GAZEBO_NAME)*" -q) 2>/dev/null || true
	docker rmi $(shell docker images --filter=reference="$(GUI_NAME)*" -q) 2>/dev/null || true

print-tag:
	@echo "Git Tag: $(GIT_TAG)"

print-image-names: print-tag
	@echo "==================================================================="
	@echo "Base Images:"
	@echo "  Base: $(BASE_NAME):$(GIT_TAG)"
	@echo "  Gazebo: $(GAZEBO_NAME):$(GIT_TAG)"
	@echo "  GUI: $(GUI_NAME):$(GIT_TAG)"
	@echo ""
	@echo "Multi-Component Images:"
	@echo "  Flight Controller: $(FLIGHT_CONTROLLER_NAME):$(GIT_TAG)"
	@echo "  Flight Computer: $(FLIGHT_COMPUTER_NAME):$(GIT_TAG)"
	@echo "  Simulation: $(SIMULATION_NAME):$(GIT_TAG)"
	@echo "==================================================================="