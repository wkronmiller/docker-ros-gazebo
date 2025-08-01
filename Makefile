# Get the git tag if exists, else fallback to commit hash
GIT_TAG := $(shell git describe --tags --exact-match 2>/dev/null || git rev-parse --short HEAD)
BASE_NAME := ghcr.io/wkronmiller/docker-ros-base
GAZEBO_NAME := ghcr.io/wkronmiller/docker-ros-gazebo
GUI_NAME := ghcr.io/wkronmiller/docker-ros-gazebo-gui

.PHONY: all base gazebo gui

all: gui

base:
	docker build -t $(BASE_NAME):$(GIT_TAG) -f base/Dockerfile .

gazebo: base
	docker build --build-arg BASE_IMAGE=$(BASE_NAME):$(GIT_TAG) -t $(GAZEBO_NAME):$(GIT_TAG) -f gazebo/Dockerfile .

gui: gazebo
	docker build --build-arg BASE_IMAGE=$(GAZEBO_NAME):$(GIT_TAG) -t $(GUI_NAME):$(GIT_TAG) -f gui/Dockerfile .

publish: base gazebo gui
	docker push $(BASE_NAME):$(GIT_TAG)
	docker push $(GAZEBO_NAME):$(GIT_TAG)
	docker push $(GUI_NAME):$(GIT_TAG)

publish-latest: publish
	docker tag $(BASE_NAME):$(GIT_TAG) $(BASE_NAME):latest
	docker push $(BASE_NAME):latest
	docker tag $(GAZEBO_NAME):$(GIT_TAG) $(GAZEBO_NAME):latest
	docker push $(GAZEBO_NAME):latest
	docker tag $(GUI_NAME):$(GIT_TAG) $(GUI_NAME):latest
	docker push $(GUI_NAME):latest

print-tag:
	@echo "Tag: $(GIT_TAG)"

print-image-names: print-tag
	@echo "Base Image: $(BASE_NAME)"
	@echo "Gazebo Image: $(GAZEBO_NAME)"
	@echo "GUI Image: $(GUI_NAME)"
