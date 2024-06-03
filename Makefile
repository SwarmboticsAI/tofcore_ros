SHELL := /bin/bash
# Version info for appImage
VERSION_MAJOR ?= 0
VERSION_MINOR ?= 0
BITBUCKET_BUILD_NUMBER ?= 0
REPO_REF ?= develop
export VERSION_MAJOR
export VERSION_MINOR
export BITBUCKET_BUILD_NUMBER
export REPO_REF

TARGET_DISTRO_VER ?= $(shell lsb_release -sc)
SET_ROS2=source /opt/ros/galactic/setup.bash 

.PHONY: pipeline
pipeline:
	$(MAKE) build

.PHONY: ros2
ros2:
	 colcon build
	 
.PHONY: clean
clean:
	rm -rf build install log 
	rm -r -f dist
	rm -r -f target
	rm -r -f ros2/debian
	rm -r -f ros2/.obj-x86_64-linux-gnu
	rm -r -f ros2/target
	rm -r -f tofcore_msgs/debian
	rm -r -f tofcore_msgs/.obj-x86_64-linux-gnu
	rm -r -f ros-galactic-tofcore* 
	rm -r -f AppDir
	rm -r -f appimage-builder-cache
	rm -r -f *.AppImage
	rm -r -f *.zsync
.PHONY: clobber
clobber: clean
	rm -rf ros2/tofcore_ros/libtofcore ros1/tofcore_ros/libtofcore algorithm ros2/algorithm
