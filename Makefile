#!/bin/bash

.PHONY: docs release clean build test run

ROS_DISTRO=indigo
ROS_CI_DESKTOP="`lsb_release -cs`"
CI_SOURCE_PATH=$(shell pwd)/mediator_bot
ROSINSTALL_FILE=$(CI_SOURCE_PATH)/dependencies.rosinstall
CATKIN_OPTIONS=$(CI_SOURCE_PATH)/catkin.options
ROS_PARALLEL_JOBS='-j8 -l6'

BUILD_DIR=build

OS=$(shell lsb_release -si)
ARCH=$(shell uname -m | sed 's/x86_//;s/i[3-6]86/32/')
VER=$(shell lsb_release -sr)

ifeq ($(OS), LinuxMint)
ifeq ($(VER), 17.3)
# Override mint rosa with ubuntu trusty 
$(info "Using Ubuntu 14.04 trusty")
ROS_CI_DESKTOP="trusty"
OS_OVERRIDE=--os ubuntu:trusty
else
$(error "Invalid OS")
endif
endif

clean:
	rm -rf env htmlcov $(BUILD_DIR) .eggs *.egg-info .cache

setup_env:
	virtualenv -p /usr/bin/python env

install_ros:
	sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu $(ROS_CI_DESKTOP) main\" > /etc/apt/sources.list.d/ros-latest.list"
	wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
	sudo apt-get update -qq
	sudo apt-get install -y python-catkin-pkg python-rosdep python-wstool ros-$(ROS_DISTRO)-catkin python-rosinstall
	bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash"
# Prepare rosdep to install dependencies.
	sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
	sudo rosdep init
	rosdep update

setup_build:
	@echo "Using build dir $(BUILD_DIR)"
	mkdir -p $(BUILD_DIR)/src
	- cd $(BUILD_DIR)/src && \
	catkin_init_workspace
# Create the devel/setup.bash (run catkin_make with an empty workspace) and
# source it to set the path variables.
	cd $(BUILD_DIR) && \
	catkin_make
# Add the project to the workspace using a symlink.
	ln -s $(CI_SOURCE_PATH) $(BUILD_DIR)/src

full_build: clean setup_env setup_build build

install_dependencies:
	rm -f $(BUILD_DIR)/src/.rosinstall*
	wstool init $(BUILD_DIR)/src
	bash -c "source $(BUILD_DIR)/devel/setup.bash" && \
	cd $(BUILD_DIR)/src && \
	if [ -f $(ROSINSTALL_FILE) ] ; then wstool merge $(ROSINSTALL_FILE) ; fi && \
	wstool up
# package depdencies: install using rosdep.
	bash -c "source $(BUILD_DIR)/devel/setup.bash" && \
	cd $(BUILD_DIR) && \
	rosdep install -y --from-paths src --ignore-src --rosdistro $(ROS_DISTRO) $(OS_OVERRIDE)

build: install_dependencies
	rm -rf $(BUILD_DIR)/build/catkin_pip_env
	virtualenv -p python2.7 $(BUILD_DIR)/build/catkin_pip_env
# Setup a new virtual environment for th build
	. $(BUILD_DIR)/build/catkin_pip_env/bin/activate && \
	pip install --upgrade -r test_requirements.txt && \
	bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash" && \
	bash -c "source $(BUILD_DIR)/devel/setup.bash" && \
	cd $(BUILD_DIR) && \
	catkin_make $( [ -f $(CATKIN_OPTIONS) ] && cat $(CATKIN_OPTIONS) )

test:
	. $(BUILD_DIR)/build/catkin_pip_env/bin/activate && \
	bash -c "source $(BUILD_DIR)/devel/setup.bash" && \
	cd $(BUILD_DIR) && \
	catkin_make run_tests && \
	catkin_test_results

docs:
	sphinx-build -aE docs build/docs > /dev/null

