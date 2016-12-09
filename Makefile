#!/bin/bash

.PHONY: docs release clean build test run setup-build install-ros setup-env install-dependencies full-build

ROS_DISTRO=indigo
ROS_CI_DESKTOP="`lsb_release -cs`"
CI_SOURCE_PATH:=$(shell pwd)/mediator_bot
ROSINSTALL_FILE=$(CI_SOURCE_PATH)/dependencies.rosinstall
CATKIN_OPTIONS=$(CI_SOURCE_PATH)/catkin.options
ROS_PARALLEL_JOBS='-j8 -l6'

BUILD_DIR:=$(shell pwd)/build

OS:=$(shell lsb_release -si)
ARCH:=$(shell uname -m | sed 's/x86_//;s/i[3-6]86/32/')
VER:=$(shell lsb_release -sr)

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

all: full-build

clean:
	rm -rf env htmlcov $(BUILD_DIR) .eggs *.egg-info .cache

setup-env:
	virtualenv -p python2.7 env

install-ros:
	sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu $(ROS_CI_DESKTOP) main\" > /etc/apt/sources.list.d/ros-latest.list"
	wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
	sudo apt-get update -qq
	sudo apt-get install -y python-catkin-pkg python-rosdep python-wstool ros-$(ROS_DISTRO)-catkin python-rosinstall ros-$(ROS_DISTRO)-smach ros-$(ROS_DISTRO)-smach-viewer
# Prepare rosdep to install dependencies.
	sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
	sudo rosdep init
	rosdep update

setup-build: setup-env
	@echo "Using build dir $(BUILD_DIR)"
	mkdir -p $(BUILD_DIR)/src
	mkdir -p $(BUILD_DIR)/build
	-ln -s $(shell pwd)/env $(BUILD_DIR)/build/catkin_pip_env
# Create the devel/setup.bash (run catkin_make with an empty workspace)
	( \
	. $(BUILD_DIR)/build/catkin_pip_env/bin/activate ; \
	sudo apt-get update -qq ; \
	sudo apt-get install -y libboost-all-dev libblitz0-dev libopenblas-dev liblapack-dev gfortran; \
	pip install --upgrade numpy==1.11.2 ; \
	pip install --upgrade -r bob-requirements.txt ; \
	pip install --upgrade bob.extension==2.3.4 ; \
	pip install --upgrade bob.blitz==2.0.11 ; \
	pip install --upgrade bob.core==2.1.6 ; \
	pip install --upgrade bob.sp==2.0.7 ; \
	pip install --upgrade bob.ap==2.1.1 ; \
	pip install --upgrade  -r requirements.txt ; \
	pip install --upgrade -e git+https://github.com/patengelbert/speaker-recognition.git@b4ec098a928f2197076f7e1063a58ad5b2926cb7#egg=SpeakerRecognition ; \
	. /opt/ros/$(ROS_DISTRO)/setup.sh ;  \
	cd $(BUILD_DIR)/src ; \
	catkin_init_workspace ; \
	cd $(BUILD_DIR) ; \
	catkin_make ; \
	)
# Add the project to the workspace using a symlink.
	-ln -s $(CI_SOURCE_PATH) $(BUILD_DIR)/src
	@echo "Setup Build complete"

install-dependencies:
	rm -f $(BUILD_DIR)/src/.rosinstall*
# package depdencies: install using rosdep.
	( \
	. /opt/ros/$(ROS_DISTRO)/setup.sh ; \
	. $(BUILD_DIR)/devel/setup.sh ; \
	wstool init $(BUILD_DIR)/src ; \
	cd $(BUILD_DIR)/src ; \
	if [ -f $(ROSINSTALL_FILE) ] ; then wstool merge $(ROSINSTALL_FILE) ; fi ; \
	wstool up ; \
	cd $(BUILD_DIR) ; \
	rosdep install -y --from-paths src --ignore-src --rosdistro $(ROS_DISTRO) $(OS_OVERRIDE) ; \
	)

build: install-dependencies
	@echo "Building"
# Symlink the virtual environment
	rm -rf $(BUILD_DIR)/build/catkin_pip_env
	-ln -s $(shell pwd)/env $(BUILD_DIR)/build/catkin_pip_env
	( \
	. $(BUILD_DIR)/build/catkin_pip_env/bin/activate ; \
	pip install --upgrade  -r requirements.txt ; \
	. /opt/ros/$(ROS_DISTRO)/setup.sh ; \
	. $(BUILD_DIR)/devel/setup.sh ; \
	cd $(BUILD_DIR) && \
	catkin_make $( [ -f $(CATKIN_OPTIONS) ] && cat $(CATKIN_OPTIONS) ) ; \
	)

full-build: clean setup-build build

test:
	( \
	. /opt/ros/$(ROS_DISTRO)/setup.sh ; \
	. $(BUILD_DIR)/build/catkin_pip_env/bin/activate ; \
	pip install --upgrade  -r requirements.txt ; \
	. $(BUILD_DIR)/devel/setup.sh ; \
	cd $(BUILD_DIR) ; \
	catkin_make run_tests ; \
	catkin_test_results ; \
	)

run:
	( \
	. /opt/ros/$(ROS_DISTRO)/setup.sh ; \
	roscore ; \
	)

docs:
	sphinx-build -aE docs build/docs > /dev/null

