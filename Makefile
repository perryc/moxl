REMOTE_HOST ?= moxl.local
REMOTE_USER ?= moxl
ROS_LOG_DIR = log/
SHELL := /bin/bash

all: deps build

.PHONY: deps build

deps:
	rosdep install --from-paths ./ -i -y -r

custom-deps:
	sh utils/install-custom-deps.sh

build-libs:
	colcon build --base-paths "src/lib/*"

build:
	colcon build --symlink-install

build-release:
	colcon build --base-paths "src/lib/*" --cmake-args -DCMAKE_BUILD_TYPE=Release
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

sim:
	ros2 launch moxl sim.launch.py

run:
	ros2 launch moxl moxl.launch.py

teleop:
	ros2 launch moxl teleop.launch.py

run-foxglove:
	ros2 launch foxglove_bridge foxglove_bridge_launch.xml

rsp:
	ros2 launch moxl rsp.launch.py

test:
	cd src/toolpath_planner && python3 -m pytest test/ -v
