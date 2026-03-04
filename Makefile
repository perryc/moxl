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
	colcon build --symlink-install --base-paths . src/toolpath_planner

build-release:
	colcon build --base-paths "src/lib/*" --cmake-args -DCMAKE_BUILD_TYPE=Release
	colcon build --symlink-install --base-paths . src/toolpath_planner --cmake-args -DCMAKE_BUILD_TYPE=Release

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

# ── WSL2 targets (run from Windows terminal) ─────────────────────
WSL_DISTRO ?= Ubuntu-24.04
WSL_MOXL   = /root/moxl
WSL_RUN    = wsl -d $(WSL_DISTRO) -- bash -lc

wsl-setup:
	wsl -d $(WSL_DISTRO) -- bash /mnt/c/Users/4perr/documents/github/moxl/utils/wsl-setup.sh

wsl-build:
	$(WSL_RUN) "cd $(WSL_MOXL) && colcon build --symlink-install --base-paths . src/toolpath_planner"

wsl-sim:
	$(WSL_RUN) "pkill -9 -f 'ros2|ruby|gz|parameter_bridge|robot_state' 2>/dev/null; sleep 1; cd $(WSL_MOXL) && source install/setup.bash && ros2 launch moxl sim.launch.py"

wsl-test:
	$(WSL_RUN) "cd $(WSL_MOXL) && source install/setup.bash && cd src/toolpath_planner && python3 -m pytest test/ -v"

wsl-teleop:
	$(WSL_RUN) "cd $(WSL_MOXL) && source install/setup.bash && ros2 launch moxl teleop.launch.py"

wsl-foxglove:
	$(WSL_RUN) "cd $(WSL_MOXL) && source install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml"

wsl-sync:
	wsl -d $(WSL_DISTRO) -- bash -c "rsync -a --exclude=build --exclude=install --exclude=log --exclude=.git /mnt/c/Users/4perr/documents/github/moxl/ $(WSL_MOXL)/"

wsl-shell:
	wsl -d $(WSL_DISTRO)
