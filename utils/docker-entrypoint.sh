#!/bin/bash
set -e

# Source ROS environment + workspace overlay (includes all packages)
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source ${WORKSPACE}/install/setup.bash
fi

# Execute the command passed to the container
exec "$@"
