#!/bin/bash
set -e

# Setup the Space ROS environment
source "${SPACEROS_DIR}/install/setup.bash"
export IKOS_SCAN_NOTIFIER_FILES="" # make ikos create .ikosbin files for compiled packages
source "/opt/ros/$ROS_DISTRO/setup.bash" --
cd /workspace/omnilrs
exec "$@"