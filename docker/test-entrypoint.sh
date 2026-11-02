#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/mppi_ws/install/local_setup.bash
exec "$@"
