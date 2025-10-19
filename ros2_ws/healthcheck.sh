#!/bin/bash
# Central healthcheck script for ROS2 containers
# Usage: ./healthcheck.sh <check_type> <pattern>
# check_type: node, topic, process
# pattern: string to grep for

set -e

CHECK_TYPE=$1
PATTERN=$2

if [ -z "$CHECK_TYPE" ] || [ -z "$PATTERN" ]; then
    echo "Usage: $0 <check_type> <pattern>"
    echo "check_type: node, topic, process"
    echo "pattern: string to grep for"
    exit 1
fi

case $CHECK_TYPE in
    node)
        # Check if ROS2 node exists
        source /opt/ros/humble/setup.bash
        if [ -f /ros2_ws/install/setup.bash ]; then
            source /ros2_ws/install/setup.bash
        fi
        ros2 node list 2>/dev/null | grep -q "$PATTERN"
        ;;
    topic)
        # Check if ROS2 topic exists
        source /opt/ros/humble/setup.bash
        if [ -f /ros2_ws/install/setup.bash ]; then
            source /ros2_ws/install/setup.bash
        fi
        ros2 topic list 2>/dev/null | grep -q "$PATTERN"
        ;;
    process)
        # Check if process is running
        pgrep -f "$PATTERN" > /dev/null
        ;;
    *)
        echo "Unknown check type: $CHECK_TYPE"
        echo "Valid types: node, topic, process"
        exit 1
        ;;
esac

exit $?
