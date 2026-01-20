#!/bin/bash

# Check for root permissions
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run as root" >&2
    exit 1
fi

# Source the ROS environment
# Replace '/opt/ros/noetic' with your actual ROS distribution path.
source /opt/ros/noetic/setup.bash
# Replace '/home/ara/bunker_ws' with your actual ROS ws path.
source /home/ara/bunker_ws/devel/setup.bash

# Load the gs_usb kernel module if not already loaded
if ! lsmod | grep -q gs_usb; then
    echo "Loading gs_usb module..."
    sudo modprobe gs_usb
else
    echo "gs_usb module already loaded."
fi

# Set up the can0 device with a specified bitrate if not already set
# Check if can0 is already up with the correct settings
current_bitrate=$(ip -details link show can0 | grep -oP 'bitrate \K[^ ]+' || echo "down")

if [[ $current_bitrate != "500000" ]]; then
    echo "Setting up CAN0 interface..."
    sudo ip link set can0 up type can bitrate 500000
    if [ $? -eq 0 ]; then
        echo "CAN0 interface setup successful."
    else
        echo "Failed to set up CAN0 interface." >&2
        exit 2
    fi
else
    echo "CAN0 interface already set up with correct bitrate."
fi

# Launch the ROS nodes
echo "Launching ROS nodes..."
roslaunch bunker_bringup bunker_robot_base.launch
