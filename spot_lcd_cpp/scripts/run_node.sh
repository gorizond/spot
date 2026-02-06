#!/bin/bash

# Script to select and run the appropriate ROS2 node based on command line argument
NODE_TYPE=${1:-"combined"}  # Default to "combined" if no argument provided

# Source ROS2 environment
source /opt/ros/humble/setup.bash

case "$NODE_TYPE" in
    "lcd")
        echo "Starting LCD node..."
        exec spot_lcd_cpp_lcd_node
        ;;
    "temp"|"temperature")
        echo "Starting Temperature node..."
        exec spot_lcd_cpp_temp_node
        ;;
    "uptime")
        echo "Starting Uptime node..."
        exec spot_lcd_cpp_uptime_node
        ;;
    "all"|"combined")
        echo "Starting combined node..."
        exec spot_lcd_cpp_node -m all
        ;;
    *)
        echo "Unknown node type: $NODE_TYPE"
        echo "Usage: $0 [lcd|temp|temperature|uptime|all|combined]"
        echo "Available nodes: lcd, temp/temperature, uptime, all/combined"
        exit 1
        ;;
esac