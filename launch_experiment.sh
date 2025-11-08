#!/bin/bash
# Quick reference script for launching experiments
# This script provides examples of how to launch each experiment type

set -e

# Source workspace
cd /home/vscode/workspace/packages
source install/setup.bash

echo "========================================="
echo "Experiments Package - Quick Reference"
echo "========================================="
echo ""
echo "Available experiments:"
echo "  1. Monte Carlo"
echo "  2. YOLO"
echo "  3. Interference"
echo ""
echo "Select an experiment to launch (1-3), or 0 to see all commands:"
read -p "Choice: " choice

case $choice in
    0)
        echo ""
        echo "========================================="
        echo "All Launch Commands"
        echo "========================================="
        echo ""
        echo "1. Monte Carlo Experiment:"
        echo "   ros2 launch experiments monte_carlo.launch.py"
        echo ""
        echo "   With parameters:"
        echo "   ros2 launch experiments monte_carlo.launch.py \\"
        echo "       use_multi_threaded:=true \\"
        echo "       log_level:=debug"
        echo ""
        echo "2. YOLO Experiment:"
        echo "   ros2 launch experiments yolo.launch.py"
        echo ""
        echo "   With parameters:"
        echo "   ros2 launch experiments yolo.launch.py \\"
        echo "       use_multi_threaded:=true \\"
        echo "       log_level:=debug"
        echo ""
        echo "3. Interference Experiment:"
        echo "   ros2 launch experiments interference.launch.py"
        echo ""
        echo "   With parameters:"
        echo "   ros2 launch experiments interference.launch.py \\"
        echo "       use_multi_threaded:=true \\"
        echo "       timer_period_ms:=100 \\"
        echo "       execution_time_ms:=10 \\"
        echo "       log_level:=debug"
        echo ""
        ;;
    1)
        echo ""
        echo "Launching Monte Carlo experiment..."
        echo "Press Ctrl+C to stop"
        echo ""
        ros2 launch experiments monte_carlo.launch.py
        ;;
    2)
        echo ""
        echo "Launching YOLO experiment..."
        echo "Press Ctrl+C to stop"
        echo ""
        ros2 launch experiments yolo.launch.py
        ;;
    3)
        echo ""
        echo "Launching Interference experiment..."
        echo "Press Ctrl+C to stop"
        echo ""
        ros2 launch experiments interference.launch.py
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac
