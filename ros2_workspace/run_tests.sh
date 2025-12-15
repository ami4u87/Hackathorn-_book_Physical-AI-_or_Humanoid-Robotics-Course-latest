#!/bin/bash

# Script to run all ROS 2 example validation tests

set -e  # Exit on any error

echo "ROS 2 Examples Validation Script"
echo "==============================="

# Check if we're in the right directory
if [ ! -f "src/module1_examples/publisher_example/package.xml" ]; then
    echo "Error: Not in the ROS 2 workspace directory"
    exit 1
fi

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS 2 Humble sourced"
else
    echo "Error: ROS 2 Humble not found"
    exit 1
fi

# Source the workspace if it exists
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✓ Workspace sourced"
fi

# Install dependencies if not already installed
echo "Checking dependencies..."
rosdep update || echo "Warning: rosdep update failed"
rosdep install --from-paths src --ignore-src -r -y || echo "Warning: Could not install all dependencies"

# Build the workspace if not already built
if [ ! -d "install" ]; then
    echo "Building workspace..."
    colcon build --packages-select \
        publisher_example \
        subscriber_example \
        service_example \
        action_example \
        param_example \
        launch_example \
        tf2_example
else
    echo "Workspace already built"
fi

# Source the workspace again after build
source install/setup.bash

# Run the Python test script
echo "Running validation tests..."
python3 test_examples.py

echo "Tests completed!"