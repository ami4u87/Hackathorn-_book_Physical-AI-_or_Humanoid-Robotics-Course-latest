# ROS 2 Workspace for Physical AI Course

This workspace contains example packages for the Physical AI & Humanoid Robotics course, demonstrating fundamental ROS 2 concepts.

## Package Structure

The workspace includes the following example packages:

- `publisher_example`: Demonstrates publisher-subscriber communication pattern
- `subscriber_example`: Demonstrates subscriber node implementation
- `service_example`: Demonstrates service-server and service-client pattern
- `action_example`: Demonstrates action-client and action-server pattern
- `param_example`: Demonstrates ROS 2 parameter system
- `launch_example`: Demonstrates launch file functionality
- `tf2_example`: Demonstrates TF2 (Transform Library) for coordinate frame management

## Prerequisites

- ROS 2 Humble Hawksbill (Ubuntu 22.04)
- Python 3.8+
- colcon build system

## Building the Workspace

```bash
# Source ROS 2 installation
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd ros2_workspace

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## Running Examples

### Publisher-Subscriber Example
```bash
# Terminal 1: Start the publisher
ros2 run publisher_example publisher_node

# Terminal 2: Start the subscriber
ros2 run subscriber_example subscriber_node
```

### Service Example
```bash
# Terminal 1: Start the service server
ros2 run service_example service_server

# Terminal 2: Call the service
ros2 run service_example service_client
```

### Action Example
```bash
# Terminal 1: Start the action server
ros2 run action_example action_server

# Terminal 2: Send an action goal
ros2 run action_example action_client
```

### Parameter Example
```bash
# Start the parameter node
ros2 run param_example param_node

# Change parameters dynamically
ros2 param set /parameter_node max_velocity 1.0
```

### Launch Example
```bash
# Launch nodes using launch file
ros2 launch launch_example simple_launch.py
```

### TF2 Example
```bash
# Launch TF2 broadcaster and listener
ros2 launch tf2_example tf2_example_launch.py

# View TF tree
rqt_tf_tree
```

## Docker Build

To build and run the examples in a Docker container:

```bash
# Build the Docker image
docker build -f Dockerfile.ros2-humble -t physical-ai-ros2 .

# Run the container
docker run -it physical-ai-ros2

# Inside the container, you can run any of the examples above
```

## Troubleshooting

- If you encounter build errors, make sure all dependencies are installed with `rosdep install`
- Ensure your ROS 2 environment is properly sourced before building
- Check that your PYTHONPATH includes the workspace installation directory