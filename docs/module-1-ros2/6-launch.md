# Launch Files

Launch files in ROS 2 provide a way to start multiple nodes with specific configurations simultaneously. They allow you to define complex robotic systems with a single command, making it easy to reproduce experiments, switch between different configurations, and manage complex multi-node applications.

## Understanding Launch Files

Launch files replace the ROS 1 launch system with a more flexible, Python-based approach. They can:
- Start multiple nodes with specific parameters
- Set up remappings between topics
- Configure node-specific settings
- Handle node lifecycle management
- Include other launch files for modularity

## Creating Launch Files

### Python Launch Files

The recommended approach in ROS 2 is to use Python launch files. Let's create an example launch file.

First, create a launch directory in your package:

```bash
mkdir -p ~/ros2_ws/src/py_launch/launch
```

Create a launch file at `src/py_launch/launch/example_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('py_launch')

    # Example of loading parameters from a file
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # Launch a simple talker node
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_node',
            parameters=[
                # Example of setting parameters directly
                {'param_name': 'param_value'},
                # Example of loading from a file
                # config_file
            ],
            remappings=[
                # Example of remapping topics
                ('/chatter', '/custom_chatter')
            ],
            # Set node execution priority
            # prefix='nice -n 10',
            # Additional configurations
            output='screen',  # Output to screen
        ),

        # Launch a simple listener node
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener_node',
            # Set namespace for the node
            namespace='robot1',
            parameters=[
                {'use_sim_time': False}
            ],
            output='screen',
        ),

        # Launch our parameter node
        Node(
            package='py_parameters',
            executable='parameter_node',
            name='configurable_node',
            parameters=[
                {'robot_name': 'turtlebot4'},
                {'max_velocity': 0.75}
            ],
            output='screen',
        )
    ])
```

### Launch File with Conditional Logic

You can also create launch files with conditional logic:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot',
        description='Name of the robot'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        # Add the argument declarations
        sim_time_arg,
        robot_name_arg,

        # Launch node with configurations
        Node(
            package='py_parameters',
            executable='parameter_node',
            name='parameter_node',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_name': robot_name}
            ],
            output='screen',
        )
    ])
```

## Launch File Concepts

### Node Configuration Options

When defining nodes in launch files, you have several configuration options:

```python
Node(
    package='package_name',      # Package containing the executable
    executable='executable_name', # Name of the executable
    name='node_name',            # Name to give the node (overrides default)
    namespace='namespace',       # Namespace for the node
    parameters=[...],            # List of parameters
    remappings=[...],            # List of topic/service remappings
    arguments=[...],             # Command line arguments
    output='both',               # Output destination ('screen', 'log', 'both')
    condition=...                # Conditional launch (if/unless)
)
```

### Remappings

Remappings allow you to redirect topics between nodes:

```python
remappings=[
    ('/original_topic', '/new_topic'),
    ('/tf', '/robot1/tf'),  # Useful for multi-robot systems
]
```

### Conditional Launch

You can conditionally launch nodes based on arguments:

```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

# Example with conditional launch
Node(
    package='demo_nodes_cpp',
    executable='talker',
    name='conditional_talker',
    condition=IfCondition(LaunchConfiguration('start_talker'))
)
```

## Running Launch Files

To run a launch file:

```bash
# Basic launch
ros2 launch py_launch example_launch.py

# With arguments
ros2 launch py_launch example_launch.py use_sim_time:=true robot_name:=jackal

# With specific launch file from package
ros2 launch <package_name> <launch_file.py>
```

## Advanced Launch Concepts

### Including Other Launch Files

You can include other launch files for modularity:

```python
from launch import LaunchDescription, LaunchInclude
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include another launch file
    other_launch_file = os.path.join(
        get_package_share_directory('other_package'),
        'launch',
        'other_launch.py'
    )

    return LaunchDescription([
        # Include another launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(other_launch_file)
        ),

        # Your nodes here
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            output='screen',
        )
    ])
```

### Event Handlers

You can define event handlers for node lifecycle events:

```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    talker_node = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='talker',
    )

    listener_node = Node(
        package='demo_nodes_py',
        executable='listener',
        name='listener',
    )

    # Register event handler
    listener_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=talker_node,
            on_start=[
                # Start listener after talker starts
                listener_node
            ]
        )
    )

    return LaunchDescription([
        talker_node,
        listener_event_handler,
    ])
```

## Best Practices

1. **Modular launch files**: Break complex systems into smaller, reusable launch files.

2. **Use launch arguments**: Make your launch files configurable with launch arguments.

3. **Parameter files**: Use YAML parameter files for complex configurations.

4. **Namespace organization**: Use namespaces to organize multi-robot systems.

5. **Descriptive names**: Use clear, descriptive names for nodes and launch files.

6. **Error handling**: Use event handlers for robust error handling and recovery.

7. **Documentation**: Document your launch files with comments explaining the purpose of each node.

## Launch System Tools

ROS 2 provides command-line tools for working with launch systems:

```bash
# Show launch file description
ros2 launch --show-args <package_name> <launch_file.py>

# Dry run (show what would be launched without actually launching)
# This is done by examining the launch file directly

# List active launch processes
ps aux | grep -i launch
```

## Summary

Launch files provide a powerful way to configure and start complex robotic systems with a single command. They are essential for reproducible experiments and make it easy to switch between different configurations. Understanding how to create and use launch files effectively is crucial for working with real-world ROS 2 applications.

In the next section, we'll explore TF2 (Transform Library), which provides a way to keep track of coordinate frames in a robotic system over time.