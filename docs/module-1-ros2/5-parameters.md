# Parameters

Parameters in ROS 2 provide a way to configure nodes dynamically at runtime. They allow you to modify node behavior without recompiling code, making your robotic systems more flexible and easier to tune. Parameters can be set at launch time, changed during runtime, and even declared with specific types and constraints.

## Understanding Parameters

Parameters are key-value pairs that can be associated with a node. They provide a way to:
- Configure node behavior without recompilation
- Tune algorithm parameters during operation
- Set initial conditions or configuration values
- Adapt to different environments or hardware

Parameters can be of various types:
- Integer (`int`)
- Double (`float`)
- String (`str`)
- Boolean (`bool`)
- Lists of the above types

## Declaring and Using Parameters

### Python Parameter Example

Let's create a node that uses parameters. First, create a new package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_parameters
```

Create a parameter node at `src/py_parameters/py_parameters/parameter_node.py`:

```python
import rclpy
from rclpy.node import Node


class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter('robot_name', 'turtlebot4')
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('sensors', ['imu', 'lidar', 'camera'])

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.sensors = self.get_parameter('sensors').value

        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Max velocity: {self.max_velocity}')
        self.get_logger().info(f'Use sim time: {self.use_sim_time}')
        self.get_logger().info(f'Sensors: {self.sensors}')

        # Set up a parameter callback to handle dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.type_ == Parameter.Type.DOUBLE:
                if param.value > 2.0:
                    self.get_logger().warn('Max velocity is set too high!')
                    return SetParametersResult(successful=False, reason='Velocity too high')

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    parameter_node = ParameterNode()

    # Example of setting a parameter from within the node
    parameter_node.set_parameters([Parameter('max_velocity', Parameter.Type.DOUBLE, 1.0)])

    rclpy.spin(parameter_node)

    parameter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Note: We need to import the Parameter class and SetParametersResult:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult


class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter('robot_name', 'turtlebot4',
                              ParameterDescriptor(description='Name of the robot'))
        self.declare_parameter('max_velocity', 0.5,
                              ParameterDescriptor(description='Maximum linear velocity (m/s)'))
        self.declare_parameter('use_sim_time', False,
                              ParameterDescriptor(description='Use simulation time'))
        self.declare_parameter('sensors', ['imu', 'lidar', 'camera'],
                              ParameterDescriptor(description='List of sensor types'))

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.sensors = self.get_parameter('sensors').value

        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Max velocity: {self.max_velocity}')
        self.get_logger().info(f'Use sim time: {self.use_sim_time}')
        self.get_logger().info(f'Sensors: {self.sensors}')

        # Set up a parameter callback to handle dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.type_ == Parameter.Type.DOUBLE:
                if param.value > 2.0:
                    self.get_logger().warn('Max velocity is set too high!')
                    result = SetParametersResult()
                    result.successful = False
                    result.reason = 'Velocity too high'
                    return result

        result = SetParametersResult()
        result.successful = True
        return result


def main(args=None):
    rclpy.init(args=args)

    parameter_node = ParameterNode()

    rclpy.spin(parameter_node)

    parameter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Setting Parameters at Launch Time

You can set parameters when launching a node using launch files. Create a launch file at `src/py_parameters/launch/parameter_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_parameters',
            executable='parameter_node',
            name='parameter_node',
            parameters=[
                {'robot_name': 'husky'},
                {'max_velocity': 1.0},
                {'use_sim_time': True},
                {'sensors': ['lidar', 'camera', 'gps']}
            ]
        )
    ])
```

## Setting Parameters from Command Line

You can also set parameters directly from the command line:

```bash
# Run the node with specific parameters
ros2 run py_parameters parameter_node --ros-args -p robot_name:=jackal -p max_velocity:=2.0

# Or use the parameter server
ros2 param set /parameter_node max_velocity 1.5

# Get parameter value
ros2 param get /parameter_node robot_name

# List all parameters for a node
ros2 param list /parameter_node
```

## Parameter Files

You can also define parameters in YAML files and load them at launch. Create a parameter file at `src/py_parameters/config/params.yaml`:

```yaml
parameter_node:
  ros__parameters:
    robot_name: 'turtlebot4'
    max_velocity: 0.5
    use_sim_time: false
    sensors: ['imu', 'lidar', 'camera']
    wheel_diameter: 0.1
    max_acceleration: 1.0
```

Then load it in your launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('py_parameters'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='py_parameters',
            executable='parameter_node',
            name='parameter_node',
            parameters=[config]
        )
    ])
```

## Advanced Parameter Concepts

### Parameter Descriptors

You can add metadata to parameters using descriptors:

```python
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import IntegerRange, FloatingPointRange

# For integer parameters with range
self.declare_parameter(
    'wheel_count',
    4,
    ParameterDescriptor(
        description='Number of wheels on the robot',
        integer_range=[IntegerRange(from_value=2, to_value=8, step=2)]
    )
)

# For floating point parameters with range
self.declare_parameter(
    'max_velocity',
    0.5,
    ParameterDescriptor(
        description='Maximum linear velocity in m/s',
        floating_point_range=[FloatingPointRange(from_value=0.0, to_value=5.0, step=0.1)]
    )
)
```

### Parameter Callbacks with Validation

You can implement complex validation logic in parameter callbacks:

```python
def parameter_callback(self, params):
    successful = True
    reason = ''

    for param in params:
        if param.name == 'max_velocity':
            if param.value <= 0:
                successful = False
                reason = 'Max velocity must be positive'
                break
            elif param.value > 5.0:
                successful = False
                reason = 'Max velocity must be <= 5.0 m/s'
                break
        elif param.name == 'robot_name':
            if not isinstance(param.value, str) or len(param.value) == 0:
                successful = False
                reason = 'Robot name must be a non-empty string'
                break

    return SetParametersResult(successful=successful, reason=reason)
```

## Best Practices

1. **Use descriptive parameter names**: Choose clear, consistent names that indicate the parameter's purpose.

2. **Provide meaningful defaults**: Set reasonable default values that work in most scenarios.

3. **Validate parameters**: Use parameter callbacks to validate values and prevent invalid configurations.

4. **Document parameters**: Use parameter descriptors to document the purpose and constraints of each parameter.

5. **Group related parameters**: Organize related parameters logically, especially in configuration files.

6. **Use appropriate types**: Choose the correct parameter type for your data to ensure type safety.

7. **Consider parameter ranges**: For numeric parameters, consider setting reasonable ranges to prevent invalid values.

## Command Line Tools

ROS 2 provides several command-line tools for working with parameters:

```bash
# List all parameters for a node
ros2 param list <node_name>

# Get a specific parameter value
ros2 param get <node_name> <param_name>

# Set a parameter value
ros2 param set <node_name> <param_name> <value>

# Describe parameters
ros2 param describe <node_name> <param_name>

# Load parameters from a file
ros2 param load <node_name> <param_file.yaml>

# Dump parameters to a file
ros2 param dump <node_name>
```

## Summary

Parameters provide a flexible way to configure nodes at runtime without recompiling code. They are essential for creating adaptable robotic systems that can be tuned for different environments, robots, or operational requirements. Understanding how to properly declare, use, and validate parameters is crucial for building robust ROS 2 applications.

In the next section, we'll explore launch files, which provide a way to start multiple nodes with specific configurations simultaneously.