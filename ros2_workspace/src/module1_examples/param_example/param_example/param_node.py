import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from typing import Any, Optional, List


class ParameterNode(Node):
    """
    A ROS 2 node that demonstrates parameter handling and dynamic reconfiguration.

    This node declares various parameters with default values and descriptions,
    and sets up a callback to validate parameter changes at runtime.
    """

    def __init__(self) -> None:
        """
        Initialize the parameter node.

        Declares parameters with default values and descriptions, gets their values,
        logs the parameter values, and sets up a parameter callback for validation.
        """
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

    def parameter_callback(self, params: List[Parameter]) -> SetParametersResult:
        """
        Callback function that validates parameter changes.

        Checks if the max_velocity parameter is being set to a value higher than 2.0
        and rejects the change if so. Otherwise, accepts the parameter change.

        Args:
            params: List of parameters being set

        Returns:
            SetParametersResult indicating whether the parameter change was successful
        """
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


def main(args: Optional[Any] = None) -> None:
    """
    Main function to initialize and run the parameter node.

    Initializes the ROS 2 communication, creates the parameter node,
    spins the node to process parameter callbacks, and properly shuts down.

    Args:
        args: Optional arguments passed to the ROS 2 initialization
    """
    rclpy.init(args=args)

    parameter_node = ParameterNode()

    rclpy.spin(parameter_node)

    parameter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()