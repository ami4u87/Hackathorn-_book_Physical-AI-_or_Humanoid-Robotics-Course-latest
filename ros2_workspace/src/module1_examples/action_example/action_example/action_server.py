from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from typing import Any, Optional


class FibonacciActionServer(Node):
    """
    A ROS 2 action server node that generates Fibonacci sequences.

    This node demonstrates the action-server communication pattern in ROS 2.
    It generates a Fibonacci sequence up to a specified order and provides feedback during execution.
    """

    def __init__(self) -> None:
        """
        Initialize the action server node.

        Creates an action server for the Fibonacci action on the 'fibonacci' action name.
        Sets up the execute_callback method to handle incoming action goals.
        """
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle: Fibonacci.Goal) -> Fibonacci.Result:
        """
        Execute callback that processes incoming action goals.

        Generates a Fibonacci sequence up to the requested order, providing feedback
        during the process and returning the complete sequence as the result.

        Args:
            goal_handle: The action goal handle containing the request order

        Returns:
            The result containing the complete Fibonacci sequence
        """
        self.get_logger().info('Executing goal...')

        # Initialize the sequence with the first two Fibonacci numbers
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Publish initial feedback
        goal_handle.publish_feedback(feedback_msg)

        # Generate the Fibonacci sequence up to the requested order
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            self.get_logger().info(f'Publishing feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info(f'Returning result: {result.sequence}')

        return result


def main(args: Optional[Any] = None) -> None:
    """
    Main function to initialize and run the action server node.

    Initializes the ROS 2 communication, creates the action server node,
    spins the node to process incoming action goals, and properly shuts down.

    Args:
        args: Optional arguments passed to the ROS 2 initialization
    """
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()