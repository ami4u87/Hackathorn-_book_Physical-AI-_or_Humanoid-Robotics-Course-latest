from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from typing import Any, Optional


class FibonacciActionClient(Node):
    """
    A ROS 2 action client node that sends Fibonacci sequence generation goals.

    This node demonstrates the action-client communication pattern in ROS 2.
    It sends a goal to the Fibonacci action server to generate a sequence of a specified order.
    """

    def __init__(self) -> None:
        """
        Initialize the action client node.

        Creates an action client for the Fibonacci action on the 'fibonacci' action name.
        """
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order: int) -> None:
        """
        Send a goal to the Fibonacci action server.

        Creates a goal message with the specified order, waits for the server,
        sends the goal asynchronously with a feedback callback, and sets up
        callbacks for goal response and result.

        Args:
            order: The order of the Fibonacci sequence to generate
        """
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future) -> None:
        """
        Callback function that handles the goal response from the server.

        Checks if the goal was accepted, and if so, sets up the result callback.

        Args:
            future: The future containing the goal handle
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg: Fibonacci.Feedback) -> None:
        """
        Callback function that handles feedback from the action server.

        Logs the partial sequence received as feedback during action execution.

        Args:
            feedback_msg: The feedback message containing the partial sequence
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future: Future) -> None:
        """
        Callback function that handles the final result from the action server.

        Logs the final Fibonacci sequence result and shuts down the ROS 2 communication.

        Args:
            future: The future containing the result message
        """
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()


def main(args: Optional[Any] = None) -> None:
    """
    Main function to initialize and run the action client node.

    Initializes the ROS 2 communication, creates the action client node,
    sends a goal to the action server, and spins to process callbacks.

    Args:
        args: Optional arguments passed to the ROS 2 initialization
    """
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()