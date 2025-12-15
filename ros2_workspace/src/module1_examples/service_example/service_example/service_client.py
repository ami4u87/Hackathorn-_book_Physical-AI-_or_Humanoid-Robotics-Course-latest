from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from typing import Any, Optional


class MinimalClientAsync(Node):
    """
    A minimal ROS 2 service client node that calls the add_two_ints service.

    This node demonstrates the service-client communication pattern in ROS 2.
    It calls the 'add_two_ints' service with two integers and receives their sum.
    """

    def __init__(self) -> None:
        """
        Initialize the service client node.

        Creates a client for the 'add_two_ints' service and waits for the service to be available.
        Initializes a request object for the service calls.
        """
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a: int, b: int) -> AddTwoInts.Response:
        """
        Send a request to the add_two_ints service.

        Sets the values of a and b in the request object, calls the service asynchronously,
        waits for the response, and returns it.

        Args:
            a: First integer to add
            b: Second integer to add

        Returns:
            The response from the service containing the sum of a and b
        """
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args: Optional[Any] = None) -> None:
    """
    Main function to initialize and run the service client node.

    Initializes the ROS 2 communication, creates the service client node,
    sends a request to the service, logs the result, and properly shuts down.

    Args:
        args: Optional arguments passed to the ROS 2 initialization
    """
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        f'Result of add_two_ints: for {1} + {2} = {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()