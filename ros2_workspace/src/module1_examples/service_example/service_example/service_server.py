from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from typing import Any, Optional


class MinimalService(Node):
    """
    A minimal ROS 2 service server node that adds two integers.

    This node demonstrates the service-server communication pattern in ROS 2.
    It provides an 'add_two_ints' service that takes two integers and returns their sum.
    """

    def __init__(self) -> None:
        """
        Initialize the service server node.

        Creates a service for adding two integers on the 'add_two_ints' service name.
        Sets up the add_two_ints_callback method to handle incoming requests.
        """
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request: AddTwoInts.Request, response: AddTwoInts.Response) -> AddTwoInts.Response:
        """
        Callback function that processes incoming service requests.

        Adds the two integers from the request and returns the sum in the response.

        Args:
            request: The incoming service request containing two integers (a and b)
            response: The service response to be filled with the sum

        Returns:
            The response with the sum of the two input integers
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}\n')
        return response


def main(args: Optional[Any] = None) -> None:
    """
    Main function to initialize and run the service server node.

    Initializes the ROS 2 communication, creates the service server node,
    spins the node to process incoming service requests, and properly shuts down.

    Args:
        args: Optional arguments passed to the ROS 2 initialization
    """
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()