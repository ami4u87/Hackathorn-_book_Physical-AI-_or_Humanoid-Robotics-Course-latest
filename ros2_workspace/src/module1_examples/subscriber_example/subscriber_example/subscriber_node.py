import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import Any, Optional


class MinimalSubscriber(Node):
    """
    A minimal ROS 2 subscriber node that listens to String messages.

    This node demonstrates the publisher-subscriber communication pattern in ROS 2.
    It subscribes to String messages on the 'example_topic' topic and logs received messages.
    """

    def __init__(self) -> None:
        """
        Initialize the subscriber node.

        Creates a subscription to String messages on the 'example_topic' topic with a queue size of 10.
        Sets up the listener_callback method to handle incoming messages.
        """
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'example_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: String) -> None:
        """
        Callback function that processes incoming messages.

        Logs the received message data to the console.

        Args:
            msg: The incoming String message
        """
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args: Optional[Any] = None) -> None:
    """
    Main function to initialize and run the subscriber node.

    Initializes the ROS 2 communication, creates the subscriber node,
    spins the node to process incoming messages, and properly shuts down.

    Args:
        args: Optional arguments passed to the ROS 2 initialization
    """
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()