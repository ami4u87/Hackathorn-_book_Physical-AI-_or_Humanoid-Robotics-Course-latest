import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import Any, Optional


class MinimalPublisher(Node):
    """
    A minimal ROS 2 publisher node that publishes 'Hello World' messages.

    This node demonstrates the publisher-subscriber communication pattern in ROS 2.
    It publishes String messages to a topic at a fixed interval.
    """

    def __init__(self) -> None:
        """
        Initialize the publisher node.

        Creates a publisher for String messages on the 'example_topic' topic with a queue size of 10.
        Sets up a timer to call the timer_callback method every 0.5 seconds.
        """
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'example_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self) -> None:
        """
        Timer callback that publishes a message to the topic.

        Creates a String message with a counter value, publishes it,
        logs the message to the console, and increments the counter.
        """
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args: Optional[Any] = None) -> None:
    """
    Main function to initialize and run the publisher node.

    Initializes the ROS 2 communication, creates the publisher node,
    spins the node to process callbacks, and properly shuts down.

    Args:
        args: Optional arguments passed to the ROS 2 initialization
    """
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()