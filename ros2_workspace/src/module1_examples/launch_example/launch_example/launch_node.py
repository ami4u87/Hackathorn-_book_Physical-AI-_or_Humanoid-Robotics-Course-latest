import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import Any, Optional


class LaunchNode(Node):
    """
    A simple ROS 2 node that demonstrates launch file functionality.
    This node publishes messages at a configurable rate.
    """

    def __init__(self) -> None:
        """
        Initialize the launch node.

        Declares parameters for publish rate and topic name, creates a publisher,
        sets up a timer based on the publish rate, and logs initialization info.
        """
        super().__init__('launch_node')

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('topic_name', 'launch_example_topic')

        # Get parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        self.topic_name = self.get_parameter('topic_name').value

        # Create publisher
        self.publisher = self.create_publisher(String, self.topic_name, 10)

        # Create timer based on publish rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.counter = 0
        self.get_logger().info(f'Launch node started - Publishing to {self.topic_name} at {self.publish_rate} Hz')

    def timer_callback(self) -> None:
        """
        Timer callback that publishes messages.

        Creates a String message with a counter value, publishes it,
        logs the message to the console, and increments the counter.
        """
        msg = String()
        msg.data = f'Launch example message #{self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args: Optional[Any] = None) -> None:
    """
    Main function to initialize and run the launch node.

    Initializes the ROS 2 communication, creates the launch node,
    spins the node to process callbacks, and properly shuts down.

    Args:
        args: Optional arguments passed to the ROS 2 initialization
    """
    rclpy.init(args=args)

    launch_node = LaunchNode()

    try:
        rclpy.spin(launch_node)
    except KeyboardInterrupt:
        pass
    finally:
        launch_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()