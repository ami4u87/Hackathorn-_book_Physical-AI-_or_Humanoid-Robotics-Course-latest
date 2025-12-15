import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from typing import Any, Optional
import math


class StaticFramePublisher(Node):
    """
    A node that publishes a static transform to the TF2 tree.
    This node creates a transform between 'world' and 'robot1' frames.
    """

    def __init__(self) -> None:
        """
        Initialize the static frame publisher node.

        Declares parameters for transform values, creates a transform broadcaster,
        sets up a timer to broadcast the static transform, and logs initialization info.
        """
        super().__init__('static_frame_publisher')

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('translation_x', 0.0)
        self.declare_parameter('translation_y', 0.0)
        self.declare_parameter('translation_z', 0.0)
        self.declare_parameter('rotation_x', 0.0)
        self.declare_parameter('rotation_y', 0.0)
        self.declare_parameter('rotation_z', 0.0)
        self.declare_parameter('rotation_w', 1.0)

        # Get parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        self.translation_x = self.get_parameter('translation_x').value
        self.translation_y = self.get_parameter('translation_y').value
        self.translation_z = self.get_parameter('translation_z').value
        self.rotation_x = self.get_parameter('rotation_x').value
        self.rotation_y = self.get_parameter('rotation_y').value
        self.rotation_z = self.get_parameter('rotation_z').value
        self.rotation_w = self.get_parameter('rotation_w').value

        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timer to broadcast transform
        self.timer = self.create_timer(1.0 / self.publish_rate, self.broadcast_transform)

        self.get_logger().info('Static frame publisher node started')

    def broadcast_transform(self) -> None:
        """
        Broadcast the static transform.

        Creates a TransformStamped message with the configured transform values
        and publishes it to the TF2 tree.
        """
        t = TransformStamped()

        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot1'

        # Set transform values
        t.transform.translation.x = self.translation_x
        t.transform.translation.y = self.translation_y
        t.transform.translation.z = self.translation_z

        t.transform.rotation.x = self.rotation_x
        t.transform.rotation.y = self.rotation_y
        t.transform.rotation.z = self.rotation_z
        t.transform.rotation.w = self.rotation_w

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)


class DynamicFramePublisher(Node):
    """
    A node that publishes a dynamic transform to the TF2 tree.
    This node creates a transform between 'robot1' and 'robot1_moving' frames
    that changes over time to demonstrate dynamic transformations.
    """

    def __init__(self) -> None:
        """
        Initialize the dynamic frame publisher node.

        Declares parameters for transform dynamics, creates a transform broadcaster,
        sets up a timer to broadcast the dynamic transform, and logs initialization info.
        """
        super().__init__('dynamic_frame_publisher')

        # Declare parameters
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('amplitude', 2.0)
        self.declare_parameter('frequency', 1.0)

        # Get parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value

        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timer to broadcast transform
        self.timer = self.create_timer(1.0 / self.publish_rate, self.broadcast_transform)

        self.time = 0.0
        self.get_logger().info('Dynamic frame publisher node started')

    def broadcast_transform(self) -> None:
        """
        Broadcast the dynamic transform.

        Creates a TransformStamped message with dynamically calculated transform values
        based on oscillating motion and publishes it to the TF2 tree.
        """
        t = TransformStamped()

        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'robot1'
        t.child_frame_id = 'robot1_moving'

        # Calculate dynamic position (oscillating motion)
        self.time += 1.0 / self.publish_rate
        x = self.amplitude * math.sin(self.frequency * self.time)
        y = self.amplitude * math.cos(self.frequency * self.time)

        # Set transform values
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # Simple rotation around z-axis
        angle = self.frequency * self.time
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(angle / 2.0)
        t.transform.rotation.w = math.cos(angle / 2.0)

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)


def main(args: Optional[Any] = None) -> None:
    """
    Main function to initialize and run the TF2 broadcaster nodes.

    Initializes the ROS 2 communication, creates both static and dynamic frame publishers,
    spins the nodes to broadcast transforms, and properly shuts down.

    Args:
        args: Optional arguments passed to the ROS 2 initialization
    """
    rclpy.init(args=args)

    # Create both static and dynamic frame publishers
    static_publisher = StaticFramePublisher()
    dynamic_publisher = DynamicFramePublisher()

    try:
        rclpy.spin(static_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        static_publisher.destroy_node()
        dynamic_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()