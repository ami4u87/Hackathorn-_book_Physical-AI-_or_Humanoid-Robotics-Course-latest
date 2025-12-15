import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
from typing import Any, Optional


class FrameListener(Node):
    """
    A node that listens to TF2 transforms and uses them to transform points.
    This node listens for transforms between 'world' and 'robot1_moving' frames
    and transforms a point from one frame to another.
    """

    def __init__(self) -> None:
        """
        Initialize the frame listener node.

        Creates a TF2 buffer and listener, sets up a publisher for transformed points,
        creates a timer to periodically look up transforms, initializes a point in
        robot1_moving frame, and logs initialization info.
        """
        super().__init__('frame_listener')

        # Create TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create publisher for transformed points
        self.pub = self.create_publisher(PointStamped, 'transformed_point', 10)

        # Create timer to periodically look up transforms
        self.timer = self.create_timer(1.0, self.lookup_transform)

        # Create a point in robot1_moving frame
        self.point = PointStamped()
        self.point.header.frame_id = 'robot1_moving'
        self.point.point.x = 1.0
        self.point.point.y = 0.0
        self.point.point.z = 0.0

        self.get_logger().info('Frame listener node started')

    def lookup_transform(self) -> None:
        """
        Look up transform and transform the point.

        Looks up the transform from 'robot1_moving' to 'world' frame,
        transforms the point using the transform, updates the header,
        publishes the transformed point, and logs the result.
        """
        try:
            # Look up the transform from 'robot1_moving' to 'world'
            transform = self.tf_buffer.lookup_transform(
                'world',
                'robot1_moving',
                Time(),
                timeout=Duration(seconds=1.0)
            )

            # Transform the point
            transformed_point = do_transform_point(self.point, transform)

            # Update header for the transformed point
            transformed_point.header.stamp = self.get_clock().now().to_msg()
            transformed_point.header.frame_id = 'world'

            # Publish the transformed point
            self.pub.publish(transformed_point)

            self.get_logger().info(
                f'Transformed point: ({transformed_point.point.x:.2f}, '
                f'{transformed_point.point.y:.2f}, {transformed_point.point.z:.2f}) '
                f'from frame {transformed_point.header.frame_id}'
            )

        except Exception as e:
            self.get_logger().warn(f'Could not transform point: {str(e)}')


def main(args: Optional[Any] = None) -> None:
    """
    Main function to initialize and run the frame listener node.

    Initializes the ROS 2 communication, creates the frame listener node,
    spins the node to process transform lookups, and properly shuts down.

    Args:
        args: Optional arguments passed to the ROS 2 initialization
    """
    rclpy.init(args=args)

    frame_listener = FrameListener()

    try:
        rclpy.spin(frame_listener)
    except KeyboardInterrupt:
        pass
    finally:
        frame_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()