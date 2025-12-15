from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description for the launch example node.

    Returns:
        LaunchDescription containing the launch configuration for the launch_example node
    """

    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Rate at which the launch node publishes messages'
    )

    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='launch_example_topic',
        description='Topic name for the launch node to publish to'
    )

    # Create the launch node
    launch_node = Node(
        package='launch_example',
        executable='launch_node',
        name='launch_node',
        parameters=[
            {'publish_rate': LaunchConfiguration('publish_rate')},
            {'topic_name': LaunchConfiguration('topic_name')}
        ],
        output='screen'
    )

    return LaunchDescription([
        publish_rate_arg,
        topic_name_arg,
        launch_node
    ])