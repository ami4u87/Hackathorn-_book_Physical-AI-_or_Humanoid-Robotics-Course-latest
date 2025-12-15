from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description for the TF2 example nodes.

    Returns:
        LaunchDescription containing the launch configuration for TF2 broadcaster and listener nodes
    """

    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate at which the TF2 nodes publish transforms'
    )

    amplitude_arg = DeclareLaunchArgument(
        'amplitude',
        default_value='2.0',
        description='Amplitude of the dynamic transform motion'
    )

    # Create the broadcaster node
    broadcaster_node = Node(
        package='tf2_example',
        executable='tf2_broadcaster',
        name='tf2_broadcaster',
        parameters=[
            {'publish_rate': LaunchConfiguration('publish_rate')},
            {'amplitude': LaunchConfiguration('amplitude')},
        ],
        output='screen'
    )

    # Create the listener node
    listener_node = Node(
        package='tf2_example',
        executable='tf2_listener',
        name='tf2_listener',
        parameters=[
            {'publish_rate': LaunchConfiguration('publish_rate')},
        ],
        output='screen'
    )

    return LaunchDescription([
        publish_rate_arg,
        amplitude_arg,
        broadcaster_node,
        listener_node
    ])