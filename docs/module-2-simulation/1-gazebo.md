# Gazebo Harmonic with ROS 2

Gazebo is a powerful 3D robotics simulator that provides accurate physics simulation and sensor modeling. In this lesson, you'll learn to set up Gazebo Harmonic (the latest version) with ROS 2 and simulate humanoid robots.

## What is Gazebo Harmonic?

Gazebo Harmonic is the latest generation of the Gazebo simulator, featuring:

- **Modern Architecture**: Complete rewrite with improved performance
- **Photorealistic Rendering**: Advanced graphics using Ogre 2.x
- **Physics Engines**: Support for multiple physics engines (Bullet, DART, ODE)
- **Sensor Simulation**: Cameras, LiDAR, IMU, force/torque sensors
- **ROS 2 Integration**: Native support via ros_gz packages

## Installation

### Prerequisites

Before installing Gazebo Harmonic, ensure you have:
- Ubuntu 22.04 (Jammy) or later
- ROS 2 Humble or later
- At least 4GB RAM and a dedicated GPU (recommended)

### Installing Gazebo Harmonic

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install
sudo apt update
sudo apt install gz-harmonic
```

### Installing ROS 2 - Gazebo Bridge

```bash
# Install ros_gz packages
sudo apt install ros-humble-ros-gz

# Source ROS 2
source /opt/ros/humble/setup.bash
```

## Launching Gazebo

### Basic Launch

```bash
# Launch empty world
gz sim empty.sdf

# Launch with a specific world file
gz sim worlds/shapes.sdf
```

### With ROS 2 Bridge

Create a launch file `gazebo_sim.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to Gazebo ROS package
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Bridge ROS topics and Gazebo topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
    ])
```

Launch with:
```bash
ros2 launch your_package gazebo_sim.launch.py
```

## Creating a Humanoid Robot Model

### SDF Format

Gazebo uses SDF (Simulation Description Format) for robot models. Here's a simplified humanoid structure:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="simple_humanoid">
    <!-- Torso -->
    <link name="torso">
      <pose>0 0 1.0 0 0 0</pose>
      <inertial>
        <mass>30.0</mass>
        <inertia>
          <ixx>1.0</ixx>
          <iyy>1.0</iyy>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <collision name="torso_collision">
        <geometry>
          <box>
            <size>0.4 0.3 0.6</size>
          </box>
        </geometry>
      </collision>
      <visual name="torso_visual">
        <geometry>
          <box>
            <size>0.4 0.3 0.6</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Head -->
    <link name="head">
      <pose>0 0 1.45 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <iyy>0.1</iyy>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <collision name="head_collision">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="head_visual">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.9 0.7 0.6 1</ambient>
          <diffuse>0.9 0.7 0.6 1</diffuse>
        </material>
      </visual>

      <!-- Camera Sensor -->
      <sensor name="head_camera" type="camera">
        <pose>0.1 0 0 0 0 0</pose>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
      </sensor>
    </link>

    <!-- Neck Joint -->
    <joint name="neck_joint" type="revolute">
      <parent>torso</parent>
      <child>head</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
        </limit>
      </axis>
    </joint>

    <!-- Add arms, legs, and other components... -->

  </model>
</sdf>
```

## Spawning Robots in Gazebo

### Method 1: Using Launch Files

```python
from launch_ros.actions import Node

def generate_launch_description():
    # ... previous gazebo launch code ...

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_humanoid',
            '-file', '/path/to/humanoid.sdf',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        spawn_robot,
    ])
```

### Method 2: Using ROS 2 Service

```bash
# Spawn using service call
ros2 service call /world/empty/create entity_name:="my_humanoid" file_path:="/path/to/humanoid.sdf" pose:="{position: {x: 0.0, y: 0.0, z: 1.0}}"
```

## Controlling Robots in Gazebo

### Joint Control Plugin

Add to your SDF model:

```xml
<plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
  <joint_name>neck_joint</joint_name>
</plugin>
```

### Publishing Joint Commands

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher = self.create_publisher(
            Float64,
            '/model/my_humanoid/joint/neck_joint/cmd_pos',
            10
        )
        self.timer = self.create_timer(0.1, self.publish_command)
        self.angle = 0.0

    def publish_command(self):
        msg = Float64()
        msg.data = self.angle
        self.publisher.publish(msg)
        # Oscillate neck
        self.angle += 0.05
        if abs(self.angle) > 1.0:
            self.angle *= -1

def main():
    rclpy.init()
    node = JointCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Integration

### Camera Feed to ROS 2

Bridge camera data:

```python
# In your launch file, add to bridge arguments:
bridge_args = [
    '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
    '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
]

bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=bridge_args,
    output='screen'
)
```

### IMU Sensor

Add IMU to your robot:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </x>
      <!-- Add y and z... -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <!-- Add y and z... -->
    </linear_acceleration>
  </imu>
</sensor>
```

## Best Practices

1. **Model Organization**: Keep robot models in separate packages with clear structure
2. **Physics Tuning**: Adjust physics parameters (step size, iterations) for stability
3. **Sensor Noise**: Add realistic noise to sensors for robust algorithm development
4. **Performance**: Use lower update rates for non-critical sensors
5. **Version Control**: Track SDF models and world files in version control

## Troubleshooting

### Common Issues

**Gazebo crashes on launch:**
```bash
# Check graphics driver
glxinfo | grep "OpenGL version"

# Try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
gz sim
```

**Robot falls through ground:**
- Check collision geometries
- Verify mass and inertia values
- Ensure proper contact parameters

**Slow simulation:**
- Reduce sensor update rates
- Simplify collision meshes
- Disable unnecessary plugins
- Lower real-time factor

## Next Steps

In the next lesson, you'll learn to set up Unity with the ROS-TCP-Connector for more advanced graphics and simulation scenarios.

## Further Reading

- [Gazebo Documentation](https://gazebosim.org/docs)
- [SDF Specification](http://sdformat.org/)
- [ros_gz Repository](https://github.com/gazebosim/ros_gz)
