# NVIDIA Isaac Sim

NVIDIA Isaac Sim is a robotics simulation platform built on NVIDIA Omniverse, providing photorealistic, physically accurate virtual environments with GPU-accelerated computation. It's designed specifically for robotic applications with advanced features for AI and machine learning.

## What is Isaac Sim?

Isaac Sim offers unique capabilities for robotics:

- **GPU-Accelerated Physics**: PhysX 5 for fast, accurate simulation
- **Photorealistic Rendering**: Real-time ray tracing with RTX
- **Synthetic Data Generation**: Domain randomization for training
- **ROS/ROS 2 Integration**: Native bridges for seamless communication
- **Isaac SDK Integration**: Motion planning, navigation, manipulation
- **Multi-Robot Simulation**: Coordinate multiple agents efficiently

## System Requirements

### Minimum Requirements
- **GPU**: NVIDIA RTX 2070 or higher
- **VRAM**: 8GB minimum, 12GB+ recommended
- **RAM**: 32GB system memory
- **OS**: Ubuntu 20.04/22.04 or Windows 10/11
- **Storage**: 50GB for installation

### Recommended
- **GPU**: NVIDIA RTX 3080 or A6000
- **VRAM**: 24GB+
- **RAM**: 64GB
- **Storage**: NVMe SSD

## Installation

### Step 1: Install Omniverse Launcher

Download from [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/):

#### Linux
```bash
# Download the AppImage
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run
./omniverse-launcher-linux.AppImage
```

#### Windows
Download and run the Windows installer from the website.

### Step 2: Install Isaac Sim

1. Open Omniverse Launcher
2. Go to **EXCHANGE** tab
3. Find **Isaac Sim**
4. Click **Install**
5. Choose installation path (requires ~40GB)
6. Wait for installation to complete

### Step 3: Install ROS 2 Bridge

Isaac Sim includes ROS 2 bridge by default. Verify installation:

```bash
# In Isaac Sim Python environment
cd ~/.local/share/ov/pkg/isaac_sim-*
./python.sh -m pip list | grep ros
```

## Launching Isaac Sim

### GUI Mode

```bash
# Navigate to Isaac Sim directory
cd ~/.local/share/ov/pkg/isaac_sim-*/

# Launch
./isaac-sim.sh
```

### Headless Mode (for servers)

```bash
./isaac-sim.sh --headless
```

### With ROS 2 Bridge

```bash
./isaac-sim.sh --ros2-bridge
```

## Creating a Scene

### Using the GUI

1. **Create Ground Plane**:
   - Create > Physics > Ground Plane
   - Adjust size in Properties

2. **Add Lighting**:
   - Create > Light > Dome Light
   - Load HDR environment (Content Browser)

3. **Import Robot**:
   - Isaac > Import Robot
   - Select URDF or USD file
   - Configure import settings

### Using Python

Create `humanoid_scene.py`:

```python
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

import omni
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import XFormPrim
import numpy as np

# Create world
world = World()
world.scene.add_default_ground_plane()

# Add lighting
omni.isaac.core.utils.stage.add_reference_to_stage(
    "/Environment/DomeLight",
    "omniverse://localhost/NVIDIA/Assets/Skies/Clear/noon_grass_4k.hdr"
)

# Create simple humanoid structure
def create_simple_humanoid():
    # Torso
    torso = world.scene.add(
        DynamicCuboid(
            prim_path="/World/Humanoid/Torso",
            name="torso",
            position=np.array([0, 0, 1.0]),
            scale=np.array([0.4, 0.3, 0.6]),
            color=np.array([0.5, 0.5, 0.5]),
            mass=30.0
        )
    )

    # Head
    head = world.scene.add(
        DynamicCuboid(
            prim_path="/World/Humanoid/Head",
            name="head",
            position=np.array([0, 0, 1.75]),
            scale=np.array([0.3, 0.3, 0.3]),
            color=np.array([0.9, 0.7, 0.6]),
            mass=5.0
        )
    )

    return torso, head

torso, head = create_simple_humanoid()

# Reset world
world.reset()

# Run simulation
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

Run with:
```bash
./python.sh humanoid_scene.py
```

## Importing Robots

### From URDF

```python
from omni.isaac.urdf import _urdf

# Import URDF
urdf_interface = _urdf.acquire_urdf_interface()

# Import settings
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False
import_config.self_collision = True

# Import
urdf_interface.parse_urdf(
    urdf_path="/path/to/humanoid.urdf",
    import_config=import_config,
    dest_path="/World/Humanoid"
)
```

### From USD (Universal Scene Description)

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# Add USD robot
add_reference_to_stage(
    usd_path="/path/to/humanoid.usd",
    prim_path="/World/Humanoid"
)
```

## ROS 2 Integration

### Enable ROS 2 Bridge

```python
from omni.isaac.core import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Enable ROS 2 bridge
import omni.isaac.ros2_bridge as ros2_bridge

# Initialize
ros2_bridge.enable_ros2_bridge()

# The bridge is now active and will publish/subscribe to ROS 2 topics
```

### Publishing Joint States

```python
from omni.isaac.core.articulations import Articulation

# Get robot articulation
robot = Articulation("/World/Humanoid")
robot.initialize()

# Enable joint state publisher
from omni.isaac.ros2_bridge import ROS2Publisher

joint_state_pub = ROS2Publisher(
    topic_name="/joint_states",
    message_type="sensor_msgs/JointState",
    queue_size=10
)

# In simulation loop:
while simulation_app.is_running():
    world.step(render=True)

    # Get joint positions
    joint_positions = robot.get_joint_positions()

    # Publish to ROS 2
    # (Isaac Sim handles this automatically when bridge is enabled)
```

### Subscribing to Commands

```python
from omni.isaac.ros2_bridge import ROS2Subscriber

def joint_command_callback(msg):
    """Handle incoming joint commands"""
    positions = msg.position
    robot.set_joint_position_targets(positions)

# Subscribe
joint_cmd_sub = ROS2Subscriber(
    topic_name="/joint_command",
    message_type="sensor_msgs/JointState",
    callback=joint_command_callback
)
```

## Camera and Sensors

### RGB Camera

```python
from omni.isaac.sensor import Camera
import numpy as np

# Create camera
camera = Camera(
    prim_path="/World/Humanoid/Head/Camera",
    position=np.array([0, 0, 1.8]),
    orientation=np.array([1, 0, 0, 0]),  # quaternion
    frequency=30,
    resolution=(640, 480)
)

camera.initialize()

# Enable ROS 2 publishing
camera.add_ros2_publisher(topic_name="/camera/image_raw")

# Get image data
rgb_data = camera.get_rgba()
```

### Depth Camera

```python
from omni.isaac.sensor import DepthCamera

depth_cam = DepthCamera(
    prim_path="/World/Humanoid/Head/DepthCamera",
    position=np.array([0, 0, 1.8]),
    frequency=20,
    resolution=(640, 480)
)

depth_cam.initialize()
depth_cam.add_ros2_publisher(topic_name="/camera/depth")

# Get depth data
depth_data = depth_cam.get_depth_data()
```

### LiDAR

```python
from omni.isaac.range_sensor import LidarRtx

lidar = LidarRtx(
    prim_path="/World/Humanoid/Torso/Lidar",
    position=np.array([0, 0, 1.5]),
    rotation_frequency=10,  # Hz
    horizontal_fov=360.0,
    horizontal_resolution=0.4,
    num_channels=16,
    max_range=100.0
)

lidar.initialize()
lidar.add_ros2_publisher(topic_name="/scan")
```

### IMU

```python
from omni.isaac.sensor import IMUSensor

imu = IMUSensor(
    prim_path="/World/Humanoid/Torso/IMU",
    position=np.array([0, 0, 1.0]),
    frequency=100
)

imu.initialize()
imu.add_ros2_publisher(topic_name="/imu/data")

# Get IMU readings
linear_acc = imu.get_linear_acceleration()
angular_vel = imu.get_angular_velocity()
```

## Physics Configuration

### Articulation Settings

```python
from omni.isaac.core.articulations import Articulation

robot = Articulation("/World/Humanoid")
robot.initialize()

# Set physics properties
robot.set_solver_position_iteration_count(64)
robot.set_solver_velocity_iteration_count(64)

# Set joint properties
for joint_name in robot.dof_names:
    robot.set_joint_stiffness(joint_name, 1000.0)
    robot.set_joint_damping(joint_name, 100.0)
    robot.set_joint_max_force(joint_name, 500.0)
```

### Contact Sensors

```python
from omni.isaac.sensor import ContactSensor

# Add contact sensor to foot
foot_sensor = ContactSensor(
    prim_path="/World/Humanoid/LeftFoot/ContactSensor",
    radius=0.05
)

foot_sensor.initialize()

# Check contact
is_in_contact = foot_sensor.is_in_contact()
contact_force = foot_sensor.get_contact_force()
```

## Synthetic Data Generation

### Domain Randomization

```python
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.synthetic_utils import DomainRandomization
import random

# Randomize lighting
def randomize_lighting():
    intensity = random.uniform(500, 2000)
    dome_light = omni.isaac.core.utils.prims.get_prim_at_path("/Environment/DomeLight")
    dome_light.GetAttribute("inputs:intensity").Set(intensity)

# Randomize textures
def randomize_materials():
    # Apply random materials to objects
    pass

# Randomize object poses
def randomize_scene():
    # Move objects to random positions
    objects = [...]
    for obj in objects:
        random_pos = np.random.uniform(-1, 1, 3)
        obj.set_world_pose(position=random_pos)

# Run in simulation loop
while simulation_app.is_running():
    if world.current_time_step_index % 100 == 0:
        randomize_lighting()
        randomize_scene()
    world.step(render=True)
```

### Capture Annotations

```python
from omni.isaac.synthetic_utils import SyntheticDataHelper

# Initialize synthetic data helper
sd_helper = SyntheticDataHelper()

# Enable annotations
sd_helper.initialize(
    sensor_names=["rgb", "semantic_segmentation", "instance_segmentation", "depth"],
    viewport_name="Viewport"
)

# Capture data
while simulation_app.is_running():
    world.step(render=True)

    # Get annotated data
    rgb = sd_helper.get_rgb()
    depth = sd_helper.get_depth()
    semantic = sd_helper.get_semantic_segmentation()
    instance = sd_helper.get_instance_segmentation()

    # Save or process data
```

## Motion Planning Integration

### Using Isaac Motion Planning

```python
from omni.isaac.motion_planning import RMPFlowController

# Create controller
controller = RMPFlowController(
    robot_prim_path="/World/Humanoid",
    urdf_path="/path/to/humanoid.urdf"
)

controller.initialize()

# Set target
target_position = np.array([1.0, 0.5, 1.2])
controller.set_end_effector_target(target_position)

# Execute in simulation loop
while simulation_app.is_running():
    world.step(render=True)

    # Compute actions
    actions = controller.forward()

    # Apply to robot
    robot.apply_action(actions)
```

## Performance Optimization

### Reduce Physics Overhead

```python
# Set physics scene settings
from omni.isaac.core.utils.physics import set_physics_scene_settings

set_physics_scene_settings(
    physics_dt=1.0/60.0,  # Physics timestep
    rendering_dt=1.0/60.0,  # Rendering timestep
    gpu_max_rigid_contact_count=512 * 1024,
    gpu_found_lost_pairs_capacity=1024 * 1024
)
```

### Use GPU Pipeline

```python
# Enable GPU dynamics
from omni.physx.scripts import physicsUtils

physicsUtils.set_physics_scene_broadphase_type("GPU")
```

### LOD (Level of Detail)

Use simplified collision meshes and reduce visual complexity for distant objects.

## Example: Complete Humanoid Control

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.sensor import Camera
import numpy as np

# Create world
world = World()
world.scene.add_default_ground_plane()

# Load humanoid (assuming URDF import done)
robot = world.scene.add(Articulation("/World/Humanoid"))

# Add camera
camera = Camera(
    prim_path="/World/Humanoid/Head/Camera",
    position=np.array([0, 0, 1.8]),
    frequency=30,
    resolution=(640, 480)
)

# Reset
world.reset()

# Control loop
while simulation_app.is_running():
    world.step(render=True)

    # Simple gait pattern
    time = world.current_time

    # Hip oscillation
    left_hip_angle = 0.3 * np.sin(2 * np.pi * time)
    right_hip_angle = -0.3 * np.sin(2 * np.pi * time)

    # Apply commands
    robot.set_joint_position_target("left_hip_joint", left_hip_angle)
    robot.set_joint_position_target("right_hip_joint", right_hip_angle)

    # Get camera image
    rgb_data = camera.get_rgba()

simulation_app.close()
```

## Troubleshooting

**Slow Performance:**
- Enable GPU physics
- Reduce physics iterations
- Simplify collision meshes
- Lower rendering quality

**Robot Unstable:**
- Increase solver iterations
- Adjust joint stiffness/damping
- Check mass distribution
- Verify contact parameters

**ROS 2 Bridge Not Working:**
```bash
# Verify ROS 2 environment
source /opt/ros/humble/setup.bash
echo $ROS_DOMAIN_ID

# Check if topics are visible
ros2 topic list
```

**GPU Memory Issues:**
- Reduce scene complexity
- Lower resolution textures
- Limit number of simultaneous sensors
- Use headless mode

## Next Steps

In the next lesson, you'll learn physics validation techniques to ensure your simulations accurately represent real-world behavior.

## Further Reading

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [Isaac SDK](https://developer.nvidia.com/isaac-sdk)
- [Omniverse Platform](https://www.nvidia.com/en-us/omniverse/)
- [USD Documentation](https://graphics.pixar.com/usd/docs/index.html)
