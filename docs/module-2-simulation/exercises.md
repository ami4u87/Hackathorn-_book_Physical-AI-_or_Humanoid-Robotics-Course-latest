# Module 2 Exercises

These exercises will help you practice the simulation concepts covered in this module. Complete them in order, as they build on each other.

## Exercise 1: Basic Gazebo Scene Setup

**Objective**: Create a custom simulation environment in Gazebo Harmonic.

**Tasks**:

1. Create a new Gazebo world file with:
   - A ground plane
   - Walls forming a 5m × 5m room
   - At least 3 objects of different shapes (box, sphere, cylinder)
   - Appropriate lighting

2. Launch your world using a ROS 2 launch file

3. Use the ROS 2-Gazebo bridge to:
   - Subscribe to clock messages
   - Publish the world state to a ROS topic

**Deliverables**:
- `custom_world.sdf` - Your world file
- `launch_world.launch.py` - Launch file
- Screenshot of your running simulation

**Validation**:
```bash
# Your simulation should support these commands:
ros2 launch your_package launch_world.launch.py
ros2 topic list  # Should show /clock and other bridged topics
ros2 topic echo /clock
```

---

## Exercise 2: Robot Model Creation

**Objective**: Build a simplified humanoid robot model.

**Tasks**:

1. Create a URDF or SDF file for a humanoid robot with:
   - Torso (main body)
   - Head with camera sensor
   - 2 arms (shoulder and elbow joints each)
   - 2 legs (hip, knee, ankle joints each)
   - Appropriate mass and inertia values

2. Add sensors:
   - RGB camera in the head
   - IMU in the torso
   - Contact sensors in both feet

3. Spawn your robot in Gazebo at position (0, 0, 1)

**Deliverables**:
- `humanoid.urdf` or `humanoid.sdf`
- `spawn_robot.launch.py`
- Documentation of mass distribution

**Validation**:
```bash
# Check robot spawns correctly
ros2 launch your_package spawn_robot.launch.py

# Verify sensors are publishing
ros2 topic list | grep camera
ros2 topic list | grep imu
```

**Bonus**: Add joint controllers and make the robot wave one arm.

---

## Exercise 3: Joint Control and Movement

**Objective**: Implement basic joint control for your robot.

**Tasks**:

1. Add joint controller plugins to your robot model

2. Create a Python node that:
   - Publishes joint position commands
   - Implements a simple gait pattern (alternating leg swing)
   - Logs joint states

3. Visualize joint trajectories in RViz2

**Deliverables**:
- `joint_controller.py` - ROS 2 node
- Configuration files for controllers
- Video/GIF of robot moving

**Validation**:
```python
# Your node should publish to these topics:
/joint_commands  # Your commands
/joint_states    # Feedback from Gazebo

# Test with:
ros2 run your_package joint_controller.py
```

**Expected Behavior**: Robot should perform a walking motion without falling.

---

## Exercise 4: Unity Scene Setup

**Objective**: Set up a Unity environment and connect to ROS 2.

**Tasks**:

1. Create a Unity project with:
   - ROS-TCP-Connector installed
   - A floor and obstacles
   - Lighting and materials

2. Create a simple robot in Unity (or import your URDF)

3. Set up ROS 2 communication:
   - Configure ROS-TCP endpoint
   - Create a publisher for robot position
   - Create a subscriber for velocity commands

4. Test bidirectional communication

**Deliverables**:
- Unity project folder
- `ROSPublisher.cs` and `ROSSubscriber.cs` scripts
- Screenshot showing ROS Settings configured

**Validation**:
```bash
# Start ROS-TCP endpoint
ros2 run ros_tcp_endpoint default_server_endpoint

# In another terminal, check topics
ros2 topic list | grep unity
ros2 topic echo /unity/position

# Publish command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
```

**Expected Behavior**: Robot in Unity should move when you publish to `/cmd_vel`.

---

## Exercise 5: Camera Integration

**Objective**: Stream camera images from simulation to ROS 2.

**Tasks**:

1. In Gazebo or Unity, configure a robot camera

2. Bridge camera topics to ROS 2:
   - `/camera/image_raw` (sensor_msgs/Image)
   - `/camera/camera_info` (sensor_msgs/CameraInfo)

3. Create a Python node that:
   - Subscribes to camera images
   - Detects a colored object (use OpenCV)
   - Publishes bounding box overlay

4. Visualize in RViz2 or using `rqt_image_view`

**Deliverables**:
- Camera configuration in SDF/Unity
- `object_detector.py` - Detection node
- Screenshot showing detection working

**Validation**:
```bash
# View camera feed
ros2 run rqt_image_view rqt_image_view

# Run detector
ros2 run your_package object_detector.py

# Check detection output
ros2 topic echo /detected_objects
```

---

## Exercise 6: Isaac Sim Scene (Advanced)

**Objective**: Create a scene in NVIDIA Isaac Sim with ROS 2 integration.

**Tasks**:

1. Install Isaac Sim and launch it

2. Create a Python script that:
   - Initializes a World
   - Adds ground plane and lighting
   - Spawns a robot (from URDF or built-in asset)
   - Adds a camera and depth sensor

3. Enable ROS 2 bridge and publish:
   - Joint states
   - Camera images
   - Depth maps

4. Implement basic teleoperation

**Deliverables**:
- `isaac_scene.py` - Scene creation script
- `teleop_controller.py` - Teleoperation node
- Video of robot being controlled

**Validation**:
```bash
# Run Isaac Sim scene
~/.local/share/ov/pkg/isaac_sim-*/python.sh isaac_scene.py

# In another terminal
ros2 topic list  # Should show robot topics
ros2 run your_package teleop_controller.py
```

---

## Exercise 7: Physics Validation

**Objective**: Validate simulation physics against theoretical predictions.

**Tasks**:

1. **Drop Test**:
   - Drop a sphere from height h = 1.0m
   - Measure impact time: t_expected = √(2h/g) ≈ 0.452s
   - Compare simulation to theory (should be within 5%)

2. **Pendulum Test**:
   - Create a simple pendulum (length L = 1.0m)
   - Release from 30° angle
   - Measure period: T_expected = 2π√(L/g) ≈ 2.01s
   - Validate simulation matches theory

3. **Friction Test**:
   - Place a box on an inclined plane
   - Gradually increase angle until it slips
   - Calculate friction coefficient: μ = tan(θ)
   - Verify matches your configured friction value

**Deliverables**:
- World files for each test
- Python script to analyze results
- Report comparing theory vs simulation

**Validation Template**:
```python
def validate_drop_test(measured_time, height=1.0):
    g = 9.81
    expected_time = (2 * height / g) ** 0.5
    error_percent = abs(measured_time - expected_time) / expected_time * 100

    print(f"Expected: {expected_time:.3f}s")
    print(f"Measured: {measured_time:.3f}s")
    print(f"Error: {error_percent:.2f}%")

    return error_percent < 5.0  # Pass if within 5%
```

---

## Exercise 8: Multi-Robot Simulation

**Objective**: Simulate multiple robots in the same environment.

**Tasks**:

1. Create a Gazebo world with:
   - 3 identical mobile robots
   - Obstacles and walls
   - Individual namespaces for each robot

2. Implement a simple coordination algorithm:
   - Each robot patrols a different area
   - Robots avoid collisions with each other
   - Use TF to track all robot poses

3. Visualize all robots in RViz2

**Deliverables**:
- Multi-robot launch file
- `robot_controller.py` with namespace support
- Video showing coordination

**Validation**:
```bash
# Launch all robots
ros2 launch your_package multi_robot.launch.py

# Check namespaced topics
ros2 topic list | grep robot_1
ros2 topic list | grep robot_2
ros2 topic list | grep robot_3

# Visualize TF tree
ros2 run tf2_tools view_frames
```

---

## Exercise 9: Sensor Fusion

**Objective**: Combine data from multiple sensors.

**Tasks**:

1. Configure a robot with:
   - RGB camera
   - Depth camera
   - IMU
   - Lidar (optional)

2. Create a sensor fusion node that:
   - Synchronizes camera and depth data (using `message_filters`)
   - Combines IMU orientation with camera pose
   - Publishes fused pose estimate

3. Compare fused estimate to ground truth from simulation

**Deliverables**:
- `sensor_fusion.py` - Fusion node
- Launch file with all sensors configured
- Plot comparing fused vs ground truth poses

**Code Starter**:
```python
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, Imu

class SensorFusion:
    def __init__(self):
        # Subscribers with time synchronization
        image_sub = Subscriber('/camera/image', Image)
        imu_sub = Subscriber('/imu/data', Imu)

        ts = ApproximateTimeSynchronizer(
            [image_sub, imu_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        ts.registerCallback(self.callback)

    def callback(self, image_msg, imu_msg):
        # Fuse data here
        pass
```

---

## Exercise 10: Sim-to-Real Preparation (Capstone)

**Objective**: Prepare a simulation for potential real-world deployment.

**Tasks**:

1. Choose a simple task (e.g., "pick up a cube")

2. Implement in simulation with:
   - Realistic physics parameters (validated)
   - Sensor noise models
   - Actuator limits
   - Domain randomization

3. Create deployment documentation:
   - Hardware requirements
   - Calibration procedures
   - Safety considerations
   - Performance metrics

4. Test with varied conditions:
   - Different lighting (if using vision)
   - Different object positions
   - Different floor friction values

**Deliverables**:
- Complete simulation package
- `deployment_guide.md` documentation
- Video compilation of successful task completion under varied conditions
- Performance analysis report

**Success Criteria**:
- Task succeeds in >90% of randomized trials
- Physics validation tests pass
- All sensors publish at expected rates
- Documentation is complete

---

## Bonus Challenges

### Challenge A: Dynamic Obstacles
Add moving obstacles to your environment that robots must avoid in real-time.

### Challenge B: Complex Manipulation
Simulate a humanoid robot grasping and placing objects on shelves at different heights.

### Challenge C: Terrain Adaptation
Create environments with stairs, ramps, and uneven terrain. Make your robot navigate them.

### Challenge D: Multi-Modal Sensing
Implement visual-inertial odometry (VIO) combining camera and IMU data.

### Challenge E: Cloud Simulation
Deploy your simulation to a cloud instance (AWS, Azure) and control it remotely.

---

## Submission Guidelines

For each exercise, submit:

1. **Code**: Well-commented source code with README
2. **Documentation**: Explain your approach and design decisions
3. **Results**: Screenshots, videos, or plots demonstrating success
4. **Reflection**: What challenges did you face? How did you solve them?

### Code Quality Checklist
- [ ] Code follows ROS 2 style guidelines
- [ ] All nodes have launch files
- [ ] Parameters are configurable (not hardcoded)
- [ ] Error handling implemented
- [ ] README with installation and usage instructions

### Testing Checklist
- [ ] All nodes start without errors
- [ ] Topics publish at expected rates
- [ ] Simulation runs in real-time (or faster)
- [ ] No warnings in console output

---

## Additional Resources

- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Isaac Sim Tutorials](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorials.html)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)

Good luck with your exercises! Remember: simulation is a tool for learning and validation. The goal is to build intuition about robot behavior before deploying to real hardware.
