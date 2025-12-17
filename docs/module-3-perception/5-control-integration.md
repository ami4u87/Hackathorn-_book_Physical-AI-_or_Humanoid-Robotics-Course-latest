# Control System Integration

Integrating perception with control systems allows robots to react to their environment. This lesson covers perception-control loops, visual servoing, and reactive behaviors.

## Perception-Control Architecture

```
Sensors → Perception → Decision Making → Motion Planning → Control → Actuators
   ↑                                                                      ↓
   └──────────────────── Feedback Loop ─────────────────────────────────┘
```

## Basic Reactive Controller

React to detected objects.

```python
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray

class ReactiveController(Node):
    def __init__(self):
        super().__init__('reactive_controller')

        # Subscribe to detections
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10
        )

        # Publish velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State
        self.target_detected = False
        self.target_center_x = 0

    def detection_callback(self, msg):
        """React to detected objects"""

        # Find target class (e.g., "person")
        target_class = "person"
        target_found = False

        for detection in msg.detections:
            if detection.id == target_class:
                target_found = True
                self.target_center_x = detection.bbox.center.position.x
                break

        self.target_detected = target_found

        # Generate control command
        self.generate_command()

    def generate_command(self):
        """Generate velocity command based on perception"""
        cmd = Twist()

        if self.target_detected:
            # Center target in image (assume 640px width)
            image_center = 320
            error = self.target_center_x - image_center

            # Proportional control
            Kp = 0.001  # Tuning parameter
            angular_velocity = -Kp * error

            cmd.linear.x = 0.2  # Move forward
            cmd.angular.z = angular_velocity
        else:
            # Search behavior
            cmd.angular.z = 0.3  # Rotate to find target

        self.cmd_vel_pub.publish(cmd)
```

## Visual Servoing

Control robot based on visual features.

```python
class VisualServo(Node):
    def __init__(self):
        super().__init__('visual_servo')

        # Subscribe to ArUco marker poses
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Joint controller
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_position_controller/commands', 10
        )

        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Target
        self.target_frame = 'aruco_marker_0'

    def control_loop(self):
        """Visual servoing control loop"""

        try:
            # Get marker pose in camera frame
            transform = self.tf_buffer.lookup_transform(
                'camera_link',
                self.target_frame,
                rclpy.time.Time()
            )

            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # Desired position
            x_desired = 0.0  # Centered in camera
            y_desired = 0.0
            z_desired = 0.5  # 0.5m away

            # Compute error
            error_x = x_desired - x
            error_y = y_desired - y
            error_z = z_desired - z

            # Proportional control
            Kp = 0.5
            vel_x = Kp * error_x
            vel_y = Kp * error_y
            vel_z = Kp * error_z

            # Convert to joint velocities (inverse kinematics)
            joint_velocities = self.velocity_ik(vel_x, vel_y, vel_z)

            # Publish
            cmd = Float64MultiArray()
            cmd.data = joint_velocities
            self.joint_cmd_pub.publish(cmd)

        except Exception as e:
            self.get_logger().warn(f"No transform: {e}")

    def velocity_ik(self, vx, vy, vz):
        """
        Simplified velocity inverse kinematics

        In practice, use actual Jacobian matrix
        """
        # Placeholder - implement actual IK
        return [vx, vy, vz, 0.0, 0.0, 0.0]
```

## State Machine for Complex Behaviors

```python
from enum import Enum

class State(Enum):
    SEARCH = 1
    APPROACH = 2
    GRASP = 3
    RETREAT = 4

class StateMachineController(Node):
    def __init__(self):
        super().__init__('state_machine_controller')

        self.state = State.SEARCH

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.state_machine_update)

        # State variables
        self.target_detected = False
        self.target_distance = 0.0

    def detection_callback(self, msg):
        # Update state variables based on detections
        self.target_detected = len(msg.detections) > 0
        if self.target_detected:
            # Estimate distance from bbox size (simplified)
            bbox_height = msg.detections[0].bbox.size_y
            self.target_distance = 1000.0 / bbox_height  # Rough approximation

    def state_machine_update(self):
        """Main state machine logic"""

        if self.state == State.SEARCH:
            self.search_behavior()

        elif self.state == State.APPROACH:
            self.approach_behavior()

        elif self.state == State.GRASP:
            self.grasp_behavior()

        elif self.state == State.RETREAT:
            self.retreat_behavior()

    def search_behavior(self):
        """Rotate to find target"""
        if self.target_detected:
            self.state = State.APPROACH
            self.get_logger().info("Target found, approaching")
        else:
            cmd = Twist()
            cmd.angular.z = 0.5
            self.cmd_vel_pub.publish(cmd)

    def approach_behavior(self):
        """Move toward target"""
        if not self.target_detected:
            self.state = State.SEARCH
            self.get_logger().info("Lost target, searching")
        elif self.target_distance < 0.3:
            self.state = State.GRASP
            self.get_logger().info("Reached target, grasping")
        else:
            cmd = Twist()
            cmd.linear.x = 0.2
            self.cmd_vel_pub.publish(cmd)

    def grasp_behavior(self):
        """Execute grasp"""
        # Trigger gripper close
        self.get_logger().info("Grasping")
        # ... gripper control logic ...

        # After grasp completes
        self.state = State.RETREAT

    def retreat_behavior(self):
        """Move back after grasp"""
        cmd = Twist()
        cmd.linear.x = -0.1
        self.cmd_vel_pub.publish(cmd)

        # After retreating
        # self.state = State.SEARCH
```

## Obstacle Avoidance

Use depth sensor for collision avoidance.

```python
class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Subscribe to depth or laser scan
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.min_distance = float('inf')
        self.obstacle_angle = 0.0

    def scan_callback(self, msg):
        """Process laser scan for obstacles"""

        # Find minimum distance
        ranges = np.array(msg.ranges)
        ranges[ranges == 0] = float('inf')  # Ignore invalid

        self.min_distance = np.min(ranges)
        min_idx = np.argmin(ranges)

        # Calculate angle to obstacle
        self.obstacle_angle = msg.angle_min + min_idx * msg.angle_increment

        # Generate avoidance command
        self.avoid_obstacle()

    def avoid_obstacle(self):
        """Generate velocity to avoid obstacle"""
        cmd = Twist()

        if self.min_distance < 0.5:
            # Too close, turn away
            cmd.linear.x = 0.0
            cmd.angular.z = -np.sign(self.obstacle_angle) * 1.0
        elif self.min_distance < 1.0:
            # Moderate distance, slow down and adjust
            cmd.linear.x = 0.1
            cmd.angular.z = -np.sign(self.obstacle_angle) * 0.5
        else:
            # Safe, proceed normally
            cmd.linear.x = 0.3

        self.cmd_vel_pub.publish(cmd)
```

## Performance Monitoring

```python
class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        self.detection_times = []
        self.control_times = []

        # Timers
        self.last_detection_time = self.get_clock().now()

    def log_detection_latency(self):
        """Measure detection pipeline latency"""
        now = self.get_clock().now()
        latency = (now - self.last_detection_time).nanoseconds / 1e6  # ms

        self.detection_times.append(latency)

        if len(self.detection_times) > 100:
            avg_latency = np.mean(self.detection_times)
            self.get_logger().info(f"Avg detection latency: {avg_latency:.1f}ms")
            self.detection_times = []

        self.last_detection_time = now
```

## Further Reading

- [ros2_control](https://control.ros.org/)
- [MoveIt 2](https://moveit.ros.org/)
- [Navigation2](https://navigation.ros.org/)
