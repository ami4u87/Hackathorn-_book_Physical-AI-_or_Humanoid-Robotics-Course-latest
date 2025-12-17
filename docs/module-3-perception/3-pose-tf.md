# Pose Estimation and TF Publishing

Pose estimation determines the 3D position and orientation of objects. Publishing these poses to the TF (Transform) tree allows the robot to reason about spatial relationships and plan actions.

## Understanding TF2

TF2 is ROS 2's coordinate frame transformation system.

### Key Concepts

- **Frame**: A coordinate system (e.g., `world`, `camera_link`, `base_link`)
- **Transform**: Translation + rotation between two frames
- **TF Tree**: Hierarchical graph of frame relationships
- **Static vs Dynamic**: Static transforms don't change; dynamic transforms update over time

### TF Tree Example

```
world
├── map
│   └── odom
│       └── base_link
│           ├── camera_link
│           │   └── camera_optical_frame
│           ├── left_gripper
│           └── right_gripper
└── detected_object_1
```

## Publishing Static Transforms

For fixed relationships (e.g., camera mounted on robot).

```python
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')

        self.broadcaster = StaticTransformBroadcaster(self)

        # Publish camera transform
        self.publish_camera_transform()

    def publish_camera_transform(self):
        """
        Publish static transform from base_link to camera_link

        Camera is 0.2m forward, 0.0m sideways, 0.5m up,
        rotated 30 degrees down
        """
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        # Translation
        t.transform.translation.x = 0.2
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.5

        # Rotation (quaternion for 30-degree pitch)
        pitch = math.radians(-30)
        t.transform.rotation.x = math.sin(pitch / 2)
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(pitch / 2)

        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = StaticTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Using Launch Files

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.2', '0', '0.5', '0', '-0.5', '0', 'base_link', 'camera_link']
            # Arguments: x y z yaw pitch roll parent_frame child_frame
        )
    ])
```

## Publishing Dynamic Transforms

For moving objects or changing relationships.

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_publisher')

        self.broadcaster = TransformBroadcaster(self)

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.publish_transforms)

    def publish_transforms(self):
        """Publish dynamic transforms"""

        # Example: Object detected at varying position
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = 'detected_object'

        # Position from perception system
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.5
        t.transform.translation.z = 0.3

        # Orientation (identity quaternion = no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t)
```

## Rotation Representations

### Euler Angles to Quaternion

```python
def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion

    Args:
        roll, pitch, yaw: Rotation angles in radians

    Returns:
        (x, y, z, w): Quaternion components
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)

# Example usage
roll, pitch, yaw = 0.0, np.radians(30), np.radians(45)
quat = euler_to_quaternion(roll, pitch, yaw)
```

### Quaternion to Euler Angles

```python
def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to Euler angles

    Returns:
        (roll, pitch, yaw): Rotation angles in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)
```

### Rotation Matrix to Quaternion

```python
def rotation_matrix_to_quaternion(R):
    """
    Convert 3x3 rotation matrix to quaternion

    Args:
        R: 3x3 rotation matrix

    Returns:
        (x, y, z, w): Quaternion
    """
    trace = np.trace(R)

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return (x, y, z, w)
```

## Object Pose Estimation

### ArUco Marker Detection

ArUco markers provide known geometry for robust pose estimation.

```python
import cv2
import cv2.aruco as aruco

class ArUcoPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimator')

        # ArUco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()

        # Camera intrinsics (should load from camera_info in practice)
        self.camera_matrix = np.array([
            [615.0, 0.0, 320.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros((5, 1))

        # Marker size in meters
        self.marker_size = 0.05  # 5cm

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None:
            # Estimate pose for each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            for i in range(len(ids)):
                marker_id = ids[i][0]
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # Publish TF
                self.publish_marker_tf(marker_id, rvec, tvec, msg.header.stamp)

                # Draw axis for visualization
                cv2.drawFrameAxes(
                    cv_image, self.camera_matrix, self.dist_coeffs,
                    rvec, tvec, self.marker_size * 0.5
                )

            # Draw detected markers
            aruco.drawDetectedMarkers(cv_image, corners, ids)

        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)

    def publish_marker_tf(self, marker_id, rvec, tvec, timestamp):
        """Publish marker pose as TF transform"""

        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'camera_link'
        t.child_frame_id = f'aruco_marker_{marker_id}'

        # Translation
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        # Convert rotation vector to quaternion
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat = rotation_matrix_to_quaternion(rotation_matrix)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)
```

### PnP (Perspective-n-Point) for Generic Objects

For objects with known 3D models.

```python
def estimate_pose_pnp(image_points, model_points, camera_matrix, dist_coeffs):
    """
    Estimate object pose using PnP

    Args:
        image_points: Nx2 array of 2D points in image
        model_points: Nx3 array of corresponding 3D points in object frame
        camera_matrix: 3x3 camera intrinsic matrix
        dist_coeffs: Distortion coefficients

    Returns:
        rvec, tvec: Rotation and translation vectors
    """
    success, rvec, tvec = cv2.solvePnP(
        model_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE
    )

    if not success:
        return None, None

    # Refine using Levenberg-Marquardt
    rvec, tvec = cv2.solvePnPRefineLM(
        model_points, image_points, camera_matrix, dist_coeffs, rvec, tvec
    )

    return rvec, tvec

# Example: Cube with known corners
model_points = np.array([
    [0, 0, 0],      # Corner 1
    [0.05, 0, 0],   # Corner 2
    [0.05, 0.05, 0], # Corner 3
    [0, 0.05, 0],   # Corner 4
], dtype=np.float32)

# Detected corners in image
image_points = np.array([
    [100, 200],
    [150, 210],
    [140, 250],
    [95, 245]
], dtype=np.float32)

rvec, tvec = estimate_pose_pnp(image_points, model_points, camera_matrix, dist_coeffs)
```

## Listening to Transforms

Lookup transforms to convert between frames.

```python
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_transform(self, target_frame, source_frame):
        """
        Get transform from source_frame to target_frame

        Returns:
            TransformStamped or None if not available
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return transform
        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return None

    def transform_pose(self, pose_stamped, target_frame):
        """
        Transform pose to target frame

        Args:
            pose_stamped: PoseStamped message
            target_frame: Target frame name

        Returns:
            Transformed PoseStamped or None
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose_stamped.header.frame_id,
                pose_stamped.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            transformed_pose = do_transform_pose(pose_stamped, transform)
            return transformed_pose

        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")
            return None

# Example usage
def example_transform_usage(node):
    # Create pose in camera frame
    pose = PoseStamped()
    pose.header.frame_id = 'camera_link'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = 1.0
    pose.pose.position.y = 0.5
    pose.pose.position.z = 0.3
    pose.pose.orientation.w = 1.0

    # Transform to base_link
    transformed = node.transform_pose(pose, 'base_link')

    if transformed:
        print(f"In base_link: x={transformed.pose.position.x:.2f}, "
              f"y={transformed.pose.position.y:.2f}, "
              f"z={transformed.pose.position.z:.2f}")
```

## Complete Perception Pipeline

Integrate detection and pose estimation:

```python
class PerceptionPipeline(Node):
    def __init__(self):
        super().__init__('perception_pipeline')

        # Object detector
        self.detector = YOLODetector('yolov5s.onnx')

        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera intrinsics
        self.camera_matrix = None
        self.depth_image = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10
        )

        self.bridge = CvBridge()

    def camera_info_callback(self, msg):
        """Extract camera intrinsics"""
        K = np.array(msg.k).reshape(3, 3)
        self.camera_matrix = K

    def depth_callback(self, msg):
        """Store latest depth image"""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg):
        """Main perception loop"""
        if self.camera_matrix is None or self.depth_image is None:
            return

        # Detect objects
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detections = self.detector.detect(cv_image)

        # Estimate 3D poses
        for i, det in enumerate(detections):
            pose_3d = self.estimate_3d_pose(det, self.depth_image, self.camera_matrix)

            if pose_3d:
                # Publish TF
                self.publish_object_tf(
                    f"{det['class_name']}_{i}",
                    pose_3d,
                    msg.header.stamp
                )

    def estimate_3d_pose(self, detection, depth_image, camera_matrix):
        """Estimate 3D pose from 2D detection and depth"""
        x, y, w, h = detection['box']

        cx = int(x + w / 2)
        cy = int(y + h / 2)

        # Get depth
        roi = depth_image[int(y):int(y+h), int(x):int(x+w)]
        roi_valid = roi[roi > 0]

        if len(roi_valid) == 0:
            return None

        z = np.median(roi_valid) / 1000.0  # Convert mm to m

        # Back-project
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx_cam = camera_matrix[0, 2]
        cy_cam = camera_matrix[1, 2]

        x_3d = (cx - cx_cam) * z / fx
        y_3d = (cy - cy_cam) * z / fy

        return (x_3d, y_3d, z)

    def publish_object_tf(self, object_id, position, timestamp):
        """Publish object pose to TF tree"""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'camera_depth_optical_frame'
        t.child_frame_id = f'object_{object_id}'

        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        # Identity orientation (can be improved with full 6D pose estimation)
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
```

## Visualization in RViz2

```bash
# Launch RViz2
ros2 run rviz2 rviz2

# Add displays:
# - TF: Shows all coordinate frames
# - Camera: Shows camera feed
# - Marker: Shows custom markers for objects
```

## Next Steps

In the next lesson, you'll learn to visualize perception results in RViz2 and integrate with control systems.

## Further Reading

- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [OpenCV Camera Calibration](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)
- [ArUco Documentation](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)
