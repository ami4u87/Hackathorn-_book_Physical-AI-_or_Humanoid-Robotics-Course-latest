# Vision and Depth Perception

Vision is the primary sensing modality for humanoid robots to understand their environment. This lesson covers RGB cameras, depth sensors, and techniques for extracting meaningful information from visual data.

## Types of Vision Sensors

### 1. RGB Cameras

**Characteristics**:
- Captures color information (Red, Green, Blue channels)
- High resolution (640×480 to 4K+)
- Low cost
- No direct depth information

**Use Cases**:
- Object recognition
- Color-based tracking
- Visual servoing
- Human detection

**ROS 2 Integration**:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process image
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Display
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Depth Cameras

Depth cameras provide distance measurements for each pixel.

**Technologies**:

| Type | Technology | Range | Pros | Cons |
|------|-----------|-------|------|------|
| **Stereo** | Triangulation | 1-20m | Works outdoors | Needs texture |
| **Structured Light** | IR pattern | 0.5-5m | High accuracy | Indoor only |
| **Time-of-Flight** | Light travel time | 0.1-10m | Fast, accurate | Limited range |
| **LiDAR** | Laser scanning | 1-100m+ | Long range | Expensive |

**Popular Sensors**:
- **Intel RealSense D435i**: Stereo + IMU, $200-400
- **Azure Kinect**: ToF + RGB, $400
- **ZED 2**: Stereo with AI, $450
- **Ouster OS1**: LiDAR, $3500+

### 3. RGB-D (RGB + Depth)

Combines color and depth in aligned frames.

**ROS 2 Depth Processing**:

```python
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')

        # Subscribe to RGB and Depth
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )

        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        # Depth is typically in millimeters (uint16) or meters (float32)
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        if self.depth_image is not None:
            self.process_depth()

    def process_depth(self):
        # Convert to meters if in millimeters
        if self.depth_image.dtype == np.uint16:
            depth_m = self.depth_image.astype(np.float32) / 1000.0
        else:
            depth_m = self.depth_image

        # Filter invalid depths
        depth_m[depth_m == 0] = np.nan
        depth_m[depth_m > 10.0] = np.nan

        # Visualize depth as colormap
        depth_normalized = cv2.normalize(depth_m, None, 0, 255, cv2.NORM_MINMAX)
        depth_normalized = np.nan_to_num(depth_normalized).astype(np.uint8)
        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        cv2.imshow("Depth", depth_colored)
        cv2.waitKey(1)

    def get_distance_at_pixel(self, x, y):
        """Get distance to object at pixel (x, y)"""
        if self.depth_image is None:
            return None

        depth_mm = self.depth_image[y, x]
        depth_m = depth_mm / 1000.0

        return depth_m
```

## Camera Calibration

Camera calibration determines intrinsic parameters (focal length, principal point, distortion) needed for accurate measurements.

### Intrinsic Parameters

```python
# Camera intrinsic matrix K
# [fx  0  cx]
# [ 0 fy  cy]
# [ 0  0   1]

# fx, fy: focal length in pixels
# cx, cy: principal point (image center)

class CameraIntrinsics:
    def __init__(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

        self.K = np.array([
            [fx, 0,  cx],
            [0,  fy, cy],
            [0,  0,  1]
        ])

    def pixel_to_ray(self, u, v):
        """Convert pixel coordinates to 3D ray direction"""
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        z = 1.0

        ray = np.array([x, y, z])
        ray = ray / np.linalg.norm(ray)  # Normalize

        return ray

    def project_point(self, point_3d):
        """Project 3D point to pixel coordinates"""
        x, y, z = point_3d

        u = self.fx * (x / z) + self.cx
        v = self.fy * (y / z) + self.cy

        return int(u), int(v)

# Example: RealSense D435i intrinsics (typical)
camera = CameraIntrinsics(
    fx=615.0,  # pixels
    fy=615.0,
    cx=320.0,  # half of 640
    cy=240.0   # half of 480
)
```

### Calibration with ROS 2

```bash
# Install calibration tools
sudo apt install ros-humble-camera-calibration

# Run calibration (print a checkerboard pattern)
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.024 \
    --ros-args -r image:=/camera/image_raw

# After calibration, save to:
# ~/.ros/camera_info/camera_name.yaml
```

## Point Cloud Generation

Convert depth images to 3D point clouds.

```python
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import struct

class PointCloudGenerator(Node):
    def __init__(self):
        super().__init__('point_cloud_generator')

        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )

        self.pc_pub = self.create_publisher(
            PointCloud2, '/camera/points', 10
        )

        # Camera intrinsics (load from camera_info in practice)
        self.fx = 615.0
        self.fy = 615.0
        self.cx = 320.0
        self.cy = 240.0

        self.bridge = CvBridge()

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        depth_m = depth_image.astype(np.float32) / 1000.0

        # Generate point cloud
        points = self.depth_to_points(depth_m)

        # Publish
        pc_msg = self.create_point_cloud_msg(points, msg.header)
        self.pc_pub.publish(pc_msg)

    def depth_to_points(self, depth_image):
        """Convert depth image to 3D points"""
        height, width = depth_image.shape
        points = []

        for v in range(0, height, 4):  # Subsample for efficiency
            for u in range(0, width, 4):
                z = depth_image[v, u]

                if z == 0 or z > 10.0:  # Skip invalid
                    continue

                # Backproject to 3D
                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy

                points.append([x, y, z])

        return np.array(points)

    def create_point_cloud_msg(self, points, header):
        """Create PointCloud2 message"""
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = "camera_depth_optical_frame"

        msg.height = 1
        msg.width = len(points)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats × 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        # Pack data
        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', point[0], point[1], point[2]))

        msg.data = b''.join(buffer)

        return msg
```

Visualize in RViz2:
```bash
ros2 run rviz2 rviz2
# Add > PointCloud2 > Topic: /camera/points
```

## Filtering and Processing

### 1. Noise Filtering

```python
import scipy.ndimage as ndimage

def filter_depth_image(depth_image):
    """Remove noise from depth image"""

    # 1. Remove outliers (isolated pixels)
    depth_filtered = ndimage.median_filter(depth_image, size=5)

    # 2. Fill small holes
    mask = depth_filtered > 0
    depth_filtered = ndimage.morphology.binary_closing(mask, iterations=2) * depth_filtered

    # 3. Bilateral filter (edge-preserving)
    depth_filtered = cv2.bilateralFilter(
        depth_filtered.astype(np.float32),
        d=9,
        sigmaColor=75,
        sigmaSpace=75
    )

    return depth_filtered
```

### 2. Downsampling

```python
def downsample_point_cloud(points, voxel_size=0.01):
    """
    Voxel grid downsampling

    Args:
        points: Nx3 array
        voxel_size: Size of voxel in meters
    """
    # Quantize points to voxels
    voxel_indices = np.floor(points / voxel_size).astype(int)

    # Remove duplicates (keep one point per voxel)
    _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)

    downsampled = points[unique_indices]

    return downsampled
```

### 3. Plane Segmentation

```python
def segment_ground_plane(points, distance_threshold=0.01, iterations=100):
    """
    RANSAC plane fitting

    Args:
        points: Nx3 point cloud
        distance_threshold: Max distance to plane
        iterations: RANSAC iterations

    Returns:
        inliers: Boolean mask of ground points
        plane_model: [a, b, c, d] where ax + by + cz + d = 0
    """
    best_inliers = None
    best_plane = None
    best_count = 0

    for _ in range(iterations):
        # Sample 3 random points
        sample_indices = np.random.choice(len(points), 3, replace=False)
        p1, p2, p3 = points[sample_indices]

        # Compute plane normal
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        normal = normal / np.linalg.norm(normal)

        # Plane equation: ax + by + cz + d = 0
        a, b, c = normal
        d = -np.dot(normal, p1)

        # Compute distances
        distances = np.abs(points @ normal + d)

        # Count inliers
        inliers = distances < distance_threshold
        count = np.sum(inliers)

        if count > best_count:
            best_count = count
            best_inliers = inliers
            best_plane = np.array([a, b, c, d])

    return best_inliers, best_plane
```

## Stereo Vision

For cameras without built-in depth sensors, use stereo vision.

```python
class StereoMatcher:
    def __init__(self, baseline_m, focal_length_px):
        """
        Args:
            baseline_m: Distance between cameras in meters
            focal_length_px: Focal length in pixels
        """
        self.baseline = baseline_m
        self.focal_length = focal_length_px

        # Create stereo matcher (SGBM = Semi-Global Block Matching)
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16*10,  # Must be divisible by 16
            blockSize=5,
            P1=8 * 3 * 5**2,
            P2=32 * 3 * 5**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )

    def compute_disparity(self, left_image, right_image):
        """Compute disparity map"""

        # Convert to grayscale
        left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        # Compute disparity
        disparity = self.matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0

        return disparity

    def disparity_to_depth(self, disparity):
        """Convert disparity to depth using: Z = (f × B) / d"""

        # Avoid division by zero
        disparity[disparity <= 0] = 0.1

        depth = (self.focal_length * self.baseline) / disparity

        return depth
```

## Visualization Tools

### RViz2 Configuration

```python
# Publish markers for visualization
from visualization_msgs.msg import Marker

def publish_detection_marker(publisher, position, color):
    """Publish a sphere marker at detected object position"""

    marker = Marker()
    marker.header.frame_id = "camera_link"
    marker.header.stamp = self.get_clock().now().to_msg()

    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0

    marker.lifetime.sec = 1

    publisher.publish(marker)
```

## Performance Optimization

### 1. Region of Interest (ROI)

```python
def process_roi(image, roi_rect):
    """Process only region of interest"""
    x, y, w, h = roi_rect
    roi = image[y:y+h, x:x+w]

    # Process only ROI
    processed_roi = expensive_operation(roi)

    # Insert back
    result = image.copy()
    result[y:y+h, x:x+w] = processed_roi

    return result
```

### 2. Multi-threading

```python
import threading

class AsyncImageProcessor:
    def __init__(self):
        self.latest_image = None
        self.lock = threading.Lock()
        self.processing = False

    def image_callback(self, msg):
        with self.lock:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg)

        # Process in separate thread
        if not self.processing:
            thread = threading.Thread(target=self.process_async)
            thread.start()

    def process_async(self):
        self.processing = True

        with self.lock:
            image = self.latest_image.copy()

        # Expensive processing
        result = self.heavy_computation(image)

        self.processing = False
```

## Next Steps

In the next lesson, you'll learn object detection and classification using these vision inputs.

## Further Reading

- [OpenCV Documentation](https://docs.opencv.org/)
- [ROS 2 Image Pipeline](https://github.com/ros-perception/image_pipeline)
- [Point Cloud Library](https://pointclouds.org/)
- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense)
