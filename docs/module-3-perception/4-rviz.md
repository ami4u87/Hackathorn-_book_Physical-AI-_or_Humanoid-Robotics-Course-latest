# RViz2 Visualization

RViz2 is the primary visualization tool for ROS 2, allowing you to see sensor data, robot state, and planning results in 3D.

## Launching RViz2

```bash
# Basic launch
ros2 run rviz2 rviz2

# With config file
ros2 run rviz2 rviz2 -d ~/my_config.rviz

# In launch file
from launch_ros.actions import Node

Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', '/path/to/config.rviz']
)
```

## Common Displays

### 1. TF (Transform Frames)

Shows all coordinate frames and their relationships.

**Setup**:
- Add > TF
- Adjust marker scale for visibility
- Enable/disable specific frames

### 2. Camera

Display camera feeds.

**Setup**:
- Add > Camera
- Topic: `/camera/image_raw`
- Set overlay alpha for transparency

### 3. PointCloud2

Visualize 3D point clouds.

**Setup**:
- Add > PointCloud2
- Topic: `/camera/points`
- Style: Points, Flat Squares, or Spheres
- Size: Adjust point size
- Color Transformer: RGB8, Intensity, or AxisColor

### 4. Image

Display 2D images with overlays.

**Setup**:
- Add > Image
- Topic: `/detection/image`
- Useful for showing detection results

### 5. Marker

Custom 3D shapes and text.

```python
from visualization_msgs.msg import Marker

def create_sphere_marker(position, color, marker_id=0):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0

    return marker

# Publish
marker_pub = node.create_publisher(Marker, '/visualization_marker', 10)
marker_pub.publish(create_sphere_marker([1.0, 0.5, 0.3], [1.0, 0.0, 0.0]))
```

### 6. MarkerArray

Multiple markers efficiently.

```python
from visualization_msgs.msg import MarkerArray

def create_detection_markers(detections_3d):
    marker_array = MarkerArray()

    for i, det in enumerate(detections_3d):
        # Sphere for object
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = node.get_clock().now().to_msg()
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = det['position'][0]
        marker.pose.position.y = det['position'][1]
        marker.pose.position.z = det['position'][2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = det['size'][0]
        marker.scale.y = det['size'][1]
        marker.scale.z = det['size'][2]

        marker.color.g = 1.0
        marker.color.a = 0.5

        marker_array.markers.append(marker)

        # Text label
        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.id = i + 1000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.pose.position.x = det['position'][0]
        text_marker.pose.position.y = det['position'][1]
        text_marker.pose.position.z = det['position'][2] + 0.2

        text_marker.scale.z = 0.1
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0

        text_marker.text = f"{det['class_name']} ({det['score']:.2f})"

        marker_array.markers.append(text_marker)

    return marker_array
```

## Robot Model Display

Show URDF/SDF robot model.

**Setup**:
- Add > RobotModel
- Description Topic: `/robot_description`
- TF Prefix: (leave empty or set if using namespaces)

## Path Display

Visualize planned trajectories.

```python
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def create_path(waypoints):
    """Create path from list of (x, y, z) waypoints"""
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = node.get_clock().now().to_msg()

    for wp in waypoints:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = wp[0]
        pose.pose.position.y = wp[1]
        pose.pose.position.z = wp[2]
        pose.pose.orientation.w = 1.0

        path.poses.append(pose)

    return path

# Example
waypoints = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (1, 1, 1)]
path_pub = node.create_publisher(Path, '/planned_path', 10)
path_pub.publish(create_path(waypoints))
```

## Interactive Markers

Allow user interaction in RViz2.

```python
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl

class InteractiveMarkerNode(Node):
    def __init__(self):
        super().__init__('interactive_marker_node')

        self.server = InteractiveMarkerServer(self, 'simple_marker')

        # Create marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "goal_marker"
        int_marker.description = "Drag to set goal"

        # Create control
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        int_marker.controls.append(control)

        # Add to server
        self.server.insert(int_marker, self.marker_feedback)
        self.server.applyChanges()

    def marker_feedback(self, feedback):
        self.get_logger().info(f"Marker at: {feedback.pose.position}")
```

## Saving and Loading Configurations

```bash
# Save current config
File > Save Config As > my_config.rviz

# Load config
File > Open Config > my_config.rviz

# Or via command line
ros2 run rviz2 rviz2 -d my_config.rviz
```

## Custom Plugins

Create custom RViz2 plugins for specialized visualization.

```cpp
// Custom panel example (C++)
#include <rviz_common/panel.hpp>

class MyCustomPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  MyCustomPanel(QWidget* parent = nullptr);
  virtual void onInitialize() override;

  // Add custom UI and functionality
};
```

## Performance Tips

1. **Reduce point cloud density**: Downsample before publishing
2. **Limit marker count**: Use MarkerArray efficiently
3. **Adjust update rates**: Slower rates for non-critical displays
4. **Use PointCloud2 instead of individual points**
5. **Disable unused displays**

## Further Reading

- [RViz2 User Guide](https://github.com/ros2/rviz)
- [Visualization Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker)
