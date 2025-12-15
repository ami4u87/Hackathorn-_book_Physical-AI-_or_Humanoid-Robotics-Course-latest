# ROS 2 Test Examples for CI/CD

Complete examples of unit tests, integration tests, and Gazebo simulation tests for ROS 2 Humble.

## Table of Contents
1. [Unit Tests (pytest)](#unit-tests)
2. [Integration Tests (launch_testing)](#integration-tests)
3. [Gazebo Simulation Tests](#gazebo-simulation-tests)
4. [Test Configuration](#test-configuration)

---

## Unit Tests

### Example 1: Python Unit Test (pytest)

**File**: `test/test_math_utils.py`

```python
#!/usr/bin/env python3
"""Unit tests for math utilities - no ROS runtime required."""

import pytest
import math


class MathUtils:
    """Example utility class."""

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Convert quaternion to Euler angles."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


class TestMathUtils:
    """Unit tests for MathUtils."""

    def test_quaternion_identity(self):
        """Test identity quaternion (no rotation)."""
        roll, pitch, yaw = MathUtils.quaternion_to_euler(0.0, 0.0, 0.0, 1.0)
        assert abs(roll) < 1e-6
        assert abs(pitch) < 1e-6
        assert abs(yaw) < 1e-6

    def test_quaternion_90deg_roll(self):
        """Test 90 degree roll rotation."""
        # Quaternion for 90 deg roll: [sin(45°), 0, 0, cos(45°)]
        roll, pitch, yaw = MathUtils.quaternion_to_euler(
            0.7071068, 0.0, 0.0, 0.7071068
        )
        assert abs(roll - math.pi / 2) < 1e-6
        assert abs(pitch) < 1e-6
        assert abs(yaw) < 1e-6

    @pytest.mark.parametrize(
        "x,y,z,w,expected_roll,expected_pitch,expected_yaw",
        [
            (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),  # Identity
            (0.7071068, 0.0, 0.0, 0.7071068, math.pi / 2, 0.0, 0.0),  # 90° roll
            (0.0, 0.7071068, 0.0, 0.7071068, 0.0, math.pi / 2, 0.0),  # 90° pitch
            (0.0, 0.0, 0.7071068, 0.7071068, 0.0, 0.0, math.pi / 2),  # 90° yaw
        ],
    )
    def test_quaternion_conversions(
        self, x, y, z, w, expected_roll, expected_pitch, expected_yaw
    ):
        """Parameterized test for multiple quaternion conversions."""
        roll, pitch, yaw = MathUtils.quaternion_to_euler(x, y, z, w)
        assert abs(roll - expected_roll) < 1e-5
        assert abs(pitch - expected_pitch) < 1e-5
        assert abs(yaw - expected_yaw) < 1e-5
```

**CMakeLists.txt** (add to package):
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_math_utils test/test_math_utils.py)
endif()
```

---

## Integration Tests

### Example 2: Publisher/Subscriber Test

**File**: `test/test_talker_listener.py`

```python
#!/usr/bin/env python3
"""Integration test for talker/listener nodes."""

import unittest
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch talker and listener nodes for testing."""
    talker_node = launch_ros.actions.Node(
        package='demo_nodes_cpp',
        executable='talker',
        output='screen',
    )

    return (
        launch.LaunchDescription([
            talker_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'talker_node': talker_node,
        }
    )


class TestTalkerListener(unittest.TestCase):
    """Test pub/sub communication."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_talker_publishes(self, proc_output):
        """Test that talker node publishes messages."""
        messages_received = []

        def listener_callback(msg):
            messages_received.append(msg.data)

        subscription = self.node.create_subscription(
            String,
            'chatter',
            listener_callback,
            10
        )

        # Spin for up to 5 seconds
        timeout = 5.0
        start_time = self.node.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            elapsed = (self.node.get_clock().now() - start_time).nanoseconds / 1e9

            if len(messages_received) >= 3:
                break

            if elapsed > timeout:
                break

        # Verify we received messages
        self.assertGreaterEqual(
            len(messages_received), 3,
            f"Expected at least 3 messages, got {len(messages_received)}"
        )

        # Verify message content format
        for msg in messages_received:
            self.assertIn("Hello World", msg)

        subscription.destroy()
```

**CMakeLists.txt**:
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_talker_listener.py
    TIMEOUT 30
  )
endif()
```

---

### Example 3: Service Client Test

**File**: `test/test_add_two_ints.py`

```python
#!/usr/bin/env python3
"""Integration test for add_two_ints service."""

import unittest
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

import launch
import launch_ros.actions
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    """Launch service server for testing."""
    service_server = launch_ros.actions.Node(
        package='demo_nodes_cpp',
        executable='add_two_ints_server',
        output='screen',
    )

    return (
        launch.LaunchDescription([
            service_server,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'service_server': service_server,
        }
    )


class TestAddTwoIntsService(unittest.TestCase):
    """Test service communication."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_service_client')

    def tearDown(self):
        self.node.destroy_node()

    def test_service_available(self):
        """Test that service is available."""
        client = self.node.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service with timeout
        service_available = client.wait_for_service(timeout_sec=5.0)
        self.assertTrue(
            service_available,
            "Service 'add_two_ints' not available after 5 seconds"
        )

        client.destroy()

    def test_service_call(self):
        """Test service call returns correct result."""
        client = self.node.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service
        self.assertTrue(
            client.wait_for_service(timeout_sec=5.0),
            "Service not available"
        )

        # Create request
        request = AddTwoInts.Request()
        request.a = 10
        request.b = 20

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        # Verify response
        self.assertTrue(future.done(), "Service call did not complete")
        response = future.result()
        self.assertEqual(response.sum, 30, f"Expected 30, got {response.sum}")

        client.destroy()

    def test_service_multiple_calls(self):
        """Test multiple service calls."""
        client = self.node.create_client(AddTwoInts, 'add_two_ints')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))

        test_cases = [
            (5, 7, 12),
            (0, 0, 0),
            (-5, 10, 5),
            (100, 200, 300),
        ]

        for a, b, expected in test_cases:
            request = AddTwoInts.Request()
            request.a = a
            request.b = b

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

            response = future.result()
            self.assertEqual(
                response.sum, expected,
                f"Expected {a} + {b} = {expected}, got {response.sum}"
            )

        client.destroy()
```

---

## Gazebo Simulation Tests

### Example 4: Gazebo Spawn Test

**File**: `test/test_gazebo_spawn.py`

```python
#!/usr/bin/env python3
"""Test robot spawning in Gazebo."""

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity, GetEntityState

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import pytest
import time


@pytest.mark.launch_test
def generate_test_description():
    """Launch Gazebo and spawn robot."""
    # Start Gazebo server (headless)
    gazebo = launch.actions.ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '4', 'empty.sdf'],
        output='screen',
    )

    # ROS-Gazebo bridge
    bridge = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return (
        launch.LaunchDescription([
            gazebo,
            bridge,
            launch.actions.TimerAction(
                period=5.0,  # Wait for Gazebo to start
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {
            'gazebo': gazebo,
            'bridge': bridge,
        }
    )


class TestGazeboSpawn(unittest.TestCase):
    """Test robot spawning in Gazebo."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_gazebo_spawn')

    def tearDown(self):
        self.node.destroy_node()

    def test_gazebo_running(self):
        """Test that Gazebo simulation is running."""
        # Check for /clock topic (published by Gazebo)
        topic_list = self.node.get_topic_names_and_types()
        clock_topics = [name for name, _ in topic_list if name == '/clock']

        self.assertGreater(
            len(clock_topics), 0,
            "Gazebo /clock topic not found - simulation may not be running"
        )

    def test_spawn_robot(self):
        """Test spawning a robot in Gazebo."""
        # Create spawn service client
        spawn_client = self.node.create_client(SpawnEntity, '/spawn_entity')

        # Wait for service
        self.assertTrue(
            spawn_client.wait_for_service(timeout_sec=10.0),
            "Spawn service not available"
        )

        # Simple robot SDF
        robot_sdf = '''<?xml version="1.0"?>
        <sdf version="1.6">
          <model name="test_robot">
            <static>false</static>
            <link name="base_link">
              <collision name="collision">
                <geometry>
                  <box><size>1 1 1</size></box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box><size>1 1 1</size></box>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>'''

        # Create spawn request
        request = SpawnEntity.Request()
        request.name = 'test_robot'
        request.xml = robot_sdf
        request.robot_namespace = ''
        request.initial_pose = Pose()
        request.initial_pose.position.z = 1.0  # Spawn 1m above ground

        # Call spawn service
        future = spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        # Verify spawn success
        self.assertTrue(future.done(), "Spawn service call did not complete")
        response = future.result()
        self.assertTrue(response.success, f"Spawn failed: {response.status_message}")

        spawn_client.destroy()

    def test_robot_state(self):
        """Test querying robot state from Gazebo."""
        # First spawn robot (code omitted for brevity)
        # ... (use test_spawn_robot code)

        # Wait for robot to settle
        time.sleep(2.0)

        # Get entity state
        state_client = self.node.create_client(GetEntityState, '/get_entity_state')
        self.assertTrue(state_client.wait_for_service(timeout_sec=10.0))

        request = GetEntityState.Request()
        request.name = 'test_robot'

        future = state_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        response = future.result()
        self.assertTrue(response.success, "Failed to get robot state")

        # Verify robot position (should have fallen to z=0.5 due to gravity)
        self.assertLess(
            response.state.pose.position.z, 1.0,
            "Robot did not fall (physics not working?)"
        )

        state_client.destroy()
```

**CMakeLists.txt**:
```cmake
if(BUILD_TESTING)
  add_launch_test(
    test/test_gazebo_spawn.py
    TIMEOUT 60  # Longer timeout for Gazebo
  )
endif()
```

---

### Example 5: Gazebo Robot Control Test

**File**: `launch/test_robot_control.launch.py`

```python
#!/usr/bin/env python3
"""Launch file for testing robot control in Gazebo."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import launch_testing.actions


def generate_test_description():
    """Generate launch description for robot control test."""

    # Start Gazebo with robot world
    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-s', '-r', '-v', '4',
            'robot_world.sdf'
        ],
        output='screen',
    )

    # ROS-Gazebo bridge for control
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen',
    )

    # Robot controller node
    controller = Node(
        package='robot_control',
        executable='velocity_controller',
        output='screen',
    )

    # Test node (publishes velocity commands)
    test_node = Node(
        package='robot_control',
        executable='test_velocity_commands',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        TimerAction(period=5.0, actions=[bridge]),
        TimerAction(period=7.0, actions=[controller]),
        TimerAction(period=10.0, actions=[test_node]),
        TimerAction(
            period=15.0,
            actions=[launch_testing.actions.ReadyToTest()],
        ),
    ])


# Test implementation
import unittest


class TestRobotControl(unittest.TestCase):
    """Test robot control in Gazebo."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_robot_moves_forward(self):
        """Test robot responds to forward velocity command."""
        node = rclpy.create_node('test_robot_control')

        # Subscribe to odometry
        odom_data = []

        def odom_callback(msg):
            odom_data.append(msg)

        odom_sub = node.create_subscription(
            'nav_msgs/msg/Odometry',
            '/odom',
            odom_callback,
            10
        )

        # Publish velocity command
        vel_pub = node.create_publisher(
            'geometry_msgs/msg/Twist',
            '/cmd_vel',
            10
        )

        # Send forward command
        from geometry_msgs.msg import Twist
        vel_msg = Twist()
        vel_msg.linear.x = 0.5  # 0.5 m/s forward

        # Publish for 3 seconds
        for _ in range(30):
            vel_pub.publish(vel_msg)
            rclpy.spin_once(node, timeout_sec=0.1)

        # Stop robot
        vel_msg.linear.x = 0.0
        vel_pub.publish(vel_msg)

        # Verify robot moved
        self.assertGreater(len(odom_data), 10, "No odometry data received")

        # Check final position
        final_odom = odom_data[-1]
        initial_x = odom_data[0].pose.pose.position.x
        final_x = final_odom.pose.pose.position.x

        distance_traveled = final_x - initial_x
        self.assertGreater(
            distance_traveled, 0.5,
            f"Robot moved only {distance_traveled}m, expected > 0.5m"
        )

        node.destroy_node()
```

---

## Test Configuration

### package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>robot_control</name>
  <version>0.1.0</version>
  <description>Robot control with Gazebo</description>

  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>gazebo_msgs</depend>
  <depend>ros_gz_bridge</depend>

  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>launch_testing</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>launch_testing_ros</test_depend>
  <test_depend>python3-pytest</test_depend>
  <test_depend>ros_testing</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt (complete)

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# Add executables
add_executable(velocity_controller src/velocity_controller.cpp)
ament_target_dependencies(velocity_controller
  rclcpp geometry_msgs nav_msgs
)

# Install
install(TARGETS velocity_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  # Linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests (C++)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_math_utils test/test_math_utils.cpp)
  target_link_libraries(test_math_utils ${PROJECT_NAME}_lib)

  # Unit tests (Python)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_utils_py test/test_utils.py)

  # Integration tests
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_talker_listener.py
    TIMEOUT 30
  )

  add_launch_test(
    test/test_add_two_ints.py
    TIMEOUT 30
  )

  # Gazebo simulation tests (longer timeout)
  add_launch_test(
    test/test_gazebo_spawn.py
    TIMEOUT 60
  )

  add_launch_test(
    launch/test_robot_control.launch.py
    TIMEOUT 90
  )
endif()

ament_package()
```

### pytest.ini (project root)

```ini
[pytest]
# Pytest configuration for ROS 2 tests
testpaths = test
python_files = test_*.py
python_classes = Test*
python_functions = test_*

# Timeout for individual tests
timeout = 30

# Show extra test summary
addopts =
    -v
    --tb=short
    --strict-markers
    --disable-warnings

# Markers
markers =
    unit: Unit tests (no ROS runtime)
    integration: Integration tests (ROS nodes)
    simulation: Gazebo simulation tests
    slow: Tests that take > 10 seconds
```

### Running Tests

```bash
# Build with tests
colcon build --cmake-args -DBUILD_TESTING=ON

# Run all tests
colcon test

# Run specific package tests
colcon test --packages-select robot_control

# Run with verbose output
colcon test --event-handlers console_direct+

# Run specific test file
colcon test --pytest-args -k test_talker_listener

# Run only unit tests (fast)
colcon test --pytest-args -m unit

# Run with coverage
colcon test --pytest-args --cov --cov-report=html

# Show test results
colcon test-result --all --verbose
```

---

## CI/CD Integration

Add to `.github/workflows/ros2-ci.yml`:

```yaml
- name: Run unit tests (fast)
  run: |
    colcon test \
      --pytest-args -m unit \
      --event-handlers console_direct+

- name: Run integration tests
  run: |
    colcon test \
      --pytest-args -m integration \
      --event-handlers console_direct+

- name: Run simulation tests (headless)
  run: |
    Xvfb :99 &
    export DISPLAY=:99
    colcon test \
      --pytest-args -m simulation \
      --event-handlers console_direct+
```

This provides a complete testing framework for ROS 2 CI/CD pipelines!
