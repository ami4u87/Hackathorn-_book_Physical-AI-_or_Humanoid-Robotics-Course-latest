# System Integration

The capstone project integrates all previous modules into a complete autonomous humanoid robot system.

## System Architecture

```
Voice Input → Speech-to-Text → VLA (GPT-4V) → Action Planning → Motion Planning → Robot Control
                                      ↑                                              ↓
                Camera Input ←  Perception (YOLO + Depth) ← State Estimation ← Feedback
```

## Integration Checklist

### Module 1: ROS 2 Foundation
- [ ] All nodes communicate via topics/services
- [ ] TF tree properly configured
- [ ] Launch files for each subsystem
- [ ] Parameters externalized

### Module 2: Simulation
- [ ] Robot model in Gazebo/Isaac Sim
- [ ] Sensors configured (camera, depth, IMU)
- [ ] Physics validated
- [ ] Environment loaded

### Module 3: Perception
- [ ] Object detection running
- [ ] Pose estimation publishing to TF
- [ ] RViz2 visualization working
- [ ] Sensor fusion implemented

### Module 4: VLA
- [ ] GPT-4V integration active
- [ ] Action generation working
- [ ] Validation pipeline in place
- [ ] Ambiguity handling implemented

### Module 5: Voice Interface
- [ ] Speech-to-text configured
- [ ] Text-to-speech for feedback
- [ ] Voice commands triggering pipeline

## Implementation Plan

### Phase 1: Individual Components (Weeks 1-2)

**Tasks**:
1. Verify each module works independently
2. Create integration tests for each
3. Document interfaces (topics, services, messages)

**Deliverables**:
- Module test reports
- Interface documentation
- Individual launch files

### Phase 2: Pairwise Integration (Week 3)

**Integrations**:
1. **Perception + VLA**: Camera feed → Object detection → Action generation
2. **VLA + Control**: Generated actions → Motion planning → Execution
3. **Voice + VLA**: Speech input → Command parsing → Action generation

**Validation**: Each pair works together without the full system.

### Phase 3: Full System Integration (Week 4)

**Pipeline**:
```
Voice Command → VLA → Validated Actions → Execution → Perception Feedback → Completion
```

**Tasks**:
1. Connect all modules
2. Add state machine for task flow
3. Implement error handling
4. Create unified launch file

### Phase 4: Testing & Refinement (Week 5)

**Tests**:
1. Simple tasks (pick and place)
2. Multi-step tasks (clean table)
3. Error recovery (grasp failure)
4. Edge cases (ambiguous commands)

## Integration Code

### Master Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        # Simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('simulation_pkg'),
                            'launch', 'gazebo.launch.py')
            ])
        ),

        # Perception
        Node(
            package='perception_pkg',
            executable='object_detector',
            name='object_detector'
        ),
        Node(
            package='perception_pkg',
            executable='pose_estimator',
            name='pose_estimator'
        ),

        # VLA
        Node(
            package='vla_pkg',
            executable='vla_node',
            name='vla_node'
        ),
        Node(
            package='vla_pkg',
            executable='action_validator',
            name='action_validator'
        ),

        # Voice Interface
        Node(
            package='voice_pkg',
            executable='speech_to_text',
            name='speech_to_text'
        ),

        # Control
        Node(
            package='control_pkg',
            executable='motion_planner',
            name='motion_planner'
        ),
        Node(
            package='control_pkg',
            executable='robot_controller',
            name='robot_controller'
        ),

        # State Machine
        Node(
            package='capstone_pkg',
            executable='task_manager',
            name='task_manager'
        ),

        # Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('capstone_pkg'),
                'config', 'capstone.rviz'
            )]
        ),
    ])
```

### Task Manager (State Machine)

```python
from enum import Enum
from rclpy.node import Node

class TaskState(Enum):
    IDLE = 0
    LISTENING = 1
    PERCEIVING = 2
    PLANNING = 3
    VALIDATING = 4
    EXECUTING = 5
    COMPLETED = 6
    FAILED = 7

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        self.state = TaskState.IDLE

        # Subscribers
        self.voice_sub = self.create_subscription(
            String, '/voice_command', self.voice_callback, 10
        )
        self.perception_sub = self.create_subscription(
            Detection2DArray, '/detections', self.perception_callback, 10
        )
        self.action_sub = self.create_subscription(
            String, '/validated_actions', self.action_callback, 10
        )
        self.execution_sub = self.create_subscription(
            String, '/execution_status', self.execution_callback, 10
        )

        # Publishers
        self.command_pub = self.create_publisher(String, '/task_command', 10)
        self.status_pub = self.create_publisher(String, '/task_status', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.update)

        # State variables
        self.current_command = None
        self.current_actions = None
        self.detections = None

    def update(self):
        """State machine update"""

        self.publish_status()

        if self.state == TaskState.IDLE:
            # Waiting for command
            pass

        elif self.state == TaskState.LISTENING:
            # Waiting for complete voice command
            if self.current_command:
                self.state = TaskState.PERCEIVING

        elif self.state == TaskState.PERCEIVING:
            # Waiting for perception
            if self.detections:
                self.state = TaskState.PLANNING

        elif self.state == TaskState.PLANNING:
            # Trigger VLA
            self.trigger_action_generation()
            self.state = TaskState.VALIDATING

        elif self.state == TaskState.VALIDATING:
            # Waiting for validated actions
            if self.current_actions:
                self.state = TaskState.EXECUTING

        elif self.state == TaskState.EXECUTING:
            # Actions being executed
            pass

        elif self.state == TaskState.COMPLETED:
            self.get_logger().info("Task completed successfully!")
            self.reset()

        elif self.state == TaskState.FAILED:
            self.get_logger().error("Task failed")
            self.reset()

    def voice_callback(self, msg):
        if self.state == TaskState.IDLE:
            self.current_command = msg.data
            self.state = TaskState.LISTENING
            self.get_logger().info(f"Received command: {msg.data}")

    def perception_callback(self, msg):
        self.detections = msg

    def action_callback(self, msg):
        if self.state == TaskState.VALIDATING:
            self.current_actions = json.loads(msg.data)
            self.get_logger().info(f"Received {len(self.current_actions)} actions")

    def execution_callback(self, msg):
        if msg.data == "completed":
            self.state = TaskState.COMPLETED
        elif msg.data == "failed":
            self.state = TaskState.FAILED

    def trigger_action_generation(self):
        """Send command to VLA"""
        cmd_msg = String()
        cmd_msg.data = self.current_command
        self.command_pub.publish(cmd_msg)

    def publish_status(self):
        """Publish current state"""
        status = String()
        status.data = self.state.name
        self.status_pub.publish(status)

    def reset(self):
        """Reset for next task"""
        self.state = TaskState.IDLE
        self.current_command = None
        self.current_actions = None
        self.detections = None
```

## Communication Diagram

```
┌─────────────┐
│   Voice     │ → /voice_command
└─────────────┘
       ↓
┌─────────────┐
│ Task Manager│ → /task_command
└─────────────┘
       ↓
┌─────────────┐
│     VLA     │ → /robot_actions
└─────────────┘
       ↓
┌─────────────┐
│  Validator  │ → /validated_actions
└─────────────┘
       ↓
┌─────────────┐
│   Planner   │ → /planned_trajectory
└─────────────┘
       ↓
┌─────────────┐
│ Controller  │ → /joint_commands
└─────────────┘
       ↓
    Robot
```

## Debugging Tips

1. **Launch modules individually first**
2. **Use ROS 2 tools**:
   ```bash
   ros2 topic list
   ros2 topic echo /topic_name
   ros2 node list
   ros2 service list
   ```
3. **Check TF tree**: `ros2 run tf2_tools view_frames`
4. **Monitor in RViz2**: Visualize everything
5. **Add logging**: Verbose logs in each module
6. **Record bags**: `ros2 bag record -a`

## Next Steps

Complete the voice interface setup and execute your first end-to-end task!
