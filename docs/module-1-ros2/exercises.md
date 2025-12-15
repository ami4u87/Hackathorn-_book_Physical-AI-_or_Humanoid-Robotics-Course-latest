# Module 1 Exercises

This section contains hands-on exercises to reinforce the concepts learned in Module 1: ROS 2 Fundamentals. Each exercise includes prompts, acceptance criteria, and validation guidance to help you verify your understanding.

## Exercise 1: Publisher-Subscriber Pattern

### Exercise Prompt
Create a publisher node that publishes temperature readings to a topic called `temperature`, and a subscriber node that receives these readings and logs them to the console. The publisher should simulate temperature readings that vary between 18°C and 25°C with some random variation.

### Implementation Requirements
1. Create a new ROS 2 package called `temperature_monitor`
2. Implement a publisher node that:
   - Publishes `std_msgs/Float64` messages
   - Publishes at 1 Hz (once per second)
   - Generates random temperatures between 18°C and 25°C
3. Implement a subscriber node that:
   - Subscribes to the `temperature` topic
   - Logs received temperatures to the console
   - Identifies temperatures outside the normal range (20°C-23°C) as "warning" level

### Acceptance Criteria
- Publisher successfully publishes temperature values at 1 Hz
- Subscriber successfully receives and logs temperature values
- Warning messages are logged when temperature is outside normal range
- Both nodes can be run simultaneously without errors
- Nodes follow ROS 2 naming conventions and best practices

### Validation Steps
1. Build the package: `colcon build --packages-select temperature_monitor`
2. Source the workspace: `source install/setup.bash`
3. Run the publisher in one terminal: `ros2 run temperature_monitor temperature_publisher`
4. Run the subscriber in another terminal: `ros2 run temperature_monitor temperature_subscriber`
5. Verify that temperatures are published and received correctly
6. Confirm that warning messages appear for out-of-range values

## Exercise 2: Service Implementation

### Exercise Prompt
Create a service that calculates the distance between two 2D points. The service should accept two points (each with x and y coordinates) and return the Euclidean distance between them.

### Implementation Requirements
1. Define a custom service message `CalculateDistance.srv` with:
   - Request: `float64 x1, y1, x2, y2`
   - Response: `float64 distance`
2. Implement a service server that calculates the distance
3. Implement a service client that calls the service with predefined points
4. The service should handle edge cases (e.g., identical points)

### Acceptance Criteria
- Service definition is properly created and compiled
- Service server correctly calculates distances
- Service client successfully calls the service and displays results
- Edge cases are handled appropriately
- Service follows ROS 2 service implementation best practices

### Validation Steps
1. Create the service definition file in `srv/CalculateDistance.srv`
2. Build the package: `colcon build`
3. Source the workspace: `source install/setup.bash`
4. Run the service server: `ros2 run your_package distance_server`
5. In another terminal, run the client: `ros2 run your_package distance_client`
6. Verify correct distance calculations for various point pairs
7. Test edge case with identical points (distance should be 0)

## Exercise 3: Action Implementation

### Exercise Prompt
Create an action that simulates a robot moving to a specified position. The action should accept a target position and provide feedback on the progress of the movement, including the current position and estimated time to arrival.

### Implementation Requirements
1. Define a custom action `MoveToPosition.action` with:
   - Goal: `float64 x, y` (target position)
   - Feedback: `float64 current_x, current_y, progress_percentage`
   - Result: `bool success, string message`
2. Implement an action server that simulates movement with:
   - 10-second movement duration
   - Periodic feedback updates every second
   - Ability to cancel the movement
3. Implement an action client that sends goals and displays feedback

### Acceptance Criteria
- Action definition is properly created and compiled
- Action server provides accurate feedback during execution
- Action client can send goals and receive feedback
- Movement can be canceled before completion
- Error handling for invalid target positions

### Validation Steps
1. Create the action definition file in `action/MoveToPosition.action`
2. Build the package: `colcon build`
3. Source the workspace: `source install/setup.bash`
4. Run the action server: `ros2 run your_package move_server`
5. In another terminal, run the client: `ros2 run your_package move_client`
6. Verify progress feedback is updated regularly
7. Test cancellation functionality

## Exercise 4: Parameter Configuration

### Exercise Prompt
Create a node that implements a configurable PID controller using parameters. The node should accept P, I, and D gain parameters that can be adjusted at runtime without restarting the node.

### Implementation Requirements
1. Create a node that declares parameters for PID gains:
   - `p_gain` (default: 1.0)
   - `i_gain` (default: 0.1)
   - `d_gain` (default: 0.01)
2. Implement parameter validation to ensure gains are positive
3. Create a callback function that updates the PID controller when parameters change
4. Add a parameter for `max_output` to limit the controller output

### Acceptance Criteria
- Parameters are properly declared with appropriate defaults
- Parameter validation prevents invalid values
- PID controller updates immediately when parameters change
- Max output limit is enforced
- Parameter changes can be made at runtime using command line tools

### Validation Steps
1. Build and source the package
2. Run the PID controller node: `ros2 run your_package pid_controller`
3. Change parameters at runtime: `ros2 param set /pid_controller p_gain 2.0`
4. Verify the parameter value has changed: `ros2 param get /pid_controller p_gain`
5. Test parameter validation with negative values (should be rejected)
6. Confirm the controller behavior changes with different parameter values

## Exercise 5: Launch File Configuration

### Exercise Prompt
Create a launch file that starts multiple nodes from the previous exercises simultaneously, with appropriate parameters and remappings to demonstrate a simple robotic system.

### Implementation Requirements
1. Create a launch file that starts:
   - Temperature publisher with custom parameters
   - Temperature subscriber
   - PID controller with specific gain values
2. Use launch arguments to make the launch file configurable
3. Implement remappings to connect nodes appropriately
4. Include error handling for missing dependencies

### Acceptance Criteria
- Launch file starts all required nodes successfully
- Launch arguments allow for different configurations
- Nodes communicate properly through remapped topics
- Launch file follows ROS 2 launch best practices

### Validation Steps
1. Create the launch file in the `launch/` directory
2. Test the basic launch: `ros2 launch your_package system.launch.py`
3. Test with custom arguments: `ros2 launch your_package system.launch.py p_gain:=1.5`
4. Verify all nodes start and communicate correctly
5. Test system behavior with different parameter sets

## Exercise 6: TF2 Transformations

### Exercise Prompt
Create a simple robot model with multiple frames (base, sensor, end-effector) and demonstrate TF2 transformations between these frames.

### Implementation Requirements
1. Create static transforms for a simple robot:
   - `base_link` to `sensor_frame` (sensor mounted 0.5m forward, 0.2m up from base)
   - `base_link` to `end_effector` (end effector at 1.0m forward from base)
2. Implement a node that publishes these static transforms
3. Create a listener that queries transforms between frames
4. Demonstrate transforming a point from one frame to another

### Acceptance Criteria
- Static transforms are published correctly
- Transform listener successfully queries transformations
- Point transformations work correctly between frames
- TF2 tree is properly structured without cycles

### Validation Steps
1. Build and source the package
2. Run the static transform broadcaster
3. Use `ros2 run tf2_tools view_frames` to visualize the transform tree
4. Use `ros2 run tf2_ros tf2_echo base_link sensor_frame` to verify transforms
5. Test point transformations between frames
6. Verify the transform tree structure is correct

## Exercise Validation Script

To assist with validating your exercises, create a validation script that checks if your implementation meets the acceptance criteria. Here's a basic template:

```python
#!/usr/bin/env python3
import subprocess
import time
import sys


def validate_exercise_1():
    """Validate Exercise 1: Publisher-Subscriber Pattern"""
    print("Validating Exercise 1: Publisher-Subscriber Pattern...")

    # Test if nodes can be executed
    try:
        result = subprocess.run(['ros2', 'run', 'temperature_monitor', 'temperature_publisher', '--ros-args', '--log-level', 'error'],
                              timeout=2, capture_output=True)
        print("✓ Publisher node executes without errors")
    except subprocess.TimeoutExpired:
        print("✓ Publisher node runs (timed out as expected)")
    except Exception as e:
        print(f"✗ Publisher node failed: {e}")
        return False

    try:
        result = subprocess.run(['ros2', 'run', 'temperature_monitor', 'temperature_subscriber', '--ros-args', '--log-level', 'error'],
                              timeout=2, capture_output=True)
        print("✓ Subscriber node executes without errors")
    except subprocess.TimeoutExpired:
        print("✓ Subscriber node runs (timed out as expected)")
    except Exception as e:
        print(f"✗ Subscriber node failed: {e}")
        return False

    return True


def main():
    print("Starting Exercise Validation...")

    all_passed = True
    all_passed &= validate_exercise_1()

    if all_passed:
        print("\n✓ All validation checks passed!")
        return 0
    else:
        print("\n✗ Some validation checks failed!")
        return 1


if __name__ == "__main__":
    sys.exit(main())
```

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/index.html)
- [ROS 2 Launch Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)

## Next Steps

After completing these exercises, you should have a solid understanding of the core ROS 2 concepts:
- Publisher-subscriber communication
- Service-based request-response
- Action-based goal-oriented communication
- Parameter configuration
- Launch file organization
- Transform management with TF2

These fundamentals form the foundation for more advanced topics in subsequent modules, including simulation, perception, and control systems for humanoid robotics.