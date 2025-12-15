# Actions

Actions in ROS 2 provide a communication pattern for long-running tasks that require feedback, goal management, and cancellation capabilities. Unlike services, which are synchronous and blocking, actions allow for asynchronous execution with continuous feedback. This makes them ideal for tasks like robot navigation, manipulation, or any operation that takes a significant amount of time to complete.

## Understanding Actions

Actions combine the concepts of services and topics to provide:
- **Goal**: The task to be performed
- **Feedback**: Continuous updates on task progress
- **Result**: The final outcome of the task
- **Cancel**: Ability to cancel a running task

Actions are perfect for operations like:
- Moving a robot to a specific location
- Manipulating an object with a robotic arm
- Performing a complex calibration procedure
- Running an extended data collection process

## Creating an Action Definition

Let's create a custom action definition. Create a file called `Fibonacci.action` in a new package:

```bash
# In your workspace directory
mkdir -p src/my_robot_msgs/action
```

Create the action definition in `src/my_robot_msgs/action/Fibonacci.action`:

```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

This action takes an order (number of Fibonacci numbers to generate) and returns a sequence of Fibonacci numbers. The feedback is the partial sequence as it's being generated.

## Creating an Action Server

Let's create an action server that implements our Fibonacci action.

### Python Action Server Example

Create a new package for actions:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_action
```

Create the action server at `src/py_action/py_action/action_server.py`:

```python
from py_action.action import Fibonacci

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Initialize the sequence with the first two Fibonacci numbers
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Publish initial feedback
        goal_handle.publish_feedback(feedback_msg)

        # Generate the Fibonacci sequence up to the requested order
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            self.get_logger().info(f'Publishing feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info(f'Returning result: {result.sequence}')

        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```

## Creating an Action Client

Now let's create an action client that sends goals to our action server.

### Python Action Client Example

Create the client at `src/py_action/py_action/action_client.py`:

```python
from py_action.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

## Running the Action Example

1. First, build your workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select py_action
```

2. Source the workspace:

```bash
source install/setup.bash
```

3. Run the action server in one terminal:

```bash
ros2 run py_action action_server
```

4. In another terminal, run the client:

```bash
ros2 run py_action action_client
```

You should see the client send a goal, receive feedback during execution, and finally get the result.

## Action States

Actions can be in one of several states:
- **PENDING**: The goal has been accepted but not yet started
- **ACTIVE**: The goal is currently being processed
- **RECALLING**: The goal is being recalled before execution
- **REJECTED**: The goal was rejected by the server
- **RECALLED**: The goal was successfully recalled
- **PREEMPTING**: The goal is being preempted by a new goal
- **PREEMPTED**: The goal was successfully preempted
- **SUCCEEDED**: The goal completed successfully
- **ABORTED**: The goal was aborted due to failure

## Using Built-in Actions

ROS 2 provides various built-in actions through packages like `nav2_msgs` for navigation. You can also use command-line tools to interact with actions:

```bash
# List all available actions
ros2 action list

# Get information about a specific action
ros2 action info <action_name>

# Send a goal to an action
ros2 action send_goal <action_name> <action_type> <goal_values>
```

## Advanced Action Concepts

### Goal Handling and Preemption

Action servers can handle multiple goals and implement preemption:

```python
def execute_callback(self, goal_handle):
    self.get_logger().info('Executing goal...')

    # Check for preemption
    while some_condition:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Fibonacci.Result()

        # Check if a new goal should preempt this one
        if goal_handle.is_new_goal_available:
            goal_handle.abort()
            return Fibonacci.Result()

        # Do work...

    goal_handle.succeed()
    return Fibonacci.Result()
```

### Goal Validation

Servers can validate goals before accepting them:

```python
def execute_callback(self, goal_handle):
    # Validate the goal
    if goal_handle.request.order <= 0:
        self.get_logger().info('Invalid order, rejecting goal')
        goal_handle.abort()
        return Fibonacci.Result()

    # Accept and process the goal
    # ... rest of the implementation
```

## Best Practices

1. **Use appropriate communication pattern**: Use actions for long-running tasks that need feedback or cancellation, services for simple request-response, and topics for continuous data streams.

2. **Provide meaningful feedback**: Feedback should give useful information about progress to the client.

3. **Handle cancellation gracefully**: Always check for cancellation requests and clean up resources appropriately.

4. **Set appropriate timeouts**: Configure timeouts for action clients to prevent indefinite waiting.

5. **Design clear action interfaces**: Goals, feedback, and results should be well-defined and intuitive.

## Summary

Actions provide a sophisticated communication pattern for long-running tasks with feedback and cancellation capabilities. They are essential for operations like robot navigation, manipulation, and complex procedures that take time to complete. Understanding when to use actions versus services or topics is crucial for effective ROS 2 system design.

In the next section, we'll explore parameters, which provide a way to configure nodes dynamically at runtime.