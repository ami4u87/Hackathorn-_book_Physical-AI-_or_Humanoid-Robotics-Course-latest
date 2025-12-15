# Publisher-Subscriber Pattern

The Publisher-Subscriber (Pub/Sub) pattern is one of the fundamental communication mechanisms in ROS 2. It enables asynchronous message passing between different nodes in a robotic system. Publishers send messages to topics, and subscribers receive messages from topics without needing to know about each other directly.

## Understanding Topics

In ROS 2, a topic is a named bus over which nodes exchange messages. Topics have a type associated with them, which defines the structure of the messages that can be published to that topic. Nodes can publish messages to a topic or subscribe to a topic to receive messages.

## Creating a Publisher Node

Let's create a simple publisher node that publishes messages to a topic called `example_topic`.

### Python Publisher Example

First, let's create a new ROS 2 package for our examples:

```bash
# In your workspace directory (~/ros2_ws)
mkdir -p src/py_pubsub
cd src/py_pubsub
ros2 pkg create --build-type ament_python py_pubsub
```

Now create the publisher script at `src/py_pubsub/py_pubsub/publisher_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'example_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### C++ Publisher Example

For C++ users, create a similar publisher at `src/cpp_pubsub/src/talker.cpp`:

```cpp
#include <chrono>
#include <cinttypes>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("example_topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

## Creating a Subscriber Node

Now let's create a subscriber node that listens to the same topic.

### Python Subscriber Example

Create the subscriber at `src/py_pubsub/py_pubsub/subscriber_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'example_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Running the Publisher and Subscriber

1. First, build your workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
```

2. Source the workspace:

```bash
source install/setup.bash
```

3. Run the publisher in one terminal:

```bash
ros2 run py_pubsub publisher_member_function
```

4. In another terminal, run the subscriber:

```bash
ros2 run py_pubsub subscriber_member_function
```

You should see the publisher sending messages and the subscriber receiving them.

## Understanding the Code

### Publisher Breakdown

- `self.create_publisher(String, 'example_topic', 10)`: Creates a publisher that sends `String` messages to the topic `example_topic`. The last parameter (10) is the queue size for outgoing messages.

- `self.create_timer(timer_period, self.timer_callback)`: Creates a timer that calls `timer_callback` every 0.5 seconds.

- `self.publisher_.publish(msg)`: Publishes the message to the topic.

### Subscriber Breakdown

- `self.create_subscription(String, 'example_topic', self.listener_callback, 10)`: Creates a subscription to the `example_topic` that expects `String` messages. When a message is received, `listener_callback` is called.

- `self.listener_callback(msg)`: This function is called whenever a message is received on the topic.

## Message Types

ROS 2 comes with many built-in message types in packages like `std_msgs`, `geometry_msgs`, `sensor_msgs`, etc. You can also define your own message types. Some common message types include:

- `std_msgs/String`: Simple string message
- `std_msgs/Int32`: 32-bit integer
- `std_msgs/Float64`: 64-bit floating point
- `geometry_msgs/Twist`: For velocity commands
- `sensor_msgs/LaserScan`: For laser scanner data

## Quality of Service (QoS)

When creating publishers and subscribers, you can specify Quality of Service (QoS) settings that control how messages are delivered. These settings affect reliability, durability, and other aspects of message delivery.

```python
from rclpy.qos import QoSProfile

# Create a QoS profile with specific settings
qos_profile = QoSProfile(depth=10)
# You can also specify reliability, durability, etc.
```

## Best Practices

1. **Use descriptive topic names**: Choose clear, consistent names that indicate the purpose of the topic.

2. **Handle exceptions**: Always consider what happens if a node crashes or loses connection.

3. **Match message types**: Ensure that publishers and subscribers use the same message types.

4. **Use appropriate QoS settings**: Consider the requirements of your application when choosing QoS settings.

## Summary

The publisher-subscriber pattern is essential for ROS 2 communication. It enables decoupled, asynchronous communication between nodes in a robotic system. By creating publishers and subscribers, you can build complex systems where different components communicate through well-defined interfaces.

In the next section, we'll explore services, which provide a request-response communication pattern that complements the pub/sub model.