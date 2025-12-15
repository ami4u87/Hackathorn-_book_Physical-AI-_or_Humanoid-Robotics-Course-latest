# Services

Services in ROS 2 provide a request-response communication pattern that is different from the publisher-subscriber model. In service-based communication, a service client sends a request to a service server, which processes the request and sends back a response. This is useful for operations that require immediate feedback or when you need to ensure that a specific task is completed before proceeding.

## Understanding Services

A service has two parts:
- **Service Request**: The data sent from the client to the server
- **Service Response**: The data sent from the server back to the client

Services are synchronous, meaning the client waits for the response before continuing execution. This makes them suitable for operations like:
- Calculating a result based on input parameters
- Triggering an action and waiting for completion confirmation
- Retrieving specific information from a node

## Creating a Service Definition

First, let's create a custom service definition. Create a file called `AddTwoInts.srv` in a new package:

```bash
# In your workspace directory
mkdir -p src/my_robot_msgs/srv
```

Create the service definition in `src/my_robot_msgs/srv/AddTwoInts.srv`:

```
int64 a
int64 b
---
int64 sum
```

This service takes two integers as input and returns their sum.

## Creating a Service Server

Let's create a service server that implements our custom service.

### Python Service Server Example

Create a new package for services:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_srv
```

Create the service server at `src/py_srv/py_srv/service_member_function.py`:

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}\n')
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating a Service Client

Now let's create a service client that sends requests to our service server.

### Python Service Client Example

Create the client at `src/py_srv/py_srv/client_member_function.py`:

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        f'Result of add_two_ints: for {1} + {2} = {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Using Built-in Services

ROS 2 also provides many built-in services. For example, you can use the parameter services to get and set parameters on a node:

```bash
# List services provided by a node
ros2 service list

# Get information about a specific service
ros2 service info <service_name>

# Call a service directly from the command line
ros2 service call /add_two_ints add_two_ints_srv/srv/AddTwoInts "{a: 1, b: 2}"
```

## Running the Service Example

1. First, build your workspace (you may need to update the package.xml to include dependencies):

```bash
cd ~/ros2_ws
colcon build --packages-select py_srv
```

2. Source the workspace:

```bash
source install/setup.bash
```

3. Run the service server in one terminal:

```bash
ros2 run py_srv service_member_function
```

4. In another terminal, run the client:

```bash
ros2 run py_srv client_member_function
```

You should see the client send a request and receive a response from the server.

## Service vs. Publisher-Subscriber

| Feature | Publisher-Subscriber | Service |
|---------|---------------------|---------|
| Communication Pattern | Asynchronous | Synchronous |
| Connection Type | Many-to-many | One-to-one |
| Response Required | No | Yes |
| Use Case | Continuous data streams | Request-response operations |

## Advanced Service Concepts

### Service Quality of Service (QoS)

Like topics, services also support QoS settings:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

# Create a QoS profile for services
qos_profile = QoSProfile(depth=10)
```

### Service Error Handling

When calling services, it's important to handle potential errors:

```python
def send_request_with_error_handling(self, a, b):
    self.req.a = a
    self.req.b = b

    try:
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            return self.future.result()
        else:
            self.get_logger().error('Service call failed')
            return None
    except Exception as e:
        self.get_logger().error(f'Service call failed with exception: {e}')
        return None
```

## Best Practices

1. **Use appropriate communication pattern**: Use services for operations that require a response, and topics for continuous data streams.

2. **Handle service timeouts**: Always implement timeout handling for service calls to prevent indefinite blocking.

3. **Design clear service interfaces**: Service requests and responses should be well-defined and intuitive.

4. **Consider performance**: Services are synchronous, so long-running operations should be avoided or implemented asynchronously.

5. **Use built-in services when available**: ROS 2 provides many standard services that may meet your needs.

## Summary

Services provide a synchronous request-response communication pattern that complements the publisher-subscriber model. They are essential for operations that require immediate feedback or confirmation. Understanding when to use services versus topics is crucial for effective ROS 2 system design.

In the next section, we'll explore actions, which provide a more sophisticated communication pattern for long-running tasks with feedback and goal management.