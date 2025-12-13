---
title: Nodes, Topics, and Services
sidebar_position: 3
---

# Nodes, Topics, and Services

In ROS 2, the communication between different components of a robotic system is achieved through a distributed network of nodes that communicate using topics, services, and actions. Understanding these communication mechanisms is fundamental to developing effective robotic systems.

## Nodes

A node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. In a typical robot application, there might be one node that controls the wheel motors, another that processes camera images, and another that performs localization.

### Creating Nodes in Python

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Characteristics

- **Unique Names**: Each node must have a unique name within the ROS graph
- **Parameter Server**: Nodes can store and retrieve configuration parameters
- **Lifecycle Management**: Nodes can be managed through lifecycle states
- **Namespaces**: Nodes can be organized using namespaces for better structure

## Topics and Publishing/Subscribing

Topics enable asynchronous communication through a publish/subscribe model. Publishers send messages to topics, and subscribers receive messages from topics. This decouples the timing of message production and consumption.

### Topic Communication

```
Publisher Node A ─────┐
                      ├── Topic: /robot_commands
Publisher Node B ─────┤
                      │
                ┌─────▼─────┐
                │ Message   │
                │ Buffer    │
                └─────┬─────┘
                      │
Subscriber Node C ────┤
                      ├── Topic: /robot_commands
Subscriber Node D ────┘
```

### Python Example: Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher = self.create_publisher(String, 'robot_commands', 10)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {command}')
```

### Python Example: Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        # Process the command here
        self.execute_command(msg.data)

    def execute_command(self, command):
        # Implement command execution logic
        pass
```

## Services

Services provide synchronous request/response communication. A service client sends a request and waits for a response from a service server. This is useful for operations that need immediate results or acknowledgment.

### Service Communication

```
Service Client ──┐
                 ├── Request: Calculate IK
                 │
           ┌─────▼─────┐
           │ Service   │
           │ Server    │
           └─────┬─────┘
                 │
                 ├── Response: Joint Angles
Service Client ──┘
```

### Defining Services

Services are defined using `.srv` files that specify the request and response message types:

```
# Request (before ---)
float64 x
float64 y
float64 z
---
# Response (after ---)
float64[] joint_angles
bool success
string message
```

### Python Example: Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

### Python Example: Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions

Actions provide a way to handle long-running tasks with feedback. They combine the features of services (request/response) with the ability to monitor progress and cancel operations.

### Action Communication

```
Action Client ──┐
                ├── Goal: Navigate to x,y
                │
          ┌─────▼─────┐
          │ Action    │
          │ Server    │
          └─────┬─────┘
                │
                ├── Feedback: Progress 50%
                │
                ├── Result: Navigation Complete
Action Client ──┘
```

## Quality of Service (QoS)

ROS 2 provides Quality of Service profiles to configure how messages are delivered:

- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient local
- **History**: Keep last N messages or keep all
- **Deadline**: Maximum time between messages
- **Liveliness**: How to detect if a publisher is alive

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# For real-time sensor data
sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST
)

# For critical commands
command_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)
```

## Practical Example: Robot Command System

Here's a complete example that demonstrates nodes, topics, and services working together:

```python
# robot_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from example_interfaces.srv import Trigger

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        # Subscribers
        self.command_sub = self.create_subscription(
            String, 'robot_commands', self.command_callback, 10)

        # Services
        self.reset_service = self.create_service(
            Trigger, 'reset_robot', self.reset_callback)

        self.get_logger().info('Robot controller initialized')

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        if command == 'forward':
            self.move_forward()
        elif command == 'stop':
            self.stop_robot()
        elif command == 'rotate':
            self.rotate_robot()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.5  # m/s
        self.cmd_vel_pub.publish(twist)
        self.publish_status('Moving forward')

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.publish_status('Stopped')

    def rotate_robot(self):
        twist = Twist()
        twist.angular.z = 0.5  # rad/s
        self.cmd_vel_pub.publish(twist)
        self.publish_status('Rotating')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def reset_callback(self, request, response):
        self.stop_robot()
        response.success = True
        response.message = 'Robot reset successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

### Node Design
- Keep nodes focused on a single responsibility
- Use meaningful node names
- Implement proper error handling
- Use parameters for configuration

### Topic Design
- Use descriptive topic names
- Consider message frequency and bandwidth
- Choose appropriate QoS settings
- Use standard message types when possible

### Service Design
- Use services for operations requiring immediate response
- Implement proper error handling in service callbacks
- Consider timeout values for service calls
- Use standard service types when available

## Summary

Nodes, topics, and services form the backbone of ROS 2 communication. Understanding these concepts is crucial for developing modular, maintainable robotic systems. The publish/subscribe model of topics provides loose coupling between components, while services enable synchronous request/response communication for operations requiring immediate results. Together, these mechanisms enable the creation of complex robotic systems from simple, reusable components.