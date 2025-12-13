---
title: ROS 2 Fundamentals
sidebar_label: Chapter 2 - ROS 2 Fundamentals
description: Comprehensive introduction to ROS 2 fundamentals including nodes, topics, services, and URDF for humanoid robot modeling
---

# Chapter 2: ROS 2 Fundamentals

## Learning Objectives

By the end of this chapter, you should be able to:
- Understand the ROS 2 architecture and middleware concepts
- Create and run basic ROS 2 nodes
- Implement topic publishing and subscribing
- Develop services and actions
- Create URDF models for humanoid robot components and joints
- Use Python rclpy for ROS 2 programming

## Table of Contents
- [ROS 2 Architecture](#ros-2-architecture)
- [Nodes, Topics, and Services](#nodes-topics-and-services)
- [Python rclpy Programming](#python-rclpy-programming)
- [URDF Robot Modeling](#urdf-robot-modeling)
- [TF Transform System](#tf-transform-system)
- [Launch Files and Parameters](#launch-files-and-parameters)
- [Chapter Summary](#chapter-summary)
- [Exercises](#exercises)

## ROS 2 Architecture

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Key Concepts

- **Nodes**: Processes that perform computation
- **Topics**: Named buses over which nodes exchange messages
- **Services**: Synchronous request/response communication
- **Actions**: Asynchronous goal-oriented communication
- **Parameters**: Configuration values that nodes can use

### Middleware Implementation

ROS 2 uses DDS (Data Distribution Service) as its middleware, providing:
- Reliable message delivery
- Quality of Service (QoS) settings
- Language independence
- Platform independence

### Architecture Components

#### Nodes
Nodes are the fundamental building blocks of a ROS 2 system. Each node runs a specific task and communicates with other nodes through messages. Nodes can be written in different programming languages (C++, Python, etc.) and still communicate seamlessly.

#### Topics and Messages
Topics provide a publish-subscribe communication pattern:
- Publishers send data to topics
- Subscribers receive data from topics
- Messages are strongly-typed data structures that define the format of information passed between nodes

#### Services
Services provide request-response communication:
- A client sends a request to a service
- The service processes the request and returns a response
- This is synchronous communication

#### Actions
Actions are for long-running tasks with feedback:
- Goal: Request to perform a task
- Feedback: Updates during task execution
- Result: Final outcome of the task

### Quality of Service (QoS)

QoS settings allow fine-tuning communication behavior:
- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient local history
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to determine if a publisher is alive

### Client Library Architecture

ROS 2 uses a layered architecture:
- **Application Layer**: User code using client libraries (rclpy, rclcpp)
- **Client Library Layer**: Language-specific APIs
- **ROS Client Library (rcl)**: Common C-based library
- **DDS Abstraction Layer**: Abstracts different DDS implementations
- **DDS Implementation**: Specific DDS vendor implementation

## Nodes, Topics, and Services

### Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are organized into packages for better software management.

#### Creating a Node

In Python using rclpy:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Node initialization code here
        self.get_logger().info('MyNode has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics

Topics enable asynchronous message passing between nodes using a publish/subscribe pattern:
- Publishers send messages to topics
- Subscribers receive messages from topics
- Multiple publishers and subscribers can exist for the same topic

#### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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

#### Advanced Publisher Example - Robot Sensor Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import random

class RobotSensorPublisher(Node):
    def __init__(self):
        super().__init__('robot_sensor_publisher')

        # Publisher for laser scan data
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Publisher for robot velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for publishing sensor data
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

        self.get_logger().info('Robot sensor publisher started')

    def publish_sensor_data(self):
        # Create and publish laser scan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        scan_msg.angle_min = -math.pi / 2
        scan_msg.angle_max = math.pi / 2
        scan_msg.angle_increment = math.pi / 180  # 1 degree
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0

        # Generate random range data (simulating sensor readings)
        num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        scan_msg.ranges = [random.uniform(0.5, 5.0) for _ in range(num_readings)]
        scan_msg.intensities = [1.0] * num_readings

        self.laser_pub.publish(scan_msg)

        # Publish random velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = random.uniform(0.0, 0.5)  # Forward velocity
        cmd_msg.angular.z = random.uniform(-0.5, 0.5)  # Angular velocity
        self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = RobotSensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Advanced Subscriber Example - Robot Controller

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for obstacle detection status
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)

        self.safe_distance = 1.0  # meters
        self.get_logger().info('Robot controller initialized')

    def scan_callback(self, msg):
        # Find minimum distance in front of the robot (±30 degrees)
        front_ranges = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i, range_val in enumerate(msg.ranges):
            angle = angle_min + i * angle_increment
            if -math.pi/6 <= angle <= math.pi/6:  # Front 60 degrees
                if not math.isnan(range_val) and range_val > 0:
                    front_ranges.append(range_val)

        if front_ranges:
            min_distance = min(front_ranges)
            self.get_logger().info(f'Min front distance: {min_distance:.2f}m')

            # Create and publish velocity command based on obstacle detection
            cmd_msg = Twist()

            if min_distance < self.safe_distance:
                # Obstacle detected - stop and turn
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.5  # Turn right
                self.get_logger().warn('Obstacle detected! Turning...')

                # Publish obstacle detected message
                obstacle_msg = Bool()
                obstacle_msg.data = True
                self.obstacle_pub.publish(obstacle_msg)
            else:
                # Clear path - move forward
                cmd_msg.linear.x = 0.3  # Move forward at 0.3 m/s
                cmd_msg.angular.z = 0.0
                obstacle_msg = Bool()
                obstacle_msg.data = False
                self.obstacle_pub.publish(obstacle_msg)

            # Publish velocity command
            self.cmd_vel_pub.publish(cmd_msg)
        else:
            # No valid readings in front - stop
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Quality of Service (QoS) Examples

When working with topics, you can specify Quality of Service settings to control communication behavior:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create a QoS profile for reliable communication
reliable_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Create a QoS profile for best-effort communication (good for sensor data)
best_effort_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# Using QoS in publisher
self.publisher = self.create_publisher(String, 'topic_name', reliable_qos)

# Using QoS in subscriber
self.subscription = self.create_subscription(
    String,
    'topic_name',
    self.callback,
    best_effort_qos
)
```

### Services

Services provide synchronous request/response communication:
- Client sends a request
- Server processes the request and sends a response
- Communication is bidirectional and blocking

#### Service Server Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service Client Example

```python
import sys
from example_interfaces.srv import AddTwoInts
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
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Python rclpy Programming

rclpy is the Python client library for ROS 2. It provides Python APIs for creating ROS 2 nodes, publishers, subscribers, services, and more.

### Basic Node Structure

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

### Core Concepts in rclpy

#### Node Creation and Lifecycle

Every ROS 2 Python program starts with creating a node:

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('my_robot_node')

        # Create publishers, subscribers, services, etc.
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a timer for periodic execution
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Log information
        self.get_logger().info('MyRobotNode initialized')

    def timer_callback(self):
        # Code executed periodically
        self.get_logger().info('Timer callback executed')
```

#### Publishers and Subscribers

Creating a publisher:

```python
from std_msgs.msg import String

# In the node's __init__ method
self.publisher = self.create_publisher(String, 'topic_name', 10)
```

Creating a subscriber:

```python
from std_msgs.msg import String

def subscription_callback(self, msg):
    self.get_logger().info(f'Received: {msg.data}')

# In the node's __init__ method
self.subscription = self.create_subscription(
    String,
    'topic_name',
    self.subscription_callback,
    10
)
```

#### Working with Custom Messages

To use custom message types, first define the .msg file, then import and use:

```python
# Assuming you have a custom message RobotStatus.msg
from my_robot_msgs.msg import RobotStatus

# In your node
self.status_publisher = self.create_publisher(RobotStatus, 'robot_status', 10)

def publish_robot_status(self):
    msg = RobotStatus()
    msg.battery_level = 85.5
    msg.operational = True
    msg.timestamp = self.get_clock().now().to_msg()
    self.status_publisher.publish(msg)
```

#### Parameter Handling

Nodes can accept parameters that can be configured at runtime:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('wheel_diameter', 0.15)
        self.declare_parameter('max_velocity', 1.0)

        # Access parameter values
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.max_velocity = self.get_parameter('max_velocity').value

        # Handle parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.type_ == Parameter.Type.PARAMETER_DOUBLE:
                self.max_velocity = param.value
                self.get_logger().info(f'Updated max_velocity to {param.value}')
        return SetParametersResult(successful=True)
```

#### Timer Usage

Timers are useful for periodic tasks:

```python
def __init__(self):
    super().__init__('timed_node')

    # Create a timer that calls the callback every 0.5 seconds
    self.timer = self.create_timer(0.5, self.periodic_callback)

    # Create a one-shot timer (executes once after delay)
    self.one_shot_timer = self.create_timer(2.0, self.one_shot_callback)
    self.one_shot_timer.cancel()  # Cancel if needed

def periodic_callback(self):
    self.get_logger().info('Executing periodic task')

def one_shot_callback(self):
    self.get_logger().info('This executes only once')
    # Cancel the timer after execution
    self.one_shot_timer.cancel()
```

### Best Practices

1. **Always call super().__init__()** with a node name
2. **Use self.get_logger().info()** for logging instead of print()
3. **Store publishers/subscribers as instance variables** to prevent garbage collection
4. **Handle exceptions** in callbacks to prevent node crashes
5. **Use appropriate queue sizes** for publishers and subscribers (typically 10)
6. **Clean up resources** in destroy_node() if needed
7. **Use appropriate QoS profiles** for your application requirements

### Error Handling

Proper error handling in ROS 2 nodes:

```python
import traceback

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.safe_callback,
            10
        )

    def safe_callback(self, msg):
        try:
            # Process the message
            result = self.process_message(msg)
            self.publish_result(result)
        except Exception as e:
            # Log the error but don't crash the node
            self.get_logger().error(f'Error processing message: {e}')
            self.get_logger().debug(traceback.format_exc())

    def process_message(self, msg):
        # Your message processing logic
        if not msg.data:
            raise ValueError("Empty message data")
        return msg.data.upper()
```

## URDF Robot Modeling

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines the robot's physical and visual properties.

### URDF Fundamentals

URDF is essential for:
- Visualizing robots in RViz
- Physics simulation in Gazebo
- Kinematic analysis
- Motion planning

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.25"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.25"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### Links

Links represent rigid bodies in the robot:

```xml
<link name="link_name">
  <!-- Visual properties for display -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Choose one geometry type -->
      <box size="0.1 0.1 0.1"/>
      <!-- OR <cylinder radius="0.1" length="0.2"/> -->
      <!-- OR <sphere radius="0.1"/> -->
      <!-- OR <mesh filename="package://my_robot/meshes/link.stl"/> -->
    </geometry>
    <material name="color">
      <color rgba="0.8 0.2 0.2 1.0"/>
    </material>
  </visual>

  <!-- Collision properties for physics simulation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>

  <!-- Inertial properties for physics simulation -->
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>
```

### Joints in URDF

Joints connect links and define the kinematic relationships:

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

#### Joint Types

- **revolute**: Rotational joint with limited range
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint with limits
- **fixed**: No movement (rigid connection)
- **floating**: 6 DOF (rarely used)
- **planar**: Movement in a plane (rarely used)

### Complete Robot Example

Here's a simple differential drive robot:

```xml
<?xml version="1.0"?>
<robot name="diff_drive_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.1 -0.025" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.1 -0.025" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### URDF Best Practices

1. **Use consistent naming conventions**
2. **Organize complex robots with xacro macros**
3. **Include proper inertial properties for simulation**
4. **Use appropriate collision geometries**
5. **Validate URDF with `check_urdf` tool**
6. **Test in RViz before simulation**

### Xacro for Complex Robots

For complex robots, use Xacro (XML Macros) to simplify URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Define properties -->
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.02"/>

  <!-- Macro for wheels -->
  <xacro:macro name="wheel" params="prefix parent x y z">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x} ${y} ${z}" rpy="1.5708 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:wheel prefix="left" parent="base_link" x="0" y="0.1" z="-0.025"/>
  <xacro:wheel prefix="right" parent="base_link" x="0" y="-0.1" z="-0.025"/>
</robot>
```

## TF Transform System

The TF (Transform) system keeps track of multiple coordinate frames over time. It allows for the computation of the pose of any frame relative to any other frame at any time.

### Using TF in Code

```python
import tf2_ros
from tf2_ros import TransformBroadcaster

class TransformPublisher(Node):
    def __init__(self):
        super().__init__('transform_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        # Publish transforms at a specific rate
```

## Launch Files and Parameters

Launch files allow you to start multiple nodes with a single command and configure parameters.

### Example Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            parameters=[
                {'param_name': 'param_value'}
            ]
        )
    ])
```

## Chapter Summary

This chapter covered the fundamental concepts of ROS 2, including its architecture, communication patterns, and robot modeling with URDF. You learned how to create nodes using Python rclpy and understand the transform system.

## Exercises

1. Create a simple publisher and subscriber in ROS 2
2. Build a URDF model for a simple robot arm
3. Implement a service that responds to requests

## Hands-on Exercises for ROS 2 Node Creation

### Exercise 1: Basic Publisher Node
Create a simple ROS 2 publisher node that publishes "Hello, ROS 2!" messages to a topic called "greetings" every 2 seconds.

**Steps:**
1. Create a new Python file called `greeting_publisher.py`
2. Implement a node that inherits from `rclpy.node.Node`
3. Create a publisher for String messages on the "greetings" topic
4. Use a timer to publish messages every 2 seconds
5. Add the node to a launch file

### Exercise 2: Subscriber Node
Create a subscriber node that listens to the "greetings" topic and logs received messages.

**Steps:**
1. Create a new Python file called `greeting_subscriber.py`
2. Implement a node that subscribes to the "greetings" topic
3. Create a callback function to handle incoming messages
4. Log received messages to the console

### Exercise 3: Service Server and Client
Create a simple service that adds two numbers and implement both server and client nodes.

**Steps:**
1. Create a service server that implements addition of two integers
2. Create a service client that calls the service with test values
3. Test the communication between client and server

### Exercise 4: Simple Robot Controller
Create a node that publishes Twist messages to control a robot's movement.

**Steps:**
1. Create a publisher for geometry_msgs/Twist messages on the "/cmd_vel" topic
2. Implement logic to move the robot forward for 3 seconds, then stop
3. Add functionality to turn the robot left for 1 second
4. Create a launch file that runs the controller node

### Exercise 5: Parameter-Based Node
Create a node that accepts parameters for configuring its behavior.

**Steps:**
1. Create a node that declares parameters for speed and duration
2. Use the parameter values to control how long and how fast the robot moves
3. Test the node with different parameter values

### Exercise 6: URDF Visualization
Create a simple URDF file for a robot with a base and one rotating joint.

**Steps:**
1. Create a URDF file with a base link and a rotating wheel
2. Define the joint between the base and wheel
3. Add visual and collision properties
4. Use RViz to visualize the robot model
5. Use joint_state_publisher to animate the joint

### Exercise Solutions

**Exercise 1 Solution:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GreetingPublisher(Node):
    def __init__(self):
        super().__init__('greeting_publisher')
        self.publisher_ = self.create_publisher(String, 'greetings', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    greeting_publisher = GreetingPublisher()
    rclpy.spin(greeting_publisher)
    greeting_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Exercise 2 Solution:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GreetingSubscriber(Node):
    def __init__(self):
        super().__init__('greeting_subscriber')
        self.subscription = self.create_subscription(
            String,
            'greetings',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received greeting: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    greeting_subscriber = GreetingSubscriber()
    rclpy.spin(greeting_subscriber)
    greeting_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## URDF Modeling Exercises with Joints and Links

### Exercise 7: Robot Arm with Multiple Joints
Create a simple 3-DOF robot arm with a base, upper arm, and forearm connected by revolute joints.

**Steps:**
1. Create a base link as the fixed root of the robot
2. Add an upper arm link connected to the base with a shoulder joint (rotates around Y-axis)
3. Add a forearm link connected to the upper arm with an elbow joint (rotates around Y-axis)
4. Add a simple gripper or end effector
5. Ensure proper inertial, visual, and collision properties for each link
6. Test the model in RViz with joint_state_publisher

### Exercise 8: Mobile Robot with Differential Drive
Create a wheeled robot with a chassis and two independently controlled wheels.

**Steps:**
1. Create a chassis/base link as the main body
2. Add left and right wheel links with appropriate geometry
3. Create continuous joints for each wheel that rotate around their axis
4. Add caster wheels for stability (fixed joints)
5. Set appropriate mass and inertia values
6. Test the kinematic model in RViz

### Exercise 9: Humanoid Leg Structure
Create a simplified leg with hip, knee, and ankle joints.

**Steps:**
1. Create a pelvis/torso link as the root
2. Add a thigh link with a hip joint (3 DOF: pitch, roll, yaw)
3. Add a shin link with a knee joint (1 DOF: pitch)
4. Add a foot link with an ankle joint (2 DOF: pitch, roll)
5. Include realistic proportions based on human anatomy
6. Add visual and collision models

### Exercise 10: Sensor Integration in URDF
Add sensors to your robot model such as cameras, IMU, or LIDAR.

**Steps:**
1. Create a robot base (you can extend from Exercise 8)
2. Add a camera sensor mounted on a pan-tilt mechanism
3. Include an IMU sensor in the main body
4. Add a 2D LIDAR sensor on top of the robot
5. Define appropriate mounting positions and orientations
6. Include sensor plugins in the URDF for simulation

### Exercise Solutions

**Exercise 7 Solution: 3-DOF Robot Arm**
```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Upper arm -->
  <link name="upper_arm">
    <visual>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Forearm -->
  <link name="forearm">
    <visual>
      <origin xyz="0 0 0.2"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0.2"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <!-- End effector -->
  <link name="gripper">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0.0 1.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="wrist_joint" type="fixed">
    <parent link="forearm"/>
    <child link="gripper"/>
    <origin xyz="0 0 0.4"/>
  </joint>
</robot>
```

## Further Reading

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)

## References

1. ROS.org. (2023). *ROS 2 Documentation: Humble Hawksbill*. Open Robotics. https://docs.ros.org/en/humble/

2. ROS.org. (2023). *ROS 2 Tutorials*. Open Robotics. https://docs.ros.org/en/humble/Tutorials.html

3. Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming Robots with ROS: A Practical Introduction to the Robot Operating System*. O'Reilly Media.

4. ROS.org. (2023). *URDF Tutorials*. Open Robotics. http://wiki.ros.org/urdf/Tutorials

5. Fauser, M., & Enescu, V. (2021). *Effective Robotics Programming with ROS 3: Learn how to apply the core ROS concepts and design and build robot applications*. Packt Publishing.

6. Open Robotics. (2023). *ROS 2 Design: Data Distribution Service (DDS)*. https://design.ros2.org/articles/dds.html

7. Coltin, B., & Velasquez, J. C. (2019). *ROS Robotics Projects: Build and program popular robotic applications with ROS*. Packt Publishing.

8. MIT OpenCourseWare. (2022). *Introduction to Robotics: Kinematics, Dynamics, and Control*. Massachusetts Institute of Technology. https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-141-robotics-science-and-systems-fall-2015/

9. Siciliano, B., & Khatib, O. (2016). *Springer Handbook of Robotics*. Springer International Publishing. https://doi.org/10.1007/978-3-319-32552-1

10. Corke, P. (2017). *Robotics, Vision and Control: Fundamental Algorithms in MATLAB* (2nd ed.). Springer International Publishing. https://doi.org/10.1007/978-3-319-54413-7

11. NVIDIA Corporation. (2023). *Isaac ROS: Hardware-accelerated perception and navigation packages*. https://nvidia-isaac-ros.github.io/

12. Apache Software Foundation. (2023). *Apache 2.0 License*. https://www.apache.org/licenses/LICENSE-2.0

## Assessment

1. Explain the difference between topics and services in ROS 2.
2. Create a simple URDF model with at least 3 links and 2 joints.
3. Implement a publisher node that publishes messages to a topic.

## Assessment Materials for ROS 2 Fundamentals

### Knowledge Check Questions

1. **What is the primary difference between a ROS 2 topic and a service?**
   a) Topics are for real-time communication, services are not
   b) Topics use publish-subscribe model, services use request-response model
   c) Topics can only send strings, services can send any data type
   d) Topics are synchronous, services are asynchronous

2. **Which middleware does ROS 2 use for communication?**
   a) TCP/IP
   b) DDS (Data Distribution Service)
   c) HTTP
   d) MQTT

3. **What does QoS stand for in ROS 2?**
   a) Quality of Service
   b) Quick Operating System
   c) Quantum Operating System
   d) Query Operating Service

4. **Which of the following is NOT a valid joint type in URDF?**
   a) revolute
   b) continuous
   c) prismatic
   d) oscillating

5. **What is the purpose of the TF (Transform) system in ROS?**
   a) To store sensor data
   b) To keep track of coordinate frames over time
   c) To manage robot parameters
   d) To control robot movement

### Practical Assessment Tasks

**Task 1: Node Implementation (40 points)**
- Create a ROS 2 publisher node that publishes temperature sensor readings
- Create a subscriber node that receives and processes these readings
- Implement proper error handling
- Document the code with comments

**Task 2: URDF Modeling (35 points)**
- Design a URDF model for a simple robot with at least 4 links and 3 joints
- Include proper visual, collision, and inertial properties
- Validate the URDF using `check_urdf` tool
- Test the model in RViz

**Task 3: Message Communication (25 points)**
- Create a custom message type for robot status
- Implement a service that responds to robot state queries
- Create a launch file that starts all required nodes
- Demonstrate the system functionality

### Answer Key

**Knowledge Check Answers:**
1. b) Topics use publish-subscribe model, services use request-response model
2. b) DDS (Data Distribution Service)
3. a) Quality of Service
4. d) oscillating
5. b) To keep track of coordinate frames over time

### Grading Rubric

**Task 1: Node Implementation**
- Correct node structure and inheritance: 10 points
- Proper publisher/subscriber implementation: 10 points
- Error handling: 10 points
- Code documentation: 10 points

**Task 2: URDF Modeling**
- Correct link definitions: 10 points
- Proper joint definitions: 10 points
- Visual/collision/inertial properties: 10 points
- URDF validation and testing: 5 points

**Task 3: Message Communication**
- Custom message definition: 8 points
- Service implementation: 8 points
- Launch file: 5 points
- System demonstration: 4 points

### Learning Objectives Assessment

After completing this chapter, students should be able to:

1. **Architecture Understanding (25%)**: Demonstrate knowledge of ROS 2 architecture and middleware concepts
2. **Node Development (30%)**: Create and run basic ROS 2 nodes with proper communication patterns
3. **URDF Modeling (25%)**: Create URDF models for robot components with appropriate joints and links
4. **Programming Skills (20%)**: Use Python rclpy for ROS 2 programming with proper practices

### Self-Assessment Checklist

Students should be able to:
- [ ] Create a ROS 2 node that inherits from rclpy.node.Node
- [ ] Implement publishers and subscribers for message passing
- [ ] Create services for request-response communication
- [ ] Design URDF models with proper links and joints
- [ ] Use parameters to configure node behavior
- [ ] Apply QoS policies appropriately
- [ ] Test nodes and models in simulation environments
- [ ] Debug common ROS 2 communication issues