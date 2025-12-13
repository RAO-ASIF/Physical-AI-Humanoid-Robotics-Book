---
title: Python Integration with ROS 2
sidebar_position: 5
---

# Python Integration with ROS 2

Python is one of the most popular languages for robotics development, especially for rapid prototyping, testing, and AI integration. The Robot Operating System 2 (ROS 2) provides excellent Python support through the `rclpy` client library, enabling developers to create sophisticated robotic applications with minimal overhead.

## Understanding rclpy

`rclpy` is the Python client library for ROS 2, providing a Python API to the ROS 2 client library (rcl). It abstracts the complexities of the underlying C implementation while maintaining performance and functionality.

### Basic Node Structure

Every ROS 2 Python node follows the same basic structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code here
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers and Subscribers

### Creating Publishers

Publishers send messages to topics and are essential for the publish-subscribe communication pattern:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        # Create publisher with topic name and queue size
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)

        # Create timer to publish messages at regular intervals
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
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating Subscribers

Subscribers receive messages from topics and process them:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create subscription with topic name, message type, and callback
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)  # Queue size
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services and Clients

### Service Servers

Services provide synchronous request-response communication:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')

        # Create service with service name and callback function
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Clients

Service clients call services and wait for responses:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')

        # Create client with service type and service name
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b

        # Call service asynchronously
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()

    # Send request
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(f'Result: {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions

Actions provide a way to handle long-running tasks with feedback:

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')

        # Create action server with action type, action name, and callback
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Returning result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Python Integration

### Using Parameters

Parameters allow configuration of nodes at runtime:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('sensors', ['camera', 'lidar'])

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.sensors = self.get_parameter('sensors').value

        # Create parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.type_ == Parameter.Type.DOUBLE:
                if param.value > 2.0:
                    return SetParametersResult(successful=False, reason='Max velocity too high')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Custom Message Types

Create custom message types by defining `.msg` files and using them in Python:

```python
# Assuming we have a custom message defined as HumanoidState.msg:
# float64[] joint_positions
# float64[] joint_velocities
# geometry_msgs/Pose base_pose

import rclpy
from rclpy.node import Node
from my_robot_msgs.msg import HumanoidState  # Custom message type

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.publisher_ = self.create_publisher(HumanoidState, 'humanoid_state', 10)

    def publish_state(self, joint_positions, joint_velocities, base_pose):
        msg = HumanoidState()
        msg.joint_positions = joint_positions
        msg.joint_velocities = joint_velocities
        msg.base_pose = base_pose
        self.publisher_.publish(msg)
```

### TF Transformations

Working with coordinate transformations:

```python
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)

    def broadcast_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with AI and Machine Learning

### TensorFlow/Keras Integration

```python
import rclpy
from rclpy.node import Node
import tensorflow as tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Load pre-trained model
        self.model = tf.keras.models.load_model('path/to/model.h5')

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocess image
        input_image = cv2.resize(cv_image, (224, 224))
        input_image = np.expand_dims(input_image, axis=0) / 255.0

        # Run inference
        predictions = self.model.predict(input_image)

        # Process results
        self.get_logger().info(f'Predictions: {predictions}')
```

### PyTorch Integration

```python
import rclpy
from rclpy.node import Node
import torch
import torch.nn as nn
from sensor_msgs.msg import PointCloud2
import numpy as np

class PyTorchNode(Node):
    def __init__(self):
        super().__init__('pytorch_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            'lidar/points',
            self.pointcloud_callback,
            10)

        # Initialize PyTorch model
        self.model = self.initialize_model()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(self.device)

    def initialize_model(self):
        # Example simple model
        model = nn.Sequential(
            nn.Linear(3, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 1)
        )
        return model

    def pointcloud_callback(self, msg):
        # Process point cloud data
        # ... convert PointCloud2 to tensor ...

        with torch.no_grad():
            # Run inference
            output = self.model(input_tensor)

        self.get_logger().info(f'PyTorch output: {output}')
```

## Error Handling and Best Practices

### Robust Node Design

```python
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
import traceback

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')

        # Initialize with error handling
        try:
            self.setup_subscriptions()
            self.setup_publishers()
            self.setup_parameters()
        except Exception as e:
            self.get_logger().error(f'Failed to initialize node: {e}')
            self.get_logger().error(traceback.format_exc())
            raise

    def setup_subscriptions(self):
        try:
            self.subscription = self.create_subscription(
                String,
                'topic',
                self.callback,
                10)
        except Exception as e:
            self.get_logger().error(f'Failed to create subscription: {e}')

    def callback(self, msg):
        try:
            # Process message
            self.process_message(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')
            self.get_logger().error(traceback.format_exc())

    def process_message(self, msg):
        # Your processing logic here
        pass
```

### Lifecycle Nodes

```python
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleTestNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_test_node')
        self.get_logger().info('Lifecycle Test Node created')

    def on_configure(self, state):
        self.get_logger().info('Configuring lifecycle node')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating lifecycle node')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating lifecycle node')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up lifecycle node')
        return TransitionCallbackReturn.SUCCESS
```

## Launch Files for Python Nodes

Create launch files to manage multiple Python nodes:

```python
# launch/humanoid_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='robot_controller',
            parameters=[
                {'robot_name': 'humanoid_robot'},
                {'max_velocity': 1.0}
            ],
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='sensor_processor',
            name='sensor_processor',
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='ai_controller',
            name='ai_controller',
            output='screen'
        )
    ])
```

## Performance Considerations

### Threading and Concurrency

```python
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

class MultiThreadedNode(Node):
    def __init__(self):
        super().__init__('multithreaded_node')

        # Create separate callback groups for different threads
        cb_group = MutuallyExclusiveCallbackGroup()

        self.subscription = self.create_subscription(
            String,
            'topic',
            self.callback,
            10,
            callback_group=cb_group)

    def callback(self, msg):
        # This callback will run in a separate thread
        self.process_data_in_thread(msg)

    def process_data_in_thread(self, msg):
        thread = threading.Thread(target=self.heavy_computation, args=(msg,))
        thread.start()

def main(args=None):
    rclpy.init(args=args)

    node = MultiThreadedNode()

    # Use multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
```

## Debugging Python ROS 2 Nodes

### Common Debugging Techniques

```python
import rclpy
from rclpy.node import Node
import pdb  # Python debugger

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')

        # Add debugging statements
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Create publisher and subscriber
        self.pub = self.create_publisher(String, 'debug_topic', 10)
        self.sub = self.create_subscription(String, 'input_topic', self.debug_callback, 10)

    def debug_callback(self, msg):
        # Use debugging statements
        self.get_logger().debug(f'Received: {msg.data}')

        # Set breakpoint for interactive debugging
        # pdb.set_trace()  # Uncomment to debug interactively

        # Process message
        processed_msg = self.process_message(msg)

        self.get_logger().info(f'Processed: {processed_msg}')
        self.pub.publish(processed_msg)

    def process_message(self, msg):
        # Add validation
        if not msg.data:
            self.get_logger().warn('Empty message received')
            return String()

        # Process and return
        result = String()
        result.data = f'Processed: {msg.data}'
        return result
```

## Summary

Python integration with ROS 2 provides a powerful and flexible platform for robotics development. The `rclpy` library enables seamless integration of Python's rich ecosystem of scientific computing and machine learning libraries with ROS 2's robust communication infrastructure. By following best practices for node design, error handling, and performance optimization, developers can create sophisticated robotic applications that leverage the strengths of both Python and ROS 2.

The integration of AI/ML libraries like TensorFlow, PyTorch, and scikit-learn with ROS 2 enables the development of intelligent robotic systems that can perceive, reason, and act in complex environments. Proper use of parameters, services, actions, and TF transformations ensures robust and maintainable robotic systems that can adapt to different configurations and environments.