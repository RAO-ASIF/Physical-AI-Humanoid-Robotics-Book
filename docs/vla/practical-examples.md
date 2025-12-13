---
title: Practical VLA Examples
sidebar_position: 5
---

# Practical VLA Examples

This section provides hands-on examples of Vision-Language-Action (VLA) systems in humanoid robotics. These examples demonstrate the practical implementation of VLA concepts, showing how vision, language, and action components work together to create intelligent robot behaviors.

## Example 1: Fetch and Carry Task

This example demonstrates a complete VLA pipeline where a humanoid robot understands a natural language request, perceives objects in the environment, navigates to the target object, grasps it, and delivers it to the user.

### System Architecture

```
User Command -> Language Understanding -> Task Planning -> Navigation -> Object Perception -> Grasping -> Delivery
```

### Language Understanding Component

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json
import re

class FetchTaskLanguageUnderstandingNode(Node):
    def __init__(self):
        super().__init__('fetch_task_language_understanding_node')

        # Publishers
        self.task_plan_pub = self.create_publisher(String, 'fetch_task_plan', 10)
        self.object_request_pub = self.create_publisher(String, 'object_request', 10)

        # Subscribers
        self.user_command_sub = self.create_subscription(
            String, 'user_command', self.user_command_callback, 10
        )

        # Object vocabulary
        self.object_keywords = {
            'beverage': ['water', 'bottle', 'cup', 'coffee', 'tea', 'soda'],
            'food': ['apple', 'banana', 'snack', 'food'],
            'personal': ['keys', 'phone', 'wallet', 'glasses'],
            'stationery': ['pen', 'pencil', 'book', 'notebook']
        }

        self.get_logger().info('Fetch Task Language Understanding Node initialized')

    def user_command_callback(self, msg):
        """Process user command for fetch task"""
        command = msg.data.lower()
        self.get_logger().info(f'Processing command: {command}')

        # Extract object and location information
        object_info = self.extract_object_info(command)
        location_info = self.extract_location_info(command)

        if object_info:
            # Create task plan
            task_plan = {
                'task_type': 'fetch',
                'target_object': object_info,
                'delivery_location': location_info or 'user_location',
                'priority': 1
            }

            # Publish task plan
            plan_msg = String()
            plan_msg.data = json.dumps(task_plan)
            self.task_plan_pub.publish(plan_msg)

            # Request object detection
            obj_request = String()
            obj_request.data = object_info
            self.object_request_pub.publish(obj_request)

            self.get_logger().info(f'Created fetch task plan: {task_plan}')
        else:
            self.get_logger().warn('Could not extract object information from command')

    def extract_object_info(self, command):
        """Extract object information from command"""
        # Look for known objects in command
        for category, objects in self.object_keywords.items():
            for obj in objects:
                if obj in command:
                    return obj

        # If no known object found, return any noun phrase
        # This is a simplified approach - in practice, use NLP
        words = command.split()
        for i, word in enumerate(words):
            if word in ['the', 'a', 'an', 'some']:
                if i + 1 < len(words):
                    return words[i + 1]

        return None

    def extract_location_info(self, command):
        """Extract location information from command"""
        locations = ['kitchen', 'living room', 'bedroom', 'office', 'dining room']
        for location in locations:
            if location in command:
                return location
        return None

def main(args=None):
    rclpy.init(args=args)
    node = FetchTaskLanguageUnderstandingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down fetch task language understanding node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Object Detection and Localization

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class FetchTaskObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('fetch_task_object_detection_node')

        # Publishers
        self.object_detection_pub = self.create_publisher(Detection2DArray, 'object_detections', 10)
        self.target_object_pub = self.create_publisher(Pose, 'target_object_pose', 10)
        self.search_status_pub = self.create_publisher(String, 'search_status', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        self.object_request_sub = self.create_subscription(
            String, 'object_request', self.object_request_callback, 10
        )

        # CV Bridge
        self.cv_bridge = CvBridge()

        # Target object to search for
        self.target_object = None
        self.object_found = False

        # YOLO model (simplified - in practice, load actual model)
        self.net = None
        self.output_layers = []

        self.get_logger().info('Fetch Task Object Detection Node initialized')

    def object_request_callback(self, msg):
        """Handle object request"""
        self.target_object = msg.data
        self.object_found = False
        self.get_logger().info(f'Searching for object: {self.target_object}')

        status_msg = String()
        status_msg.data = f"searching_for:{self.target_object}"
        self.search_status_pub.publish(status_msg)

    def image_callback(self, msg):
        """Process image for object detection"""
        if not self.target_object:
            return

        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            detections = self.detect_objects(cv_image)

            # Look for target object
            target_detection = None
            for detection in detections:
                if detection['class_name'] == self.target_object:
                    target_detection = detection
                    break

            if target_detection:
                self.object_found = True
                self.get_logger().info(f'Found target object: {self.target_object}')

                # Calculate 3D position (simplified)
                pose_3d = self.calculate_3d_position(target_detection, cv_image.shape)

                # Publish target object pose
                pose_msg = Pose()
                pose_msg.position = pose_3d
                pose_msg.orientation.w = 1.0
                self.target_object_pub.publish(pose_msg)

                # Publish search status
                status_msg = String()
                status_msg.data = f"object_found:{self.target_object}"
                self.search_status_pub.publish(status_msg)
            else:
                # Publish search status
                status_msg = String()
                status_msg.data = f"searching:{self.target_object}"
                self.search_status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_objects(self, image):
        """Detect objects in image (simplified implementation)"""
        # In practice, use a trained object detection model like YOLO
        # This is a placeholder implementation

        # For demonstration, return some mock detections
        height, width = image.shape[:2]

        # Mock detections - in practice, use actual detection model
        mock_detections = [
            {
                'class_name': 'bottle',
                'confidence': 0.85,
                'bbox': [width//2 - 50, height//2 - 50, 100, 100]  # x, y, w, h
            },
            {
                'class_name': 'cup',
                'confidence': 0.78,
                'bbox': [width//3, height//3, 80, 80]
            }
        ]

        return mock_detections

    def calculate_3d_position(self, detection, image_shape):
        """Calculate 3D position of detected object (requires depth info)"""
        # This is a simplified version - in practice, use depth camera or stereo vision
        from geometry_msgs.msg import Point

        bbox = detection['bbox']
        center_x = bbox[0] + bbox[2] / 2
        center_y = bbox[1] + bbox[3] / 2

        # Placeholder 3D position (would use actual depth in real implementation)
        point = Point()
        point.x = 1.0  # Placeholder distance
        point.y = (center_x - image_shape[1]/2) * 0.01  # Approximate lateral offset
        point.z = (center_y - image_shape[0]/2) * 0.01  # Approximate height offset

        return point

def main(args=None):
    rclpy.init(args=args)
    node = FetchTaskObjectDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down fetch task object detection node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Navigation and Grasping Integration

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class FetchTaskNavigationGraspingNode(Node):
    def __init__(self):
        super().__init__('fetch_task_navigation_grasping_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.task_status_pub = self.create_publisher(String, 'fetch_task_status', 10)

        # Subscribers
        self.target_object_sub = self.create_subscription(
            Pose, 'target_object_pose', self.target_object_callback, 10
        )
        self.search_status_sub = self.create_subscription(
            String, 'search_status', self.search_status_callback, 10
        )
        self.task_plan_sub = self.create_subscription(
            String, 'fetch_task_plan', self.task_plan_callback, 10
        )

        # Navigation action client
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Task state
        self.current_task = None
        self.target_pose = None
        self.searching = False
        self.navigating = False
        self.grasping = False

        self.get_logger().info('Fetch Task Navigation Grasping Node initialized')

    def task_plan_callback(self, msg):
        """Handle task plan"""
        task_plan = json.loads(msg.data)
        self.current_task = task_plan
        self.get_logger().info(f'Received task plan: {task_plan}')

        # Start the fetch task
        self.start_fetch_task()

    def target_object_callback(self, msg):
        """Handle target object pose"""
        self.target_pose = msg
        self.get_logger().info('Received target object pose')

        # Navigate to object
        if self.current_task and not self.navigating:
            self.navigate_to_object()

    def search_status_callback(self, msg):
        """Handle search status updates"""
        status = msg.data
        self.get_logger().info(f'Search status: {status}')

        if 'object_found' in status:
            self.searching = False
            if self.target_pose:
                self.navigate_to_object()

    def start_fetch_task(self):
        """Start the fetch task"""
        if not self.current_task:
            return

        self.get_logger().info('Starting fetch task...')

        # Update task status
        status_msg = String()
        status_msg.data = "task_started"
        self.task_status_pub.publish(status_msg)

        # Begin search for object
        self.searching = True

    def navigate_to_object(self):
        """Navigate to detected object"""
        if not self.target_pose or self.navigating:
            return

        self.navigating = True
        self.get_logger().info('Navigating to target object...')

        # In practice, use Nav2 for navigation
        # For this example, we'll simulate navigation
        self.simulate_navigation_to_object()

    def simulate_navigation_to_object(self):
        """Simulate navigation to object"""
        # This is a simplified simulation
        # In practice, use proper navigation stack

        # Move forward for a short distance
        cmd = Twist()
        cmd.linear.x = 0.2  # Move forward at 0.2 m/s
        cmd.angular.z = 0.0

        # Publish command for 3 seconds
        start_time = time.time()
        while time.time() - start_time < 3.0:
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)

        # Stop
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)

        self.get_logger().info('Reached object location')

        # Update status
        status_msg = String()
        status_msg.data = "reached_object"
        self.task_status_pub.publish(status_msg)

        # Start grasping
        self.start_grasping()

    def start_grasping(self):
        """Start grasping operation"""
        self.grasping = True
        self.get_logger().info('Starting grasping operation...')

        # In practice, control the robot's gripper/arm
        # For simulation, just update status
        status_msg = String()
        status_msg.data = "grasping_started"
        self.task_status_pub.publish(status_msg)

        # Simulate grasping time
        time.sleep(2.0)

        # Update status
        status_msg.data = "object_grasped"
        self.task_status_pub.publish(status_msg)

        self.get_logger().info('Object grasped successfully')

        # Deliver to user
        self.deliver_to_user()

    def deliver_to_user(self):
        """Deliver object to user"""
        self.get_logger().info('Delivering object to user...')

        # In practice, navigate to user location
        # For simulation, just update status
        status_msg = String()
        status_msg.data = "delivering_to_user"
        self.task_status_pub.publish(status_msg)

        # Simulate delivery time
        time.sleep(3.0)

        # Update status
        status_msg.data = "task_completed"
        self.task_status_pub.publish(status_msg)

        self.get_logger().info('Fetch task completed successfully')

        # Reset task
        self.reset_task()

    def reset_task(self):
        """Reset task state"""
        self.current_task = None
        self.target_pose = None
        self.searching = False
        self.navigating = False
        self.grasping = False

def main(args=None):
    rclpy.init(args=args)
    node = FetchTaskNavigationGraspingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down fetch task navigation grasping node...')
    finally:
        # Stop robot
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 2: Person Following with VLA

This example demonstrates a VLA system that follows a person based on voice commands and visual tracking.

### Person Following Language Understanding

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json

class PersonFollowingLanguageNode(Node):
    def __init__(self):
        super().__init__('person_following_language_node')

        # Publishers
        self.following_command_pub = self.create_publisher(String, 'following_command', 10)
        self.following_status_pub = self.create_publisher(String, 'following_status', 10)

        # Subscribers
        self.user_command_sub = self.create_subscription(
            String, 'user_command', self.user_command_callback, 10
        )

        # Following modes
        self.following_modes = {
            'follow': 'follow_person',
            'stop': 'stop_following',
            'start': 'start_following',
            'maintain_distance': 'follow_at_distance'
        }

        self.get_logger().info('Person Following Language Node initialized')

    def user_command_callback(self, msg):
        """Process user commands for person following"""
        command = msg.data.lower()
        self.get_logger().info(f'Processing following command: {command}')

        # Determine following action
        action = self.parse_following_command(command)

        if action:
            # Publish following command
            cmd_msg = String()
            cmd_msg.data = action
            self.following_command_pub.publish(cmd_msg)

            # Update status
            status_msg = String()
            status_msg.data = f"command_received:{action}"
            self.following_status_pub.publish(status_msg)

            self.get_logger().info(f'Published following command: {action}')
        else:
            self.get_logger().warn(f'Could not parse following command: {command}')

    def parse_following_command(self, command):
        """Parse following command from natural language"""
        command_lower = command.lower()

        if 'follow' in command_lower or 'track' in command_lower:
            if 'stop' in command_lower or 'halt' in command_lower:
                return 'stop_following'
            elif 'me' in command_lower:
                return 'follow_user'
            elif 'person' in command_lower or 'someone' in command_lower:
                return 'follow_person'
            else:
                return 'follow_person'
        elif 'stop' in command_lower or 'halt' in command_lower:
            return 'stop_following'
        elif 'stay' in command_lower or 'wait' in command_lower:
            return 'wait_for_person'
        elif 'come' in command_lower and 'here' in command_lower:
            return 'stop_following'

        return None

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowingLanguageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down person following language node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Visual Person Tracking

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class PersonTrackingNode(Node):
    def __init__(self):
        super().__init__('person_tracking_node')

        # Publishers
        self.person_location_pub = self.create_publisher(Point, 'person_location', 10)
        self.tracking_status_pub = self.create_publisher(String, 'tracking_status', 10)
        self.velocity_cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        self.following_command_sub = self.create_subscription(
            String, 'following_command', self.following_command_callback, 10
        )

        # CV Bridge
        self.cv_bridge = CvBridge()

        # Tracking state
        self.tracking_active = False
        self.person_location = None
        self.last_person_location = None

        # PID controller parameters for following
        self.kp = 0.5  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.05  # Derivative gain
        self.prev_error_x = 0.0
        self.integral_x = 0.0
        self.prev_error_y = 0.0
        self.integral_y = 0.0

        # Following parameters
        self.target_distance = 1.0  # meters
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.5

        self.get_logger().info('Person Tracking Node initialized')

    def following_command_callback(self, msg):
        """Handle following commands"""
        command = msg.data
        self.get_logger().info(f'Received following command: {command}')

        if command == 'start_following' or command == 'follow_person':
            self.tracking_active = True
            self.get_logger().info('Started person tracking')
        elif command == 'stop_following':
            self.tracking_active = False
            self.stop_robot()
            self.get_logger().info('Stopped person tracking')
        elif command == 'follow_user':
            self.tracking_active = True
            self.get_logger().info('Started user following')

        # Update tracking status
        status_msg = String()
        status_msg.data = f"tracking_active:{self.tracking_active}"
        self.tracking_status_pub.publish(status_msg)

    def image_callback(self, msg):
        """Process image for person detection and tracking"""
        if not self.tracking_active:
            return

        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect persons in image
            person_detections = self.detect_persons(cv_image)

            if person_detections:
                # Use the largest detection (closest person)
                largest_detection = max(person_detections, key=lambda x: x['bbox'][2] * x['bbox'][3])
                self.person_location = self.calculate_person_location(largest_detection, cv_image.shape)

                # Publish person location
                location_msg = Point()
                location_msg.x = self.person_location[0]
                location_msg.y = self.person_location[1]
                location_msg.z = self.person_location[2]
                self.person_location_pub.publish(location_msg)

                # Generate following commands based on person location
                self.generate_following_commands()

                # Update tracking status
                status_msg = String()
                status_msg.data = f"person_tracked:x={self.person_location[0]:.2f},y={self.person_location[1]:.2f},z={self.person_location[2]:.2f}"
                self.tracking_status_pub.publish(status_msg)
            else:
                # No person detected, stop robot
                self.stop_robot()
                self.get_logger().info('No person detected, stopped following')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_persons(self, image):
        """Detect persons in image (simplified implementation)"""
        # In practice, use a person detection model
        # This is a placeholder implementation

        height, width = image.shape[:2]

        # Mock person detection - in practice, use actual detection model
        # Look for typical person-like features in the image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Use HOG descriptor or other person detection method in practice
        # For this example, return mock detections
        mock_detections = [
            {
                'class_name': 'person',
                'confidence': 0.9,
                'bbox': [width//2 - 30, height//3, 60, 120]  # x, y, w, h
            }
        ]

        return mock_detections

    def calculate_person_location(self, detection, image_shape):
        """Calculate 3D location of person (simplified)"""
        # This is a simplified calculation
        # In practice, use depth information or stereo vision

        bbox = detection['bbox']
        center_x = bbox[0] + bbox[2] / 2
        center_y = bbox[1] + bbox[3] / 2

        # Convert pixel coordinates to relative position
        # This is a simplified model - in practice, use proper camera calibration
        img_width, img_height = image_shape[1], image_shape[0]

        # Calculate relative position (simplified)
        rel_x = (center_x - img_width / 2) / (img_width / 2)  # -1 to 1
        rel_y = (center_y - img_height / 2) / (img_height / 2)  # -1 to 1

        # Estimate distance based on bounding box size (simplified)
        bbox_size = bbox[2] * bbox[3]  # width * height
        # Larger bounding box means closer person (simplified)
        estimated_distance = max(0.5, min(3.0, 2.0 / (bbox_size / (img_width * img_height))))

        return (rel_x, rel_y, estimated_distance)

    def generate_following_commands(self):
        """Generate velocity commands to follow person"""
        if not self.person_location:
            return

        # Calculate errors
        x_error = self.person_location[0]  # Horizontal position error
        y_error = self.person_location[1]  # Vertical position error
        distance_error = self.person_location[2] - self.target_distance  # Distance error

        # PID control for horizontal position (angular velocity)
        self.integral_x += x_error
        derivative_x = x_error - self.prev_error_x

        angular_velocity = (
            self.kp * x_error +
            self.ki * self.integral_x +
            self.kd * derivative_x
        )

        # PID control for distance (linear velocity)
        self.integral_y += distance_error
        derivative_y = distance_error - self.prev_error_y

        linear_velocity = (
            self.kp * distance_error +
            self.ki * self.integral_y +
            self.kd * derivative_y
        )

        # Limit velocities
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        linear_velocity = max(-self.max_linear_speed, min(self.max_linear_speed, linear_velocity))

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity

        self.velocity_cmd_pub.publish(cmd)

        # Update previous errors
        self.prev_error_x = x_error
        self.prev_error_y = distance_error

    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.velocity_cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down person tracking node...')
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 3: Multi-Modal Instruction Following

This example demonstrates a VLA system that can follow complex instructions combining visual and language inputs.

### Multi-Modal Instruction Parser

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection2DArray
import json
import re

class MultiModalInstructionParserNode(Node):
    def __init__(self):
        super().__init__('multi_modal_instruction_parser_node')

        # Publishers
        self.action_plan_pub = self.create_publisher(String, 'action_plan', 10)
        self.instruction_status_pub = self.create_publisher(String, 'instruction_status', 10)

        # Subscribers
        self.natural_language_sub = self.create_subscription(
            String, 'natural_language_command', self.natural_language_callback, 10
        )
        self.visual_context_sub = self.create_subscription(
            Detection2DArray, 'object_detections', self.visual_context_callback, 10
        )

        # Known objects and locations
        self.known_objects = [
            'bottle', 'cup', 'chair', 'table', 'book', 'phone', 'keys'
        ]
        self.known_locations = [
            'kitchen', 'living room', 'bedroom', 'office', 'hallway'
        ]

        # Instruction patterns
        self.instruction_patterns = {
            'bring': [
                r'bring me the (?P<object>\w+)',
                r'get me the (?P<object>\w+)',
                r'fetch the (?P<object>\w+)'
            ],
            'go_to': [
                r'go to the (?P<location>\w+)',
                r'go to (?P<location>\w+)',
                r'move to the (?P<location>\w+)'
            ],
            'follow': [
                r'follow the (?P<person>\w+)',
                r'follow (?P<person>\w+)'
            ],
            'describe': [
                r'describe what you see',
                r'what do you see',
                r'tell me about the room'
            ]
        }

        self.get_logger().info('Multi-Modal Instruction Parser Node initialized')

    def natural_language_callback(self, msg):
        """Process natural language instruction"""
        instruction = msg.data.lower()
        self.get_logger().info(f'Processing natural language instruction: {instruction}')

        # Parse instruction
        parsed_instruction = self.parse_instruction(instruction)

        if parsed_instruction:
            # Publish action plan
            plan_msg = String()
            plan_msg.data = json.dumps(parsed_instruction)
            self.action_plan_pub.publish(plan_msg)

            # Update status
            status_msg = String()
            status_msg.data = f"instruction_parsed:{parsed_instruction['action']}"
            self.instruction_status_pub.publish(status_msg)

            self.get_logger().info(f'Parsed instruction: {parsed_instruction}')
        else:
            self.get_logger().warn(f'Could not parse instruction: {instruction}')

    def visual_context_callback(self, msg):
        """Process visual context for instruction clarification"""
        # This could be used to disambiguate instructions based on visual context
        # For example, if user says "the red cup" but there are multiple cups
        pass

    def parse_instruction(self, instruction):
        """Parse natural language instruction into executable action"""
        # Check each pattern type
        for action_type, patterns in self.instruction_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, instruction, re.IGNORECASE)
                if match:
                    # Extract parameters
                    params = match.groupdict()

                    # Validate parameters against known objects/locations
                    if action_type == 'bring' and 'object' in params:
                        obj = params['object']
                        if obj in self.known_objects:
                            return {
                                'action': 'fetch_object',
                                'parameters': {'object': obj},
                                'confidence': 0.9
                            }
                    elif action_type == 'go_to' and 'location' in params:
                        loc = params['location']
                        if loc in self.known_locations:
                            return {
                                'action': 'navigate_to_location',
                                'parameters': {'location': loc},
                                'confidence': 0.9
                            }
                    elif action_type == 'follow' and 'person' in params:
                        return {
                            'action': 'follow_person',
                            'parameters': {'person': params['person']},
                            'confidence': 0.8
                        }
                    elif action_type == 'describe':
                        return {
                            'action': 'describe_environment',
                            'parameters': {},
                            'confidence': 0.95
                        }

        # If no pattern matches, try to extract general action
        return self.extract_general_action(instruction)

    def extract_general_action(self, instruction):
        """Extract general action when specific patterns don't match"""
        # Simple keyword-based action extraction
        if any(word in instruction for word in ['move', 'go', 'walk', 'navigate']):
            return {
                'action': 'move',
                'parameters': {'instruction': instruction},
                'confidence': 0.6
            }
        elif any(word in instruction for word in ['grasp', 'pick', 'take', 'grab']):
            return {
                'action': 'grasp',
                'parameters': {'instruction': instruction},
                'confidence': 0.6
            }
        elif any(word in instruction for word in ['speak', 'say', 'tell']):
            return {
                'action': 'speak',
                'parameters': {'instruction': instruction},
                'confidence': 0.7
            }
        else:
            return {
                'action': 'unknown',
                'parameters': {'instruction': instruction},
                'confidence': 0.3
            }

def main(args=None):
    rclpy.init(args=args)
    node = MultiModalInstructionParserNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down multi-modal instruction parser node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action Execution System

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class ActionExecutionSystemNode(Node):
    def __init__(self):
        super().__init__('action_execution_system_node')

        # Publishers
        self.execution_status_pub = self.create_publisher(String, 'execution_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.action_plan_sub = self.create_subscription(
            String, 'action_plan', self.action_plan_callback, 10
        )

        # Navigation action client
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Execution state
        self.current_action = None
        self.is_executing = False

        self.get_logger().info('Action Execution System Node initialized')

    def action_plan_callback(self, msg):
        """Handle action plan execution"""
        action_plan = json.loads(msg.data)
        self.get_logger().info(f'Received action plan: {action_plan}')

        if not self.is_executing:
            self.execute_action_plan(action_plan)
        else:
            self.get_logger().warn('Action already executing, queuing new action')

    def execute_action_plan(self, action_plan):
        """Execute the action plan"""
        action_type = action_plan['action']
        parameters = action_plan['parameters']
        confidence = action_plan.get('confidence', 0.5)

        # Check confidence threshold
        if confidence < 0.5:
            self.get_logger().warn(f'Action confidence too low: {confidence}')
            return

        self.is_executing = True
        self.current_action = action_type

        self.get_logger().info(f'Executing action: {action_type} with parameters: {parameters}')

        # Execute based on action type
        if action_type == 'fetch_object':
            success = self.execute_fetch_object(parameters)
        elif action_type == 'navigate_to_location':
            success = self.execute_navigate_to_location(parameters)
        elif action_type == 'follow_person':
            success = self.execute_follow_person(parameters)
        elif action_type == 'describe_environment':
            success = self.execute_describe_environment(parameters)
        elif action_type == 'move':
            success = self.execute_move(parameters)
        elif action_type == 'grasp':
            success = self.execute_grasp(parameters)
        elif action_type == 'speak':
            success = self.execute_speak(parameters)
        else:
            success = False
            self.get_logger().warn(f'Unknown action type: {action_type}')

        # Update execution status
        status_msg = String()
        status_msg.data = f"action_{'completed' if success else 'failed'}:{action_type}"
        self.execution_status_pub.publish(status_msg)

        self.is_executing = False
        self.current_action = None

    def execute_fetch_object(self, parameters):
        """Execute fetch object action"""
        obj = parameters.get('object', 'unknown')
        self.get_logger().info(f'Fetching object: {obj}')

        # In practice, this would involve:
        # 1. Navigate to object location
        # 2. Detect and localize object
        # 3. Grasp the object
        # 4. Return to user

        # Simulate the process
        self.get_logger().info('Navigating to object...')
        time.sleep(2.0)

        self.get_logger().info('Detecting object...')
        time.sleep(1.0)

        self.get_logger().info('Grasping object...')
        time.sleep(1.0)

        self.get_logger().info('Returning to user...')
        time.sleep(2.0)

        return True

    def execute_navigate_to_location(self, parameters):
        """Execute navigate to location action"""
        location = parameters.get('location', 'unknown')
        self.get_logger().info(f'Navigating to location: {location}')

        # In practice, use Nav2 navigation
        # For simulation, just move forward briefly
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.0

        start_time = time.time()
        while time.time() - start_time < 3.0:
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)

        # Stop
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)

        return True

    def execute_follow_person(self, parameters):
        """Execute follow person action"""
        person = parameters.get('person', 'unknown')
        self.get_logger().info(f'Following person: {person}')

        # In practice, this would activate person following system
        # For simulation, just log the action
        time.sleep(5.0)  # Simulate following

        return True

    def execute_describe_environment(self, parameters):
        """Execute describe environment action"""
        self.get_logger().info('Describing environment...')

        # In practice, this would use perception system to describe surroundings
        # For simulation, return a mock description
        time.sleep(2.0)

        return True

    def execute_move(self, parameters):
        """Execute general move action"""
        instruction = parameters.get('instruction', '')
        self.get_logger().info(f'Moving based on instruction: {instruction}')

        # Parse movement from instruction
        if 'forward' in instruction:
            cmd = Twist()
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
        elif 'backward' in instruction:
            cmd = Twist()
            cmd.linear.x = -0.3
            cmd.angular.z = 0.0
        elif 'left' in instruction:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3
        elif 'right' in instruction:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = -0.3
        else:
            cmd = Twist()
            cmd.linear.x = 0.2  # Default forward movement
            cmd.angular.z = 0.0

        # Execute movement for 2 seconds
        start_time = time.time()
        while time.time() - start_time < 2.0:
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)

        # Stop
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

        return True

    def execute_grasp(self, parameters):
        """Execute grasp action"""
        instruction = parameters.get('instruction', '')
        self.get_logger().info(f'Grasping based on instruction: {instruction}')

        # In practice, control gripper/arm to grasp
        # For simulation, just wait
        time.sleep(2.0)

        return True

    def execute_speak(self, parameters):
        """Execute speak action"""
        instruction = parameters.get('instruction', '')
        self.get_logger().info(f'Speaking: {instruction}')

        # In practice, use text-to-speech
        # For simulation, just log the action
        time.sleep(1.0)

        return True

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutionSystemNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down action execution system node...')
        # Stop robot
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        node.cmd_vel_pub.publish(cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete VLA System Integration

### Main VLA System Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import threading
import time

class VLASystemNode(Node):
    def __init__(self):
        super().__init__('vla_system_node')

        # Publishers
        self.system_status_pub = self.create_publisher(String, 'vla_system_status', 10)

        # Subscribers for system coordination
        self.user_command_sub = self.create_subscription(
            String, 'user_command', self.user_command_callback, 10
        )
        self.execution_status_sub = self.create_subscription(
            String, 'execution_status', self.execution_status_callback, 10
        )
        self.system_component_status_sub = self.create_subscription(
            String, 'component_status', self.component_status_callback, 10
        )

        # System state
        self.system_active = True
        self.current_task = None
        self.system_components = {
            'language_understanding': False,
            'vision_processing': False,
            'action_planning': False,
            'execution_system': False,
            'navigation': False
        }

        # Timer for system health monitoring
        self.health_timer = self.create_timer(1.0, self.system_health_check)

        # Initialize system components
        self.initialize_system()

        self.get_logger().info('VLA System Node initialized')

    def initialize_system(self):
        """Initialize VLA system components"""
        self.get_logger().info('Initializing VLA system components...')

        # In practice, each component would be a separate ROS node
        # This is a simplified representation
        self.system_components['language_understanding'] = True
        self.system_components['vision_processing'] = True
        self.system_components['action_planning'] = True
        self.system_components['execution_system'] = True
        self.system_components['navigation'] = True

    def user_command_callback(self, msg):
        """Handle user command through VLA system"""
        command = msg.data
        self.get_logger().info(f'Received user command: {command}')

        # In practice, this would route to appropriate VLA components
        # For now, just log the command
        status_msg = String()
        status_msg.data = f"command_received:{command}"
        self.system_status_pub.publish(status_msg)

    def execution_status_callback(self, msg):
        """Handle execution status updates"""
        status = msg.data
        self.get_logger().info(f'Execution status: {status}')

    def component_status_callback(self, msg):
        """Handle component status updates"""
        status = msg.data
        # Update internal component status tracking
        pass

    def system_health_check(self):
        """Check health of VLA system"""
        all_active = all(self.system_components.values())

        status_msg = String()
        status_msg.data = f"system_healthy:{all_active}:components:{self.system_components}"
        self.system_status_pub.publish(status_msg)

        if not all_active:
            self.get_logger().warn('Some VLA system components are not active')

def main(args=None):
    rclpy.init(args=args)
    node = VLASystemNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down VLA system node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## System Integration and Launch

### Complete VLA System Launch File

```xml
<!-- vla_system.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        # Fetch task language understanding node
        Node(
            package='your_robot_package',
            executable='fetch_task_language_understanding_node',
            name='fetch_task_language_understanding',
            output='screen'
        ),

        # Fetch task object detection node
        Node(
            package='your_robot_package',
            executable='fetch_task_object_detection_node',
            name='fetch_task_object_detection',
            output='screen'
        ),

        # Fetch task navigation and grasping node
        Node(
            package='your_robot_package',
            executable='fetch_task_navigation_grasping_node',
            name='fetch_task_navigation_grasping',
            output='screen'
        ),

        # Person following language node
        Node(
            package='your_robot_package',
            executable='person_following_language_node',
            name='person_following_language',
            output='screen'
        ),

        # Person tracking node
        Node(
            package='your_robot_package',
            executable='person_tracking_node',
            name='person_tracking',
            output='screen'
        ),

        # Multi-modal instruction parser node
        Node(
            package='your_robot_package',
            executable='multi_modal_instruction_parser_node',
            name='multi_modal_instruction_parser',
            output='screen'
        ),

        # Action execution system node
        Node(
            package='your_robot_package',
            executable='action_execution_system_node',
            name='action_execution_system',
            output='screen'
        ),

        # Main VLA system node
        Node(
            package='your_robot_package',
            executable='vla_system_node',
            name='vla_system',
            output='screen'
        )
    ])
```

## Testing VLA Systems

### Unit Tests for VLA Components

```python
#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestVLAComponents(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_vla_components_node')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_fetch_task_language_understanding(self):
        """Test fetch task language understanding"""
        self.assertTrue(True)  # Placeholder for actual test

    def test_person_following_integration(self):
        """Test person following integration"""
        self.assertTrue(True)  # Placeholder for actual test

    def test_multi_modal_instruction_parsing(self):
        """Test multi-modal instruction parsing"""
        self.assertTrue(True)  # Placeholder for actual test

    def test_action_execution_system(self):
        """Test action execution system"""
        self.assertTrue(True)  # Placeholder for actual test

if __name__ == '__main__':
    unittest.main()
```

## Performance Optimization

### Efficient VLA Pipeline Design

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import threading
import queue
import time
from typing import Dict, Any

class OptimizedVLAPipelineNode(Node):
    def __init__(self):
        super().__init__('optimized_vla_pipeline_node')

        # Publishers
        self.optimized_output_pub = self.create_publisher(String, 'optimized_vla_output', 10)

        # Subscribers
        self.input_sub = self.create_subscription(
            String, 'vla_input', self.input_callback, 10
        )

        # Processing queues
        self.processing_queue = queue.Queue(maxsize=10)
        self.result_queue = queue.Queue(maxsize=10)

        # Processing threads
        self.processing_threads = []
        for i in range(2):  # 2 processing threads
            thread = threading.Thread(target=self.processing_worker, args=(i,), daemon=True)
            thread.start()
            self.processing_threads.append(thread)

        # Result processing thread
        self.result_thread = threading.Thread(target=self.result_worker, daemon=True)
        self.result_thread.start()

        # Performance metrics
        self.processed_count = 0
        self.start_time = time.time()

        # Timer for performance monitoring
        self.perf_timer = self.create_timer(5.0, self.performance_report)

        self.get_logger().info('Optimized VLA Pipeline Node initialized')

    def input_callback(self, msg):
        """Handle input with optimized pipeline"""
        try:
            self.processing_queue.put(msg.data, block=False)
        except queue.Full:
            self.get_logger().warn('Processing queue is full, dropping input')

    def processing_worker(self, worker_id):
        """Worker thread for VLA processing"""
        while True:
            try:
                input_data = self.processing_queue.get(timeout=1.0)

                # Simulate VLA processing
                result = self.process_vla_input(input_data)

                # Add result to result queue
                self.result_queue.put({
                    'result': result,
                    'worker_id': worker_id,
                    'timestamp': time.time()
                })

                self.processed_count += 1

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Processing worker {worker_id} error: {e}')

    def result_worker(self):
        """Worker thread for handling results"""
        while True:
            try:
                result = self.result_queue.get(timeout=1.0)

                # Publish result
                output_msg = String()
                output_msg.data = result['result']
                self.optimized_output_pub.publish(output_msg)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Result worker error: {e}')

    def process_vla_input(self, input_data):
        """Process VLA input efficiently"""
        # In practice, this would call optimized VLA processing functions
        # This is a simplified example
        return f"Processed: {input_data}"

    def performance_report(self):
        """Report performance metrics"""
        elapsed_time = time.time() - self.start_time
        rate = self.processed_count / elapsed_time if elapsed_time > 0 else 0

        self.get_logger().info(
            f'VLA Pipeline Performance: {self.processed_count} processed, '
            f'{rate:.2f} Hz, Queue sizes: input={self.processing_queue.qsize()}, '
            f'result={self.result_queue.qsize()}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedVLAPipelineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down optimized VLA pipeline node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting VLA Systems

### 1. Vision Processing Issues
- **Cause**: Poor lighting or camera calibration
- **Solution**: Adjust camera settings and recalibrate
- **Check**: Verify image quality and detection accuracy

### 2. Language Understanding Problems
- **Cause**: Ambiguous or complex commands
- **Solution**: Implement clarification requests
- **Check**: Test with various command formulations

### 3. Action Execution Failures
- **Cause**: Physical constraints or sensor errors
- **Solution**: Implement robust error handling
- **Check**: Verify robot capabilities and sensor data

### 4. System Integration Issues
- **Cause**: Timing problems between components
- **Solution**: Implement proper synchronization
- **Check**: Monitor message timing and sequence

## Best Practices for VLA Implementation

### 1. Modularity
- Keep VLA components loosely coupled
- Use standardized interfaces between modules
- Design for easy replacement of individual components

### 2. Robustness
- Handle sensor failures gracefully
- Implement fallback behaviors
- Validate inputs and outputs

### 3. Performance
- Optimize processing pipelines for real-time operation
- Use efficient algorithms and data structures
- Monitor and optimize computational resources

### 4. Safety
- Implement safety checks at each level
- Ensure graceful degradation
- Monitor system health continuously

## Summary

Practical VLA examples demonstrate the integration of vision, language, and action systems in humanoid robotics. Key components include:

1. **Fetch and Carry Tasks**: Complete pipeline from language understanding to object manipulation
2. **Person Following**: Multi-modal tracking combining vision and voice commands
3. **Instruction Following**: Complex task execution based on natural language
4. **System Integration**: Coordinated operation of all VLA components
5. **Performance Optimization**: Efficient processing pipelines for real-time operation

These examples provide a foundation for building sophisticated VLA systems that enable humanoid robots to understand and respond to complex, multi-modal human commands in real-world environments.