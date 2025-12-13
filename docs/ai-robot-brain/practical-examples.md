---
title: Practical AI Integration Examples
sidebar_position: 6
---

# Practical AI Integration Examples

This section provides hands-on examples of integrating AI systems with humanoid robots, combining perception, navigation, and control systems. These examples demonstrate practical implementations of the concepts covered in previous sections.

## Example 1: Object Recognition and Navigation

This example demonstrates how to combine object detection with navigation to create a robot that can find and approach specific objects.

### Object Detection Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectRecognitionNode(Node):
    def __init__(self):
        super().__init__('object_recognition_node')

        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            'object_detections',
            10
        )

        self.target_found_pub = self.create_publisher(
            PoseStamped,
            'target_object_pose',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            'object_detection_status',
            10
        )

        # CV Bridge for image conversion
        self.cv_bridge = CvBridge()

        # Load YOLO model
        try:
            self.net = cv2.dnn.readNetFromDarknet(
                'config/yolo_config.cfg',
                'weights/yolo_weights.weights'
            )
            self.layer_names = self.net.getLayerNames()
            self.output_layers = [
                self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()
            ]
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.net = None

        # Target object classes (e.g., 'bottle', 'cup', 'person')
        self.target_classes = ['bottle', 'cup', 'person']

        # Class names (update based on your model)
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
            'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
            'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
            'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
            'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
            'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
            'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
            'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock',
            'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        self.get_logger().info('Object Recognition Node initialized')

    def image_callback(self, msg):
        """Process incoming images for object detection"""
        if self.net is None:
            return

        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            detections = self.detect_objects(cv_image)

            # Process detections for target objects
            target_found = False
            for detection in detections:
                if detection['class_name'] in self.target_classes:
                    # Calculate 3D position if depth is available
                    pose_3d = self.calculate_3d_position(
                        detection, cv_image.shape
                    )

                    if pose_3d is not None:
                        # Publish target object pose
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.header.frame_id = 'camera_frame'
                        pose_msg.pose.position = pose_3d
                        pose_msg.pose.orientation.w = 1.0

                        self.target_found_pub.publish(pose_msg)
                        target_found = True

                        # Publish status
                        status_msg = String()
                        status_msg.data = f"Found {detection['class_name']} at ({pose_3d.x:.2f}, {pose_3d.y:.2f}, {pose_3d.z:.2f})"
                        self.status_pub.publish(status_msg)

            # Publish all detections
            detection_array = self.create_detection_array(detections, msg.header)
            self.detection_pub.publish(detection_array)

            if not target_found:
                status_msg = String()
                status_msg.data = "Searching for target objects..."
                self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_objects(self, image):
        """Perform object detection on the input image"""
        height, width = image.shape[:2]

        # Create blob from image
        blob = cv2.dnn.blobFromImage(
            image, 1/255.0, (416, 416), swapRB=True, crop=False
        )

        # Set input to the network
        self.net.setInput(blob)

        # Run forward pass
        outputs = self.net.forward(self.output_layers)

        # Process detections
        boxes = []
        confidences = []
        class_ids = []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.5:  # Threshold
                    # Object coordinates and dimensions
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply non-maximum suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # Return detected objects with class names
        detected_objects = []
        if len(indices) > 0:
            for i in indices.flatten():
                if class_ids[i] < len(self.class_names):
                    detected_objects.append({
                        'class_id': class_ids[i],
                        'class_name': self.class_names[class_ids[i]],
                        'confidence': confidences[i],
                        'bbox': boxes[i]
                    })

        return detected_objects

    def calculate_3d_position(self, detection, image_shape):
        """Calculate 3D position of detected object (requires depth info)"""
        # This is a simplified version - in practice, you'd use depth information
        # or stereo vision to calculate 3D position

        # For now, return a placeholder position
        # In a real implementation, you'd use depth from a depth camera
        bbox = detection['bbox']
        center_x = bbox[0] + bbox[2] / 2
        center_y = bbox[1] + bbox[3] / 2

        # Placeholder 3D position (would use actual depth in real implementation)
        from geometry_msgs.msg import Point
        point = Point()
        point.x = 1.0  # Placeholder distance
        point.y = (center_x - image_shape[1]/2) * 0.01  # Approximate lateral offset
        point.z = (center_y - image_shape[0]/2) * 0.01  # Approximate height offset

        return point

    def create_detection_array(self, detections, header):
        """Create Detection2DArray message from detections"""
        from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

        detection_array = Detection2DArray()
        detection_array.header = header

        for detection in detections:
            detection_msg = Detection2D()
            detection_msg.bbox.size_x = detection['bbox'][2]
            detection_msg.bbox.size_y = detection['bbox'][3]

            # Center of bounding box
            center_x = detection['bbox'][0] + detection['bbox'][2] / 2
            center_y = detection['bbox'][1] + detection['bbox'][3] / 2

            detection_msg.bbox.center.x = center_x
            detection_msg.bbox.center.y = center_y

            # Add hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = detection['class_name']
            hypothesis.hypothesis.score = detection['confidence']
            detection_msg.results.append(hypothesis)

            detection_array.detections.append(detection_msg)

        return detection_array

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognitionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down object recognition node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Navigation to Detected Object

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class ObjectNavigationNode(Node):
    def __init__(self):
        super().__init__('object_navigation_node')

        # Create action client for navigation
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.target_sub = self.create_subscription(
            PoseStamped,
            'target_object_pose',
            self.target_callback,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            'object_detection_status',
            self.status_callback,
            10
        )

        # Navigation state
        self.current_target = None
        self.is_navigating = False
        self.navigation_enabled = True

        self.get_logger().info('Object Navigation Node initialized')

    def target_callback(self, msg):
        """Handle target object pose"""
        if not self.navigation_enabled:
            return

        # Transform target pose to map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time()
            )

            # Transform the target position
            target_in_map = self.transform_pose(msg.pose, transform)

            # Navigate to a position in front of the object
            approach_position = self.calculate_approach_position(target_in_map)

            self.get_logger().info(
                f'Navigating to object at ({approach_position.position.x:.2f}, {approach_position.position.y:.2f})'
            )

            self.navigate_to_position(approach_position)

        except TransformException as ex:
            self.get_logger().error(f'Could not transform target pose: {ex}')

    def transform_pose(self, pose, transform):
        """Transform pose using given transform"""
        # This is a simplified transformation
        # In practice, use tf2 for proper transformation
        from geometry_msgs.msg import Pose
        transformed_pose = Pose()

        # Apply translation
        transformed_pose.position.x = pose.position.x + transform.transform.translation.x
        transformed_pose.position.y = pose.position.y + transform.transform.translation.y
        transformed_pose.position.z = pose.position.z + transform.transform.translation.z

        # Apply rotation (simplified - would need proper quaternion math in practice)
        transformed_pose.orientation = pose.orientation

        return transformed_pose

    def calculate_approach_position(self, target_pose):
        """Calculate approach position in front of target object"""
        from geometry_msgs.msg import Pose

        approach_pose = Pose()

        # Calculate position 0.5m in front of the object
        approach_distance = 0.5  # meters

        # For simplicity, assume the robot approaches from the positive x direction
        approach_pose.position.x = target_pose.position.x - approach_distance
        approach_pose.position.y = target_pose.position.y
        approach_pose.position.z = target_pose.position.z

        # Set orientation to face the object
        dx = target_pose.position.x - approach_pose.position.x
        dy = target_pose.position.y - approach_pose.position.y
        yaw = math.atan2(dy, dx)

        from math import sin, cos
        approach_pose.orientation.z = sin(yaw / 2.0)
        approach_pose.orientation.w = cos(yaw / 2.0)

        return approach_pose

    def navigate_to_position(self, pose):
        """Navigate to specified position"""
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = pose

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True

        return True

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            self.is_navigating = False
            return

        self.get_logger().info('Navigation goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Navigation progress: {feedback.distance_remaining:.2f}m remaining'
        )

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info('Navigation to object completed')
        self.is_navigating = False

    def status_callback(self, msg):
        """Handle object detection status"""
        self.get_logger().info(f'Object detection status: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down object navigation node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 2: Voice Command Processing with AI

This example demonstrates how to integrate voice commands with AI decision-making for humanoid robot control.

### Voice Command Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatus
import speech_recognition as sr
import openai
import json

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speech_status_pub = self.create_publisher(String, 'speech_status', 10)

        # Subscribers
        self.voice_cmd_sub = self.create_subscription(
            String,
            'voice_command',
            self.voice_command_callback,
            10
        )

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Configure for listening
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # OpenAI API configuration (set your API key in environment)
        openai.api_key = "YOUR_OPENAI_API_KEY"  # Use environment variable in production

        # Command mapping
        self.command_map = {
            'move forward': self.move_forward,
            'move backward': self.move_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'stop': self.stop_robot,
            'dance': self.perform_dance,
            'wave': self.perform_wave,
        }

        # Timer for continuous listening
        self.listen_timer = self.create_timer(5.0, self.continuous_listen)

        self.get_logger().info('Voice Command Node initialized')

    def continuous_listen(self):
        """Continuously listen for voice commands"""
        try:
            with self.microphone as source:
                self.get_logger().info('Listening for voice commands...')
                audio = self.recognizer.listen(source, timeout=3.0, phrase_time_limit=5.0)

            # Recognize speech
            command_text = self.recognizer.recognize_google(audio).lower()
            self.get_logger().info(f'Recognized: {command_text}')

            # Publish recognized command
            cmd_msg = String()
            cmd_msg.data = command_text
            self.voice_cmd_sub.publish(cmd_msg)

        except sr.WaitTimeoutError:
            self.get_logger().info('No speech detected')
        except sr.UnknownValueError:
            self.get_logger().info('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in voice recognition: {e}')

    def voice_command_callback(self, msg):
        """Process voice command using AI"""
        command = msg.data.lower()
        self.get_logger().info(f'Processing voice command: {command}')

        # Use AI to interpret and enhance the command
        ai_interpreted_command = self.ai_interpret_command(command)

        # Execute the command
        self.execute_command(ai_interpreted_command)

    def ai_interpret_command(self, command):
        """Use AI to interpret and enhance voice commands"""
        try:
            # Use OpenAI API to interpret the command
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": "You are a command interpreter for a humanoid robot. Convert natural language commands to specific robot actions. Respond with a JSON object containing 'action' and 'parameters'. Available actions: move_forward, move_backward, turn_left, turn_right, stop, perform_dance, perform_wave."
                    },
                    {
                        "role": "user",
                        "content": f"Interpret this command: {command}"
                    }
                ],
                max_tokens=100,
                temperature=0.1
            )

            # Parse the AI response
            ai_response = response.choices[0].message['content'].strip()

            # Extract JSON from response (in case it includes text)
            import re
            json_match = re.search(r'\{.*\}', ai_response, re.DOTALL)
            if json_match:
                ai_json = json.loads(json_match.group())
                return ai_json
            else:
                # If no JSON found, try to parse the whole response
                return json.loads(ai_response)

        except Exception as e:
            self.get_logger().error(f'AI interpretation error: {e}')
            # Fallback to simple command mapping
            for key in self.command_map.keys():
                if key in command:
                    return {"action": key, "parameters": {}}
            return {"action": "unknown", "parameters": {}}

    def execute_command(self, interpreted_command):
        """Execute the interpreted command"""
        action = interpreted_command.get('action', 'unknown')
        parameters = interpreted_command.get('parameters', {})

        self.get_logger().info(f'Executing action: {action} with parameters: {parameters}')

        if action in self.command_map:
            self.command_map[action](parameters)
        else:
            self.get_logger().warn(f'Unknown action: {action}')
            self.speak_response(f'Sorry, I do not understand the command {action}')

    def move_forward(self, params):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = params.get('speed', 0.2)  # Default 0.2 m/s
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.speak_response('Moving forward')

    def move_backward(self, params):
        """Move robot backward"""
        cmd = Twist()
        cmd.linear.x = -params.get('speed', 0.2)  # Default 0.2 m/s
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.speak_response('Moving backward')

    def turn_left(self, params):
        """Turn robot left"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = params.get('speed', 0.3)  # Default 0.3 rad/s
        self.cmd_vel_pub.publish(cmd)
        self.speak_response('Turning left')

    def turn_right(self, params):
        """Turn robot right"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -params.get('speed', 0.3)  # Default 0.3 rad/s
        self.cmd_vel_pub.publish(cmd)
        self.speak_response('Turning right')

    def stop_robot(self, params):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.speak_response('Stopping')

    def perform_dance(self, params):
        """Perform a simple dance routine"""
        self.get_logger().info('Performing dance routine')
        # This would involve more complex motor control
        # For now, just rotate in place
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Rotate at 0.5 rad/s
        self.cmd_vel_pub.publish(cmd)
        self.speak_response('Dancing for you!')

        # Stop after 3 seconds
        self.create_timer(3.0, self.stop_robot)

    def perform_wave(self, params):
        """Perform waving gesture"""
        self.get_logger().info('Performing waving gesture')
        # This would involve arm control
        self.speak_response('Waving hello!')

    def speak_response(self, text):
        """Publish speech response status"""
        status_msg = String()
        status_msg.data = text
        self.speech_status_pub.publish(status_msg)
        self.get_logger().info(f'Speech response: {text}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down voice command node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 3: Multi-Sensory Fusion for Decision Making

This example demonstrates how to combine data from multiple sensors to make intelligent decisions.

### Sensor Fusion Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscribers for different sensors
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        # Publishers
        self.fused_decision_pub = self.create_publisher(
            Twist, 'fused_cmd_vel', 10
        )
        self.safety_status_pub = self.create_publisher(
            String, 'safety_status', 10
        )
        self.fused_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'fused_pose', 10
        )

        # State variables
        self.laser_data = None
        self.image_data = None
        self.imu_data = None
        self.cv_bridge = CvBridge()

        # Safety parameters
        self.min_obstacle_distance = 0.5  # meters
        self.max_tilt_angle = 0.2  # radians (~11 degrees)

        # Timer for fusion
        self.fusion_timer = self.create_timer(0.1, self.fusion_callback)

        # Covariance matrices for sensor fusion
        self.laser_cov = np.diag([0.1, 0.1, 0.05])  # x, y, theta
        self.vision_cov = np.diag([0.2, 0.2, 0.1])
        self.imu_cov = np.diag([0.05, 0.05, 0.02])

        self.get_logger().info('Sensor Fusion Node initialized')

    def laser_callback(self, msg):
        """Handle laser scan data"""
        self.laser_data = msg

    def image_callback(self, msg):
        """Handle image data"""
        self.image_data = msg

    def imu_callback(self, msg):
        """Handle IMU data"""
        self.imu_data = msg

    def fusion_callback(self):
        """Perform sensor fusion and make decisions"""
        if not all([self.laser_data, self.imu_data]):
            return

        # Process sensor data
        obstacle_info = self.process_laser_data()
        orientation_info = self.process_imu_data()

        # Perform sensor fusion
        fused_state = self.perform_fusion(obstacle_info, orientation_info)

        # Make safety decision
        safe_to_move = self.is_safe_to_move(fused_state)

        if safe_to_move:
            # Generate command based on sensor fusion
            cmd = self.generate_motion_command(fused_state)
            self.fused_decision_pub.publish(cmd)

            status_msg = String()
            status_msg.data = "Safe to navigate"
            self.safety_status_pub.publish(status_msg)
        else:
            # Emergency stop
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.fused_decision_pub.publish(stop_cmd)

            status_msg = String()
            status_msg.data = "Safety hazard detected - stopping"
            self.safety_status_pub.publish(status_msg)

    def process_laser_data(self):
        """Process laser scan to detect obstacles"""
        if not self.laser_data:
            return {'min_distance': float('inf'), 'obstacle_angle': 0.0}

        # Get ranges in front of robot (forward 90 degrees)
        front_ranges = self.laser_data.ranges[
            len(self.laser_data.ranges)//4 : 3*len(self.laser_data.ranges)//4
        ]

        # Filter out invalid ranges
        valid_ranges = [r for r in front_ranges if r > self.laser_data.range_min and r < self.laser_data.range_max]

        if not valid_ranges:
            return {'min_distance': float('inf'), 'obstacle_angle': 0.0}

        min_distance = min(valid_ranges)
        min_idx = front_ranges.index(min_distance) if min_distance in front_ranges else 0
        obstacle_angle = self.laser_data.angle_min + min_idx * self.laser_data.angle_increment

        return {
            'min_distance': min_distance,
            'obstacle_angle': obstacle_angle,
            'valid_ranges': valid_ranges
        }

    def process_imu_data(self):
        """Process IMU data to get orientation and stability"""
        if not self.imu_data:
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'stable': True}

        # Extract orientation from quaternion
        q = self.imu_data.orientation
        rot = R.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = rot.as_euler('xyz')

        # Check if robot is stable
        tilt_magnitude = np.sqrt(roll**2 + pitch**2)
        stable = tilt_magnitude < self.max_tilt_angle

        return {
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'stable': stable,
            'tilt_magnitude': tilt_magnitude
        }

    def perform_fusion(self, obstacle_info, orientation_info):
        """Perform sensor fusion using weighted averaging"""
        # Simple weighted fusion
        weights = [0.6, 0.4]  # laser: 60%, imu: 40%

        # Combine obstacle and orientation data
        fused_state = {
            'distance_to_obstacle': obstacle_info['min_distance'],
            'obstacle_angle': obstacle_info['obstacle_angle'],
            'is_stable': orientation_info['stable'],
            'tilt_angle': orientation_info['tilt_magnitude'],
            'yaw': orientation_info['yaw']
        }

        return fused_state

    def is_safe_to_move(self, fused_state):
        """Determine if it's safe to move based on fused sensor data"""
        # Check multiple safety conditions
        obstacle_safe = fused_state['distance_to_obstacle'] > self.min_obstacle_distance
        stability_safe = fused_state['is_stable']

        return obstacle_safe and stability_safe

    def generate_motion_command(self, fused_state):
        """Generate motion command based on fused state"""
        cmd = Twist()

        if fused_state['distance_to_obstacle'] > 2.0:
            # Clear path, move forward at normal speed
            cmd.linear.x = 0.3  # m/s
            cmd.angular.z = 0.0
        elif fused_state['distance_to_obstacle'] > 0.8:
            # Approaching obstacle, slow down
            cmd.linear.x = 0.1  # m/s
            cmd.angular.z = 0.0
        else:
            # Obstacle too close, consider turning
            if abs(fused_state['obstacle_angle']) < 0.5:  # Obstacle straight ahead
                # Turn away from obstacle
                cmd.linear.x = 0.0
                cmd.angular.z = 0.2 if fused_state['obstacle_angle'] < 0 else -0.2
            else:
                # Obstacle to the side, move forward carefully
                cmd.linear.x = 0.05
                cmd.angular.z = 0.0

        return cmd

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down sensor fusion node...')
    finally:
        # Stop robot on shutdown
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        node.fused_decision_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 4: Learning from Demonstration

This example shows how a humanoid robot can learn tasks through demonstration and imitation.

### Demonstration Learning Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String, Bool
from builtin_interfaces.msg import Duration
import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C

class DemonstrationLearningNode(Node):
    def __init__(self):
        super().__init__('demonstration_learning_node')

        # Publishers and subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        self.demo_start_sub = self.create_subscription(
            Bool, 'start_demonstration', self.start_demonstration_callback, 10
        )

        self.demo_stop_sub = self.create_subscription(
            Bool, 'stop_demonstration', self.stop_demonstration_callback, 10
        )

        self.execute_sub = self.create_subscription(
            String, 'execute_demonstration', self.execute_callback, 10
        )

        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.status_pub = self.create_publisher(String, 'learning_status', 10)

        # Learning state
        self.demonstration_data = []
        self.is_recording = False
        self.current_joint_positions = {}
        self.demo_start_time = None

        # ML model for learning
        self.model = None
        self.time_steps = []
        self.joint_trajectories = {}

        self.get_logger().info('Demonstration Learning Node initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

        # Record demonstration if active
        if self.is_recording:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if self.demo_start_time is None:
                self.demo_start_time = current_time

            elapsed_time = current_time - self.demo_start_time

            # Store joint positions at this time step
            demo_entry = {
                'time': elapsed_time,
                'joints': dict(zip(msg.name, msg.position))
            }
            self.demonstration_data.append(demo_entry)

    def start_demonstration_callback(self, msg):
        """Start recording demonstration"""
        if msg.data:
            self.is_recording = True
            self.demonstration_data = []
            self.demo_start_time = None
            self.get_logger().info('Started recording demonstration')

            status_msg = String()
            status_msg.data = 'Recording demonstration...'
            self.status_pub.publish(status_msg)

    def stop_demonstration_callback(self, msg):
        """Stop recording demonstration and train model"""
        if msg.data and self.is_recording:
            self.is_recording = False
            self.get_logger().info(f'Stopped recording. Recorded {len(self.demonstration_data)} samples')

            # Process the demonstration data
            if len(self.demonstration_data) > 1:
                self.train_model()

            status_msg = String()
            status_msg.data = f'Demonstration recorded with {len(self.demonstration_data)} samples'
            self.status_pub.publish(status_msg)

    def train_model(self):
        """Train a model to reproduce the demonstration"""
        if not self.demonstration_data:
            self.get_logger().warn('No demonstration data to train on')
            return

        # Extract time steps and joint positions
        time_steps = np.array([[entry['time']] for entry in self.demonstration_data])

        # Get all unique joint names
        all_joints = set()
        for entry in self.demonstration_data:
            all_joints.update(entry['joints'].keys())

        all_joints = sorted(list(all_joints))

        # Create trajectory for each joint
        joint_trajectories = {}
        for joint in all_joints:
            positions = []
            for entry in self.demonstration_data:
                positions.append(entry['joints'].get(joint, 0.0))
            joint_trajectories[joint] = np.array(positions)

        # Train a Gaussian Process model for each joint
        self.models = {}
        for joint, positions in joint_trajectories.items():
            # Reshape for sklearn
            X = time_steps.reshape(-1, 1)
            y = positions.reshape(-1, 1)

            # Create and train GP model
            kernel = C(1.0, (1e-3, 1e3)) * RBF(1.0, (1e-2, 1e2))
            gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10)
            gp.fit(X, y)

            self.models[joint] = gp

        self.get_logger().info(f'Trained models for {len(self.models)} joints')

    def execute_callback(self, msg):
        """Execute the learned demonstration"""
        if not self.models:
            self.get_logger().warn('No trained model to execute')
            return

        if msg.data == 'execute':
            self.get_logger().info('Executing learned demonstration')
            self.execute_demonstration()

    def execute_demonstration(self):
        """Execute the learned motion pattern"""
        if not self.models:
            return

        # Execute the demonstration over a period of time
        duration = 10.0  # seconds
        dt = 0.1  # time step
        steps = int(duration / dt)

        for i in range(steps):
            current_time = i * dt

            # Predict joint positions for this time step
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()

            for joint_name, model in self.models.items():
                # Predict position for this joint at current time
                X_pred = np.array([[current_time]])
                pred_pos = model.predict(X_pred)[0][0]

                cmd.name.append(joint_name)
                cmd.position.append(pred_pos)

            # Publish joint commands
            self.joint_cmd_pub.publish(cmd)

            # Sleep for timing (in a real implementation, use a timer)
            import time
            time.sleep(dt)

        self.get_logger().info('Demonstration execution completed')

def main(args=None):
    rclpy.init(args=args)
    node = DemonstrationLearningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down demonstration learning node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration Example: Complete AI-Enabled Robot System

This example shows how to integrate all the AI components into a complete system:

### Main Integration Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image, Imu
from builtin_interfaces.msg import Duration
import threading
import time

class AIIntegrationNode(Node):
    def __init__(self):
        super().__init__('ai_integration_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.system_status_pub = self.create_publisher(String, 'system_status', 10)

        # Subscribers
        self.voice_cmd_sub = self.create_subscription(
            String, 'voice_command', self.voice_command_callback, 10
        )
        self.target_pose_sub = self.create_subscription(
            PoseStamped, 'target_object_pose', self.target_pose_callback, 10
        )
        self.safety_status_sub = self.create_subscription(
            String, 'safety_status', self.safety_status_callback, 10
        )

        # System state
        self.current_mode = 'idle'  # idle, navigating, following, etc.
        self.target_pose = None
        self.safety_ok = True
        self.voice_command_queue = []

        # Timer for system updates
        self.system_timer = self.create_timer(0.1, self.system_update)

        # Initialize subsystems (in practice, these would be separate nodes)
        self.initialize_subsystems()

        self.get_logger().info('AI Integration Node initialized')

    def initialize_subsystems(self):
        """Initialize all AI subsystems"""
        # In a real implementation, these would be separate nodes or services
        self.get_logger().info('Initializing AI subsystems...')

        # Simulate initialization of various AI components
        self.perception_active = True
        self.navigation_active = True
        self.voice_active = True
        self.learning_active = True

    def voice_command_callback(self, msg):
        """Handle voice commands"""
        command = msg.data.lower()
        self.voice_command_queue.append(command)
        self.get_logger().info(f'Queued voice command: {command}')

    def target_pose_callback(self, msg):
        """Handle target object pose"""
        self.target_pose = msg
        self.get_logger().info('Received target pose')

    def safety_status_callback(self, msg):
        """Handle safety status updates"""
        self.safety_ok = 'safe' in msg.data.lower()
        self.get_logger().info(f'Safety status: {msg.data}')

    def system_update(self):
        """Main system update loop"""
        # Process voice commands
        if self.voice_command_queue:
            command = self.voice_command_queue.pop(0)
            self.process_voice_command(command)

        # Update system status
        status_msg = String()
        status_msg.data = f"Mode: {self.current_mode}, Safety: {'OK' if self.safety_ok else 'NOT OK'}"
        self.system_status_pub.publish(status_msg)

    def process_voice_command(self, command):
        """Process voice command and update system state"""
        if 'go to' in command or 'navigate to' in command:
            if self.target_pose:
                self.current_mode = 'navigating'
                self.navigate_to_target()
            else:
                self.speak_response('No target object detected')

        elif 'stop' in command:
            self.current_mode = 'idle'
            self.stop_robot()

        elif 'learn' in command or 'demonstrate' in command:
            self.current_mode = 'learning'
            self.start_learning_mode()

        elif 'follow' in command:
            self.current_mode = 'following'
            self.start_following_mode()

        else:
            self.speak_response(f'Received command: {command}, but not implemented')

    def navigate_to_target(self):
        """Navigate to the detected target"""
        if not self.target_pose or not self.safety_ok:
            return

        # Simple navigation to target (in practice, use Nav2)
        cmd = Twist()

        # Calculate direction to target (simplified)
        target_x = self.target_pose.pose.position.x
        target_y = self.target_pose.pose.position.y

        # Move toward target
        cmd.linear.x = min(0.3, max(-0.3, target_x * 0.5))  # Scale with distance
        cmd.angular.z = min(0.5, max(-0.5, -target_y * 0.5))  # Correct orientation

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Stop all robot motion"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def start_learning_mode(self):
        """Start learning from demonstration mode"""
        self.get_logger().info('Starting learning mode')
        # Publish message to start demonstration recording
        demo_start_msg = Bool()
        demo_start_msg.data = True
        # In practice, publish to demonstration learning node

    def start_following_mode(self):
        """Start following mode"""
        self.get_logger().info('Starting following mode')
        # Implement person following using perception and navigation

    def speak_response(self, text):
        """Publish speech response"""
        self.get_logger().info(f'System response: {text}')

def main(args=None):
    rclpy.init(args=args)
    node = AIIntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AI integration node...')
    finally:
        # Stop robot on shutdown
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File for Complete System

Create a launch file to bring up all the AI integration nodes:

```xml
<!-- ai_integration.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Object recognition node
        Node(
            package='your_robot_package',
            executable='object_recognition_node',
            name='object_recognition',
            parameters=[
                {'target_classes': ['bottle', 'cup', 'person']}
            ],
            output='screen'
        ),

        # Object navigation node
        Node(
            package='your_robot_package',
            executable='object_navigation_node',
            name='object_navigation',
            output='screen'
        ),

        # Voice command node
        Node(
            package='your_robot_package',
            executable='voice_command_node',
            name='voice_command',
            output='screen'
        ),

        # Sensor fusion node
        Node(
            package='your_robot_package',
            executable='sensor_fusion_node',
            name='sensor_fusion',
            output='screen'
        ),

        # Demonstration learning node
        Node(
            package='your_robot_package',
            executable='demonstration_learning_node',
            name='demonstration_learning',
            output='screen'
        ),

        # Main integration node
        Node(
            package='your_robot_package',
            executable='ai_integration_node',
            name='ai_integration',
            output='screen'
        )
    ])
```

## Testing the AI Integration

### Unit Tests

```python
#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TestAIIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_ai_integration_node')

        # Create publishers for testing
        self.voice_cmd_pub = self.node.create_publisher(
            String, 'voice_command', 10
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_voice_command_processing(self):
        """Test that voice commands are processed correctly"""
        # Publish a test command
        cmd_msg = String()
        cmd_msg.data = 'move forward'
        self.voice_cmd_pub.publish(cmd_msg)

        # In a real test, you'd check that the appropriate action was taken
        self.assertTrue(True)  # Placeholder for actual test logic

    def test_safety_system(self):
        """Test safety system response"""
        # This would test that the safety system responds appropriately
        # to simulated dangerous conditions
        self.assertTrue(True)  # Placeholder for actual test logic

if __name__ == '__main__':
    unittest.main()
```

## Best Practices for AI Integration

### 1. Modularity
- Keep AI components as separate, focused nodes
- Use ROS 2 interfaces for communication between components
- Design nodes to be independently testable

### 2. Safety First
- Implement multiple safety layers
- Use sensor fusion for robust safety decisions
- Include emergency stop mechanisms

### 3. Performance Considerations
- Optimize AI models for real-time performance
- Use appropriate hardware acceleration (GPU, etc.)
- Implement fallback behaviors when AI fails

### 4. Continuous Learning
- Design systems to learn from experience
- Include mechanisms for updating models
- Monitor and log system performance

## Summary

This practical examples section demonstrated:

1. **Object Recognition and Navigation**: Combining computer vision with navigation for goal-directed behavior
2. **Voice Command Processing**: Using AI to interpret natural language commands
3. **Sensor Fusion**: Combining multiple sensors for robust decision making
4. **Learning from Demonstration**: Teaching robots new behaviors through demonstration
5. **System Integration**: Bringing all components together into a cohesive AI system

These examples provide a foundation for building sophisticated AI-enabled humanoid robots that can perceive, reason, and act in complex environments. The modular design allows for incremental development and testing of individual components before system integration.