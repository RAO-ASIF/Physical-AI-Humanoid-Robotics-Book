#!/usr/bin/env python3
"""
AI Model Integration Examples for Humanoid Robots
This module demonstrates various approaches to integrating AI models
with humanoid robot systems, including perception, navigation, and control models.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import torch
import torch.nn as nn
import torchvision.transforms as transforms
from typing import Dict, Any, Optional, List
import threading
import queue
import time
import os


class AIModelIntegrationNode(Node):
    """
    Main node for AI model integration examples
    """
    def __init__(self):
        super().__init__('ai_model_integration_node')

        # Publishers
        self.model_output_pub = self.create_publisher(String, 'model_output', 10)
        self.action_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.model_status_pub = self.create_publisher(String, 'model_status', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # Initialize components
        self.cv_bridge = CvBridge()
        self.model_manager = AIModelManager()
        self.perception_model = PerceptionModel()
        self.control_model = ControlModel()

        # Processing queues
        self.image_queue = queue.Queue(maxsize=5)
        self.scan_queue = queue.Queue(maxsize=5)

        # Processing threads
        self.image_thread = threading.Thread(
            target=self.process_image_loop, daemon=True
        )
        self.scan_thread = threading.Thread(
            target=self.process_scan_loop, daemon=True
        )
        self.image_thread.start()
        self.scan_thread.start()

        self.get_logger().info('AI Model Integration Node initialized')

    def image_callback(self, msg):
        """Handle incoming image messages"""
        try:
            self.image_queue.put(msg, block=False)
        except queue.Full:
            self.get_logger().warn('Image queue is full, dropping frame')

    def scan_callback(self, msg):
        """Handle laser scan messages"""
        try:
            self.scan_queue.put(msg, block=False)
        except queue.Full:
            self.get_logger().warn('Scan queue is full, dropping scan')

    def process_image_loop(self):
        """Process images with AI models"""
        while rclpy.ok():
            try:
                msg = self.image_queue.get(timeout=0.1)
                cv_image = self.cv_bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8'
                )

                # Run perception model
                results = self.perception_model.predict(cv_image)

                # Publish results
                result_msg = String()
                result_msg.data = str(results)
                self.model_output_pub.publish(result_msg)

                # Update status
                status_msg = String()
                status_msg.data = f"perception_model_processed:{len(results)}_objects"
                self.model_status_pub.publish(status_msg)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Image processing error: {e}')

    def process_scan_loop(self):
        """Process laser scans with AI models"""
        while rclpy.ok():
            try:
                msg = self.scan_queue.get(timeout=0.1)

                # Process with control model
                action = self.control_model.predict_from_scan(msg)

                # Publish action
                self.action_pub.publish(action)

                # Update status
                status_msg = String()
                status_msg.data = "control_model_processed:scan"
                self.model_status_pub.publish(status_msg)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Scan processing error: {e}')


class AIModelManager:
    """
    Manages loading, updating, and switching between different AI models
    """
    def __init__(self):
        self.models = {}
        self.model_paths = {}
        self.current_model = None

    def register_model(self, name: str, model: nn.Module, path: str = None):
        """Register a model with the manager"""
        self.models[name] = model
        if path:
            self.model_paths[name] = path

    def load_model(self, name: str, model_class, model_path: str = None):
        """Load a model from file"""
        if model_path is None and name in self.model_paths:
            model_path = self.model_paths[name]

        if model_path and os.path.exists(model_path):
            model = model_class()
            model.load_state_dict(torch.load(model_path))
            model.eval()
            self.register_model(name, model, model_path)
            return model
        else:
            self.get_logger().error(f'Model path does not exist: {model_path}')
            return None

    def switch_model(self, name: str):
        """Switch to a different model"""
        if name in self.models:
            self.current_model = name
            return True
        return False

    def get_current_model(self):
        """Get the currently active model"""
        if self.current_model and self.current_model in self.models:
            return self.models[self.current_model]
        return None


class PerceptionModel(nn.Module):
    """
    Example perception model for object detection and scene understanding
    """
    def __init__(self, num_classes=80):
        super(PerceptionModel, self).__init__()

        # Simple CNN for demonstration
        self.features = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((7, 7))
        )

        self.classifier = nn.Sequential(
            nn.Linear(128 * 7 * 7, 512),
            nn.ReLU(inplace=True),
            nn.Dropout(0.5),
            nn.Linear(512, num_classes)
        )

    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), -1)
        x = self.classifier(x)
        return x

    def predict(self, image):
        """
        Run prediction on an input image
        """
        # Preprocess image
        transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225])
        ])

        # Convert image to tensor
        if isinstance(image, np.ndarray):
            if len(image.shape) == 3:
                image = np.transpose(image, (2, 0, 1))  # HWC to CHW
            image_tensor = torch.from_numpy(image).float() / 255.0
        else:
            image_tensor = image

        # Add batch dimension
        image_tensor = image_tensor.unsqueeze(0)

        # Run inference
        with torch.no_grad():
            output = self(image_tensor)
            probabilities = torch.softmax(output, dim=1)
            predicted_class = torch.argmax(probabilities, dim=1)

        # For demonstration, return mock detection results
        # In practice, this would return bounding boxes, class labels, etc.
        return [{
            'class_id': predicted_class.item(),
            'confidence': probabilities[0][predicted_class].item(),
            'bbox': [10, 10, 100, 100],  # x, y, width, height
            'class_name': f'object_{predicted_class.item()}'
        }]


class ControlModel(nn.Module):
    """
    Example control model for generating robot actions from sensor data
    """
    def __init__(self, scan_size=360, action_dim=2):
        super(ControlModel, self).__init__()

        # Simple network for processing laser scan to generate velocity commands
        self.scan_processor = nn.Sequential(
            nn.Linear(scan_size, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 32),
            nn.ReLU(inplace=True)
        )

        # Output layer for velocity commands (linear, angular)
        self.velocity_output = nn.Linear(32, action_dim)

    def forward(self, x):
        x = self.scan_processor(x)
        velocities = self.velocity_output(x)
        return velocities

    def predict_from_scan(self, scan_msg):
        """
        Generate action from laser scan data
        """
        # Process scan ranges
        ranges = np.array(scan_msg.ranges)
        # Replace invalid ranges with max range
        ranges = np.nan_to_num(ranges, nan=scan_msg.range_max, posinf=scan_msg.range_max, neginf=0.0)

        # Normalize ranges
        ranges = ranges / scan_msg.range_max

        # Convert to tensor
        scan_tensor = torch.from_numpy(ranges).float().unsqueeze(0)

        # Run inference
        with torch.no_grad():
            velocities = self(scan_tensor)

        # Create Twist message
        cmd = Twist()
        cmd.linear.x = float(velocities[0, 0])  # Linear velocity
        cmd.angular.z = float(velocities[0, 1])  # Angular velocity

        return cmd


class ReinforcementLearningModel(nn.Module):
    """
    Example reinforcement learning model for robot control
    """
    def __init__(self, state_dim=24, action_dim=4):
        super(ReinforcementLearningModel, self).__init__()

        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim),
            nn.Tanh()  # Actions between -1 and 1
        )

        self.critic = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)  # Value estimate
        )

    def forward(self, state):
        action = self.actor(state)
        value = self.critic(state)
        return action, value

    def get_action(self, state):
        """
        Get action from the policy network
        """
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        action, _ = self(state_tensor)
        return action.detach().cpu().numpy()[0]


class ImitationLearningModel(nn.Module):
    """
    Example imitation learning model for learning from demonstrations
    """
    def __init__(self, observation_dim=24, action_dim=4):
        super(ImitationLearningModel, self).__init__()

        self.network = nn.Sequential(
            nn.Linear(observation_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim)
        )

    def forward(self, x):
        return self.network(x)

    def predict_action(self, observation):
        """
        Predict action based on observation
        """
        obs_tensor = torch.FloatTensor(observation).unsqueeze(0)
        action = self(obs_tensor)
        return action.detach().cpu().numpy()[0]


class ModelTrainingManager:
    """
    Manages training of AI models with robot data
    """
    def __init__(self):
        self.training_data = []
        self.validation_data = []

    def add_training_sample(self, observation, action):
        """Add a training sample to the dataset"""
        self.training_data.append((observation, action))

    def train_model(self, model: nn.Module, epochs=100, learning_rate=1e-3):
        """Train a model with collected data"""
        if not self.training_data:
            return

        optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
        criterion = nn.MSELoss()

        for epoch in range(epochs):
            total_loss = 0.0

            for obs, act in self.training_data:
                obs_tensor = torch.FloatTensor(obs).unsqueeze(0)
                act_tensor = torch.FloatTensor(act).unsqueeze(0)

                optimizer.zero_grad()
                predicted_act = model(obs_tensor)
                loss = criterion(predicted_act, act_tensor)
                loss.backward()
                optimizer.step()

                total_loss += loss.item()

            avg_loss = total_loss / len(self.training_data)
            print(f'Epoch {epoch+1}/{epochs}, Loss: {avg_loss:.6f}')

    def validate_model(self, model: nn.Module):
        """Validate model on validation data"""
        if not self.validation_data:
            return 0.0

        criterion = nn.MSELoss()
        total_loss = 0.0

        with torch.no_grad():
            for obs, act in self.validation_data:
                obs_tensor = torch.FloatTensor(obs).unsqueeze(0)
                act_tensor = torch.FloatTensor(act).unsqueeze(0)

                predicted_act = model(obs_tensor)
                loss = criterion(predicted_act, act_tensor)
                total_loss += loss.item()

        avg_loss = total_loss / len(self.validation_data)
        return avg_loss


class ModelDeploymentManager:
    """
    Manages deployment of trained models to robot
    """
    def __init__(self):
        self.deployed_models = {}
        self.model_performance = {}

    def deploy_model(self, name: str, model: nn.Module, device='cpu'):
        """Deploy a model to the robot"""
        model.to(device)
        model.eval()  # Set to evaluation mode
        self.deployed_models[name] = {
            'model': model,
            'device': device,
            'timestamp': time.time()
        }

    def update_model_performance(self, model_name: str, accuracy: float, latency: float):
        """Update performance metrics for a deployed model"""
        if model_name not in self.model_performance:
            self.model_performance[model_name] = []

        self.model_performance[model_name].append({
            'accuracy': accuracy,
            'latency': latency,
            'timestamp': time.time()
        })

    def get_model_performance(self, model_name: str) -> Dict[str, Any]:
        """Get performance metrics for a model"""
        if model_name in self.model_performance:
            metrics = self.model_performance[model_name]
            if metrics:
                accuracies = [m['accuracy'] for m in metrics]
                latencies = [m['latency'] for m in metrics]
                return {
                    'avg_accuracy': sum(accuracies) / len(accuracies),
                    'avg_latency': sum(latencies) / len(latencies),
                    'num_evaluations': len(metrics)
                }
        return {'avg_accuracy': 0.0, 'avg_latency': 0.0, 'num_evaluations': 0}


def main(args=None):
    rclpy.init(args=args)
    node = AIModelIntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AI model integration node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()