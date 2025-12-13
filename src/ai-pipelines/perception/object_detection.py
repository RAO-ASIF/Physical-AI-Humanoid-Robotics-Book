#!/usr/bin/env python3

"""
Object Detection Pipeline for Humanoid Robots
This module implements object detection using deep learning models
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
from tensorflow import keras
import os


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Publishers
        self.detection_pub = self.create_publisher(String, 'object_detections', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Initialize detection model
        self.model = self.load_model()

        # Detection parameters
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4

        self.get_logger().info('Object Detection Node Initialized')

    def load_model(self):
        """
        Load the pre-trained object detection model
        For this example, we'll create a simple model structure
        In practice, you would load a pre-trained model like YOLO or SSD
        """
        try:
            # Check if a model file exists
            model_path = os.path.join(os.path.dirname(__file__), 'detection_model.h5')
            if os.path.exists(model_path):
                model = keras.models.load_model(model_path)
                self.get_logger().info('Pre-trained model loaded')
            else:
                # Create a dummy model for demonstration
                model = self.create_dummy_model()
                self.get_logger().info('Dummy model created for demonstration')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            model = self.create_dummy_model()

        return model

    def create_dummy_model(self):
        """
        Create a dummy model for demonstration purposes
        In a real implementation, you would use a pre-trained model
        """
        # This is just a placeholder - in reality you'd use a real model
        return None

    def image_callback(self, msg):
        """
        Callback function for processing incoming images
        Args:
            msg: The image message from the camera
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            detections = self.detect_objects(cv_image)

            # Publish detection results
            if detections:
                self.publish_detections(detections)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_objects(self, image):
        """
        Perform object detection on the input image
        Args:
            image: Input image in OpenCV format
        Returns:
            List of detections with bounding boxes and labels
        """
        # For this example, we'll use OpenCV's DNN module with a pre-trained model
        # In practice, you would use your specific model

        # This is a simplified example using OpenCV's DNN
        # For a real implementation, you'd load a pre-trained model like YOLO

        height, width = image.shape[:2]

        # Create dummy detections for demonstration
        # In a real implementation, you would run inference through your model
        dummy_detections = [
            {
                'label': 'person',
                'confidence': 0.85,
                'bbox': [int(width * 0.3), int(height * 0.3),
                         int(width * 0.4), int(height * 0.4)]
            },
            {
                'label': 'chair',
                'confidence': 0.72,
                'bbox': [int(width * 0.6), int(height * 0.5),
                         int(width * 0.7), int(height * 0.8)]
            }
        ]

        # Filter based on confidence threshold
        filtered_detections = [
            det for det in dummy_detections
            if det['confidence'] > self.confidence_threshold
        ]

        return filtered_detections

    def publish_detections(self, detections):
        """
        Publish detection results
        Args:
            detections: List of detection results
        """
        # Create a string message with detection information
        detection_str = ""
        for i, det in enumerate(detections):
            if i > 0:
                detection_str += "; "
            detection_str += f"{det['label']} ({det['confidence']:.2f}) at [{det['bbox'][0]}, {det['bbox'][1]}, {det['bbox'][2]}, {det['bbox'][3]}]"

        # Publish the detection string
        msg = String()
        msg.data = detection_str
        self.detection_pub.publish(msg)

        # Log the detections
        self.get_logger().info(f'Detected: {detection_str}')


def main(args=None):
    """Main function to run the object detection node"""
    rclpy.init(args=args)

    # Create the object detection node
    detection_node = ObjectDetectionNode()

    try:
        # Keep the node running
        rclpy.spin(detection_node)
    except KeyboardInterrupt:
        detection_node.get_logger().info('Shutting down object detection node...')
    finally:
        # Clean up
        detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()