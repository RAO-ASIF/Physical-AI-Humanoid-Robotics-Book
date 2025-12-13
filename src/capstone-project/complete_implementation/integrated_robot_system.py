#!/usr/bin/env python3

"""
Integrated Humanoid Robot System - Capstone Project
This module integrates ROS 2, perception, navigation, and VLA (Vision-Language-Action) systems
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Path
import numpy as np
import threading
import time
from enum import Enum
from typing import Optional, Dict, Any


class RobotState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    NAVIGATING = "navigating"
    PERFORMING_ACTION = "performing_action"
    EMERGENCY_STOP = "emergency_stop"


class IntegratedRobotSystem(Node):
    def __init__(self):
        super().__init__('integrated_robot_system')

        # State management
        self.current_state = RobotState.IDLE
        self.target_pose = None
        self.detected_objects = []
        self.obstacle_detected = False
        self.voice_command = ""

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.voice_response_pub = self.create_publisher(String, 'voice_response', 10)

        # Subscribers
        self.voice_sub = self.create_subscription(
            String,
            'voice_commands',
            self.voice_command_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            'planned_path',
            self.path_callback,
            10
        )

        # Timers
        self.main_control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        self.state_monitor_timer = self.create_timer(0.5, self.monitor_state)  # 2Hz

        # Robot parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.safety_distance = 0.5  # meters
        self.navigation_threshold = 0.2  # meters to goal

        # Initialize state
        self.update_state(RobotState.IDLE)
        self.get_logger().info('Integrated Robot System Initialized')

    def update_state(self, new_state: RobotState):
        """Update the robot's state and log the transition"""
        old_state = self.current_state
        self.current_state = new_state
        self.get_logger().info(f'State transition: {old_state.value} -> {new_state.value}')

        # Publish status update
        status_msg = String()
        status_msg.data = f"State: {new_state.value}"
        self.status_pub.publish(status_msg)

    def voice_command_callback(self, msg):
        """Handle voice commands from the VLA system"""
        self.voice_command = msg.data.lower().strip()
        self.get_logger().info(f'Voice command received: {self.voice_command}')

        # Process command based on current state
        if self.current_state in [RobotState.IDLE, RobotState.LISTENING]:
            self.process_voice_command()

    def image_callback(self, msg):
        """Handle image data from perception system"""
        # In a real implementation, this would process the image
        # For this example, we'll just simulate object detection
        self.detected_objects = ["person", "chair"]  # Simulated detection
        self.get_logger().debug(f'Detected objects: {self.detected_objects}')

    def scan_callback(self, msg):
        """Handle laser scan data for navigation and obstacle detection"""
        # Check for obstacles in front of the robot
        front_scan = msg.ranges[len(msg.ranges)//2 - 50 : len(msg.ranges)//2 + 50]
        min_distance = min([r for r in front_scan if 0 < r < float('inf')], default=float('inf'))

        self.obstacle_detected = min_distance < self.safety_distance
        if self.obstacle_detected:
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m')

    def path_callback(self, msg):
        """Handle received navigation path"""
        if self.current_state == RobotState.NAVIGATING and msg.poses:
            self.get_logger().info('Received navigation path')
            # In a real implementation, this would follow the path
            # For this example, we'll just acknowledge receipt

    def process_voice_command(self):
        """Process the received voice command"""
        command = self.voice_command

        if "go to" in command or "navigate to" in command:
            # Extract target location from command (simplified)
            self.update_state(RobotState.NAVIGATING)
            self.execute_navigation_command(command)

        elif "find" in command or "look for" in command:
            # Object search command
            self.update_state(RobotState.PROCESSING)
            self.execute_search_command(command)

        elif "stop" in command or "halt" in command:
            self.stop_robot()
            self.update_state(RobotState.IDLE)

        elif "help" in command:
            self.provide_assistance()

        else:
            self.respond_to_command(f"I'm not sure how to handle: {command}")
            self.update_state(RobotState.IDLE)

    def execute_navigation_command(self, command: str):
        """Execute navigation command"""
        self.get_logger().info(f'Executing navigation command: {command}')

        # For this example, we'll navigate to a predefined location
        # In a real implementation, this would parse the command for specific locations
        self.navigate_to_target([1.0, 1.0])  # Example target coordinates

    def execute_search_command(self, command: str):
        """Execute search command"""
        self.get_logger().info(f'Executing search command: {command}')

        # For this example, we'll search for a specific object
        # In a real implementation, this would search for objects in the environment
        target_object = "person"
        self.search_for_object(target_object)

    def navigate_to_target(self, target_coordinates: list):
        """Navigate to target coordinates"""
        self.get_logger().info(f'Navigating to target: {target_coordinates}')

        # Simple navigation logic (in a real implementation, this would use path planning)
        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_speed
        cmd_vel.angular.z = 0.0  # Move straight for this example

        self.cmd_vel_pub.publish(cmd_vel)

    def search_for_object(self, target_object: str):
        """Search for a specific object in the environment"""
        self.get_logger().info(f'Searching for: {target_object}')

        # For this example, we'll simulate a search pattern
        # In a real implementation, this would control the robot to look around
        cmd_vel = Twist()
        cmd_vel.angular.z = self.angular_speed  # Turn slowly to search

        self.cmd_vel_pub.publish(cmd_vel)

    def provide_assistance(self):
        """Provide assistance based on current situation"""
        self.get_logger().info('Providing assistance')

        response = "I'm here to help. I can navigate to locations, find objects, or respond to commands."
        self.respond_to_command(response)

    def control_loop(self):
        """Main control loop for the robot"""
        if self.current_state == RobotState.EMERGENCY_STOP:
            self.emergency_stop()
        elif self.current_state == RobotState.NAVIGATING:
            self.navigation_control()
        elif self.current_state == RobotState.PROCESSING:
            self.processing_control()
        elif self.obstacle_detected and self.current_state != RobotState.IDLE:
            self.handle_obstacle()

    def navigation_control(self):
        """Control logic during navigation state"""
        # Check if we've reached the target (simplified)
        # In a real implementation, this would check distance to goal
        if self.target_pose:
            # Check if close enough to target
            # For this example, we'll just switch back to idle after a delay
            pass

    def processing_control(self):
        """Control logic during processing state"""
        # For this example, we'll just return to idle after processing
        # In a real implementation, this would continue processing
        pass

    def handle_obstacle(self):
        """Handle obstacle detection"""
        if self.current_state != RobotState.EMERGENCY_STOP:
            self.get_logger().warn('Obstacle detected, stopping robot')
            self.update_state(RobotState.EMERGENCY_STOP)
            self.emergency_stop()

    def emergency_stop(self):
        """Emergency stop procedure"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        time.sleep(0.1)  # Brief pause
        self.update_state(RobotState.IDLE)

    def stop_robot(self):
        """Stop robot movement"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

    def respond_to_command(self, response: str):
        """Respond to voice command with text output"""
        self.get_logger().info(f'Responding: {response}')

        # Publish response
        response_msg = String()
        response_msg.data = response
        self.voice_response_pub.publish(response_msg)

    def monitor_state(self):
        """Monitor robot state and safety conditions"""
        # Check for emergency conditions
        if self.obstacle_detected and self.current_state != RobotState.IDLE:
            if self.current_state != RobotState.EMERGENCY_STOP:
                self.update_state(RobotState.EMERGENCY_STOP)

        # Check if we've been in processing state too long
        if self.current_state == RobotState.PROCESSING:
            # Reset if no progress after timeout
            pass

        # Regular state checks
        if self.current_state not in [RobotState.NAVIGATING, RobotState.PROCESSING]:
            # Clear voice command after processing
            if self.voice_command:
                self.voice_command = ""


def main(args=None):
    """Main function to run the integrated robot system"""
    rclpy.init(args=args)

    # Create the integrated robot system node
    robot_system = IntegratedRobotSystem()

    try:
        # Keep the node running
        rclpy.spin(robot_system)
    except KeyboardInterrupt:
        robot_system.get_logger().info('Shutting down integrated robot system...')
        robot_system.stop_robot()
    finally:
        # Clean up
        robot_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()