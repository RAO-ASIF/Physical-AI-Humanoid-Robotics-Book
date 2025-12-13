#!/usr/bin/env python3
"""
Navigation Pipeline for Humanoid Robots
This module implements a complete navigation pipeline for humanoid robots,
including path planning, obstacle avoidance, and gait-aware locomotion.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Bool, Float64MultiArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
import numpy as np
from scipy import interpolate
import threading
import queue
from typing import List, Dict, Any, Optional
import time


class NavigationPipelineNode(Node):
    """
    Main navigation pipeline node that integrates path planning,
    obstacle avoidance, and locomotion control
    """
    def __init__(self):
        super().__init__('navigation_pipeline_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.local_plan_pub = self.create_publisher(Path, 'local_plan', 10)
        self.navigation_status_pub = self.create_publisher(String, 'navigation_status', 10)
        self.gait_cmd_pub = self.create_publisher(Float64MultiArray, 'gait_commands', 10)

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        # Navigation action client
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Navigation state
        self.current_goal = None
        self.current_pose = None
        self.is_navigating = False
        self.obstacle_detected = False
        self.robot_stuck = False

        # Navigation parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.min_distance_to_goal = 0.3  # meters
        self.safety_distance = 0.5  # meters for obstacle avoidance

        # Path planning components
        self.path_planner = PathPlanner()
        self.obstacle_avoider = ObstacleAvoider()
        self.gait_controller = GaitController()

        # Processing queue for navigation tasks
        self.navigation_queue = queue.Queue(maxsize=5)
        self.navigation_thread = threading.Thread(
            target=self.navigation_loop, daemon=True
        )
        self.navigation_thread.start()

        self.get_logger().info('Navigation Pipeline Node initialized')

    def goal_callback(self, msg):
        """Handle navigation goal requests"""
        self.current_goal = msg
        self.get_logger().info(
            f'Received navigation goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

        try:
            # Add navigation task to queue
            self.navigation_queue.put({
                'type': 'navigate_to_goal',
                'goal': msg,
                'timestamp': time.time()
            }, block=False)
        except queue.Full:
            self.get_logger().warn('Navigation queue is full, dropping goal')

    def scan_callback(self, msg):
        """Handle laser scan data for obstacle detection"""
        # Check for obstacles in front of robot
        front_ranges = msg.ranges[len(msg.ranges)//4 : 3*len(msg.ranges)//4]
        valid_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]

        if valid_ranges:
            min_distance = min(valid_ranges)
            self.obstacle_detected = min_distance < self.safety_distance

            if self.obstacle_detected:
                self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m')

    def odom_callback(self, msg):
        """Handle odometry data"""
        self.current_pose = msg.pose.pose

    def imu_callback(self, msg):
        """Handle IMU data for balance and orientation"""
        # Process IMU data for gait adjustments
        orientation = msg.orientation
        # Extract roll, pitch, yaw for balance control
        # This would be used by the gait controller

    def navigation_loop(self):
        """Main navigation processing loop"""
        while rclpy.ok():
            try:
                # Get navigation task from queue
                task = self.navigation_queue.get(timeout=0.1)

                if task['type'] == 'navigate_to_goal':
                    self.execute_navigation_task(task['goal'])

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Navigation loop error: {e}')

    def execute_navigation_task(self, goal):
        """Execute navigation task to reach goal"""
        if not self.current_pose:
            self.get_logger().warn('No current pose available')
            return

        self.is_navigating = True
        self.get_logger().info('Starting navigation to goal...')

        # Update navigation status
        status_msg = String()
        status_msg.data = 'navigation_started'
        self.navigation_status_pub.publish(status_msg)

        # Plan path to goal
        path = self.path_planner.plan_path(self.current_pose, goal.pose)
        if path:
            self.path_pub.publish(path)

            # Execute path following
            success = self.follow_path(path, goal.pose)

            if success:
                self.get_logger().info('Successfully reached goal')
                status_msg.data = 'goal_reached'
            else:
                self.get_logger().warn('Navigation failed')
                status_msg.data = 'navigation_failed'
        else:
            self.get_logger().warn('Failed to plan path to goal')
            status_msg.data = 'path_planning_failed'

        self.navigation_status_pub.publish(status_msg)
        self.is_navigating = False

    def follow_path(self, path, goal_pose):
        """Follow the planned path to the goal"""
        # For this example, we'll use a simple proportional controller
        # In practice, use more sophisticated path following algorithms

        reached_goal = False
        max_attempts = 1000  # Prevent infinite loops

        for attempt in range(max_attempts):
            if not self.is_navigating:
                return False

            if not self.current_pose:
                time.sleep(0.1)
                continue

            # Calculate distance to goal
            dx = goal_pose.position.x - self.current_pose.position.x
            dy = goal_pose.position.y - self.current_pose.position.y
            distance_to_goal = np.sqrt(dx*dx + dy*dy)

            if distance_to_goal < self.min_distance_to_goal:
                reached_goal = True
                break

            # Simple proportional control for navigation
            cmd = Twist()

            # Linear velocity proportional to distance (with saturation)
            cmd.linear.x = min(self.linear_speed, max(-self.linear_speed, 0.5 * distance_to_goal))

            # Angular velocity for heading correction
            current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
            desired_yaw = np.arctan2(dy, dx)
            yaw_error = self.normalize_angle(desired_yaw - current_yaw)

            cmd.angular.z = min(self.angular_speed, max(-self.angular_speed, 2.0 * yaw_error))

            # Apply obstacle avoidance if needed
            if self.obstacle_detected:
                cmd = self.obstacle_avoider.avoid_obstacles(cmd)

            # Publish command
            self.cmd_vel_pub.publish(cmd)

            # Publish gait commands for humanoid locomotion
            self.publish_gait_commands(cmd)

            time.sleep(0.1)  # 10 Hz control loop

        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.publish_gait_commands(stop_cmd)

        return reached_goal

    def get_yaw_from_quaternion(self, quat):
        """Extract yaw angle from quaternion"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def publish_gait_commands(self, cmd_vel):
        """Publish gait commands for humanoid locomotion"""
        if cmd_vel.linear.x != 0 or cmd_vel.angular.z != 0:
            gait_cmd = Float64MultiArray()

            # Simple gait command mapping
            # In practice, this would be more sophisticated
            gait_cmd.data = [
                cmd_vel.linear.x,      # Forward velocity
                cmd_vel.angular.z,     # Angular velocity
                0.0,                   # Step height
                0.5,                   # Step timing
                0.0                    # Balance adjustment
            ]

            self.gait_cmd_pub.publish(gait_cmd)


class PathPlanner:
    """
    Path planning module for global navigation
    """
    def __init__(self):
        # Initialize path planning parameters
        self.resolution = 0.1  # meters
        self.inflation_radius = 0.3  # meters

    def plan_path(self, start_pose, goal_pose):
        """
        Plan path from start to goal using a simple approach
        In practice, use A*, Dijkstra, or other path planning algorithms
        """
        # For this example, create a straight line path
        # In real implementation, use proper path planning algorithm
        path = Path()
        path.header.frame_id = 'map'

        # Calculate path points (simplified as straight line)
        start_x = start_pose.position.x
        start_y = start_pose.position.y
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y

        # Number of points in path
        distance = np.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
        num_points = max(2, int(distance / self.resolution))

        for i in range(num_points + 1):
            t = i / num_points if num_points > 0 else 0
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0

            # Set orientation towards goal
            yaw = np.arctan2(goal_y - start_y, goal_x - start_x)
            from math import sin, cos
            pose_stamped.pose.orientation.z = sin(yaw / 2.0)
            pose_stamped.pose.orientation.w = cos(yaw / 2.0)

            path.poses.append(pose_stamped)

        return path


class ObstacleAvoider:
    """
    Obstacle avoidance module for local navigation
    """
    def __init__(self):
        self.safety_distance = 0.5  # meters
        self.avoidance_strength = 1.0

    def avoid_obstacles(self, original_cmd):
        """
        Modify velocity command to avoid obstacles
        """
        # Create avoidance command
        avoidance_cmd = Twist()

        # For this example, simply stop the robot
        # In practice, implement proper obstacle avoidance behavior
        avoidance_cmd.linear.x = 0.0
        avoidance_cmd.angular.z = 0.2  # Gentle turn to avoid

        return avoidance_cmd


class GaitController:
    """
    Gait control module for humanoid locomotion
    """
    def __init__(self):
        self.step_frequency = 1.0  # steps per second
        self.step_length = 0.3  # meters
        self.step_height = 0.05  # meters

    def generate_gait_commands(self, desired_velocity):
        """
        Generate gait commands based on desired velocity
        """
        # This would generate proper walking patterns
        # For now, return placeholder values
        return {
            'step_length': self.step_length * (desired_velocity.linear.x / 0.3) if desired_velocity.linear.x != 0 else 0,
            'step_frequency': self.step_frequency,
            'step_height': self.step_height,
            'step_timing': 0.5
        }


def main(args=None):
    rclpy.init(args=args)
    node = NavigationPipelineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down navigation pipeline node...')
        # Stop robot
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()