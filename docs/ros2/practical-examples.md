---
title: Practical ROS 2 Examples for Humanoid Robotics
sidebar_position: 6
---

# Practical ROS 2 Examples for Humanoid Robotics

This section provides practical, real-world examples of ROS 2 implementations specifically tailored for humanoid robotics applications. Each example demonstrates how to combine multiple ROS 2 concepts to create functional robotic systems.

## Example 1: Humanoid Walking Controller

This example demonstrates a complete walking controller for a humanoid robot using ROS 2 communication patterns.

### Node Structure
- `walking_controller`: Main controller node
- `joint_state_publisher`: Publishes joint states for visualization
- `imu_processor`: Processes IMU data for balance feedback
- `foot_pressure_sensors`: Processes foot pressure for gait control

### Walking Controller Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')

        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        # Walking parameters
        self.step_height = 0.05  # meters
        self.step_length = 0.1   # meters
        self.step_duration = 1.0 # seconds
        self.current_phase = 0.0
        self.is_walking = False
        self.target_velocity = Twist()

        # Joint names for humanoid legs
        self.left_leg_joints = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint'
        ]
        self.right_leg_joints = [
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.target_velocity = msg
        self.is_walking = (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01)
        if not self.is_walking:
            self.current_phase = 0.0

    def control_loop(self):
        """Main control loop"""
        if not self.is_walking:
            # Publish zero trajectory to stop
            self.publish_zero_trajectory()
            return

        # Update walking phase
        self.current_phase += 0.05 / self.step_duration  # dt / duration
        if self.current_phase > 1.0:
            self.current_phase = 0.0

        # Generate walking trajectory
        trajectory_msg = self.generate_walking_trajectory()
        self.trajectory_pub.publish(trajectory_msg)

    def generate_walking_trajectory(self):
        """Generate trajectory for current walking phase"""
        msg = JointTrajectory()
        msg.joint_names = self.left_leg_joints + self.right_leg_joints

        point = JointTrajectoryPoint()

        # Calculate joint angles based on walking phase and target velocity
        left_hip, left_knee, left_ankle = self.calculate_leg_angles(
            self.current_phase,
            self.target_velocity.linear.x,
            is_left=True
        )

        right_hip, right_knee, right_ankle = self.calculate_leg_angles(
            (self.current_phase + 0.5) % 1.0,  # Phase offset for alternating legs
            self.target_velocity.linear.x,
            is_left=False
        )

        # Apply angular velocity based on turning
        if self.target_velocity.angular.z > 0:
            # Turning right - adjust right leg more
            right_hip *= 0.8
        elif self.target_velocity.angular.z < 0:
            # Turning left - adjust left leg more
            left_hip *= 0.8

        point.positions = [
            left_hip, left_knee, left_ankle,
            right_hip, right_knee, right_ankle
        ]

        # Set timing
        point.time_from_start = Duration(sec=0, nanosec=50000000)  # 50ms

        msg.points = [point]
        return msg

    def calculate_leg_angles(self, phase, forward_velocity, is_left):
        """Calculate joint angles for a leg based on walking phase"""
        # Simplified inverse kinematics for walking
        # In real implementation, use proper IK solver

        # Forward velocity affects step size
        scale = min(abs(forward_velocity) * 2.0, 1.0)  # Max 100% scaling

        # Calculate hip angle (forward/back)
        hip_angle = math.sin(phase * 2 * math.pi) * 0.3 * scale

        # Calculate knee angle (bend for step)
        knee_angle = math.sin(phase * 2 * math.pi + math.pi) * 0.4 * scale

        # Calculate ankle angle (compensate for ground contact)
        ankle_angle = -hip_angle * 0.5  # Simple compensation

        return hip_angle, knee_angle, ankle_angle

    def publish_zero_trajectory(self):
        """Publish zero trajectory to stop walking"""
        msg = JointTrajectory()
        msg.joint_names = self.left_leg_joints + self.right_leg_joints

        point = JointTrajectoryPoint()
        point.positions = [0.0] * 6  # 3 joints per leg
        point.time_from_start = Duration(sec=0, nanosec=10000000)  # 10ms

        msg.points = [point]
        self.trajectory_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = WalkingController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down walking controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 2: Humanoid Arm Controller with Grasping

This example demonstrates coordinated arm control with grasping capabilities.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes
import math

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Publishers
        self.left_arm_pub = self.create_publisher(
            JointTrajectory,
            '/left_arm_controller/joint_trajectory',
            10
        )
        self.right_arm_pub = self.create_publisher(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            10
        )
        self.gripper_pub = self.create_publisher(String, 'gripper_control', 10)

        # Subscribers
        self.arm_command_sub = self.create_subscription(
            Pose,
            'arm_command',
            self.arm_command_callback,
            10
        )

        # Services
        self.ik_service = self.create_client(GetPositionIK, '/compute_ik')

        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # State variables
        self.target_pose = None
        self.current_joint_state = None
        self.arm_mode = "idle"  # idle, moving, grasping

        # Arm joint names
        self.left_arm_joints = [
            'left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint'
        ]
        self.right_arm_joints = [
            'right_shoulder_joint', 'right_elbow_joint', 'right_wrist_joint'
        ]

    def arm_command_callback(self, msg):
        """Handle arm command with target pose"""
        self.target_pose = msg
        self.arm_mode = "moving"

        # Request inverse kinematics
        if self.ik_service.service_is_ready():
            self.request_ik_solution()
        else:
            self.get_logger().warn('IK service not available')

    def request_ik_solution(self):
        """Request inverse kinematics solution"""
        request = GetPositionIK.Request()
        request.ik_request.group_name = "left_arm"  # or right_arm
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.pose = self.target_pose
        request.ik_request.timeout = Duration(sec=1, nanosec=0)

        future = self.ik_service.call_async(request)
        future.add_done_callback(self.ik_response_callback)

    def ik_response_callback(self, future):
        """Handle IK solution response"""
        try:
            response = future.result()
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                # Send joint trajectory to achieve target pose
                self.execute_trajectory(response.solution.joint_state)
            else:
                self.get_logger().error(f'IK failed: {response.error_code.val}')
                self.arm_mode = "idle"
        except Exception as e:
            self.get_logger().error(f'IK service call failed: {e}')
            self.arm_mode = "idle"

    def execute_trajectory(self, joint_state):
        """Execute joint trajectory to reach target"""
        msg = JointTrajectory()
        msg.joint_names = joint_state.name

        point = JointTrajectoryPoint()
        point.positions = joint_state.position
        point.time_from_start = Duration(sec=2, nanosec=0)  # 2 seconds

        msg.points = [point]

        # Determine which arm based on joint names
        if 'left' in msg.joint_names[0]:
            self.left_arm_pub.publish(msg)
        else:
            self.right_arm_pub.publish(msg)

    def control_loop(self):
        """Main control loop"""
        if self.arm_mode == "moving" and self.target_pose:
            # Check if arm is close to target
            if self.is_at_target():
                self.arm_mode = "idle"
                self.get_logger().info('Arm reached target')

    def is_at_target(self):
        """Check if arm is at target pose (simplified)"""
        # In real implementation, compare actual vs target pose
        return False  # Simplified for example

    def grasp_object(self, left_arm=True):
        """Command gripper to grasp"""
        msg = String()
        msg.data = "close" if left_arm else "right_close"
        self.gripper_pub.publish(msg)

    def release_object(self, left_arm=True):
        """Command gripper to release"""
        msg = String()
        msg.data = "open" if left_arm else "right_open"
        self.gripper_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = ArmController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down arm controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 3: Multi-Sensor Fusion Node

This example demonstrates how to fuse data from multiple sensors for better perception:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Publishers
        self.fused_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'fused_pose',
            10
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Timer for fusion
        self.fusion_timer = self.create_timer(0.05, self.fusion_loop)

        # State variables
        self.imu_data = {'orientation': [0, 0, 0, 1], 'angular_velocity': [0, 0, 0], 'linear_acceleration': [0, 0, 0]}
        self.odom_data = {'pose': [0, 0, 0], 'twist': [0, 0, 0]}  # [x, y, theta] and [linear, angular]
        self.scan_data = None

        # Covariance tracking
        self.pose_covariance = np.eye(6) * 0.1  # Initial uncertainty
        self.bridge = CvBridge()

        self.get_logger().info('Sensor fusion node initialized')

    def imu_callback(self, msg):
        """Handle IMU data"""
        self.imu_data['orientation'] = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        self.imu_data['angular_velocity'] = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        self.imu_data['linear_acceleration'] = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]

    def odom_callback(self, msg):
        """Handle odometry data"""
        # Extract position and orientation from odometry
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation

        self.odom_data['pose'] = [pos.x, pos.y, self.quaternion_to_yaw(quat)]
        self.odom_data['twist'] = [
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        ]

    def scan_callback(self, msg):
        """Handle laser scan data"""
        self.scan_data = {
            'ranges': msg.ranges,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def fusion_loop(self):
        """Main fusion algorithm"""
        # Kalman filter approach to fuse sensor data
        # This is a simplified version - real implementation would be more complex

        # Predict state based on odometry
        predicted_pose = self.predict_from_odometry()

        # Update with IMU data
        fused_pose = self.update_with_imu(predicted_pose)

        # If scan data available, use it to correct position
        if self.scan_data:
            fused_pose = self.update_with_scan(fused_pose)

        # Publish fused pose
        self.publish_fused_pose(fused_pose)

    def predict_from_odometry(self):
        """Predict pose from odometry"""
        dt = 0.05  # timer period

        # Simple motion model
        x, y, theta = self.odom_data['pose']
        linear_vel, angular_vel = self.odom_data['twist']

        new_x = x + linear_vel * dt * math.cos(theta)
        new_y = y + linear_vel * dt * math.sin(theta)
        new_theta = theta + angular_vel * dt

        return [new_x, new_y, new_theta]

    def update_with_imu(self, predicted_pose):
        """Update pose estimate with IMU data"""
        # Use IMU to correct orientation
        quat = self.imu_data['orientation']
        imu_yaw = self.quaternion_to_yaw(
            Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        )

        # Blend predicted and IMU orientation
        fused_pose = predicted_pose.copy()
        fused_pose[2] = 0.7 * predicted_pose[2] + 0.3 * imu_yaw  # 70% prediction, 30% IMU

        return fused_pose

    def update_with_scan(self, pose):
        """Update pose estimate with laser scan data"""
        # This would typically involve scan matching or landmark detection
        # For this example, we'll just return the pose
        return pose

    def publish_fused_pose(self, pose):
        """Publish the fused pose estimate"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Set pose
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]

        # Convert yaw to quaternion
        quat = self.yaw_to_quaternion(pose[2])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        # Set covariance
        msg.pose.covariance = self.pose_covariance.flatten().tolist()

        self.fused_pose_pub.publish(msg)

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [0, 0, sy, cy]

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        fusion_node.get_logger().info('Shutting down sensor fusion node...')
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 4: Behavior-Based Navigation

This example demonstrates a behavior-based navigation system for humanoid robots:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from std_msgs.msg import String
import math

class BehaviorBasedNavigator(Node):
    def __init__(self):
        super().__init__('behavior_based_navigator')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.behavior_pub = self.create_publisher(String, 'current_behavior', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            'move_base_simple/goal',
            self.goal_callback,
            10
        )

        # Timers
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)

        # State variables
        self.scan_data = None
        self.current_goal = None
        self.current_behavior = "idle"
        self.robot_pose = [0, 0, 0]  # x, y, theta

        # Navigation parameters
        self.safety_distance = 0.5
        self.approach_distance = 1.0
        self.rotation_speed = 0.5
        self.forward_speed = 0.3

        self.get_logger().info('Behavior-based navigator initialized')

    def scan_callback(self, msg):
        """Handle laser scan data"""
        self.scan_data = {
            'ranges': msg.ranges,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }

    def goal_callback(self, msg):
        """Handle navigation goal"""
        self.current_goal = [msg.pose.position.x, msg.pose.position.y]
        self.current_behavior = "navigate"
        self.get_logger().info(f'New goal: {self.current_goal}')

    def navigation_loop(self):
        """Main navigation loop"""
        if not self.scan_data or not self.current_goal:
            self.stop_robot()
            return

        # Determine current behavior based on situation
        distance_to_goal = self.calculate_distance_to_goal()

        # Check for obstacles
        min_distance = min(self.scan_data['ranges']) if self.scan_data['ranges'] else float('inf')

        if min_distance < self.safety_distance:
            # Emergency stop if too close to obstacle
            self.current_behavior = "avoid_emergency"
        elif distance_to_goal < 0.2:
            # Goal reached
            self.current_behavior = "goal_reached"
            self.current_goal = None
        elif min_distance < 1.5:  # Near obstacle
            self.current_behavior = "avoid"
        else:
            # Navigate toward goal
            self.current_behavior = "navigate"

        # Execute current behavior
        cmd_vel = self.execute_behavior()
        self.cmd_vel_pub.publish(cmd_vel)

        # Publish current behavior
        behavior_msg = String()
        behavior_msg.data = self.current_behavior
        self.behavior_pub.publish(behavior_msg)

    def execute_behavior(self):
        """Execute current navigation behavior"""
        cmd_vel = Twist()

        if self.current_behavior == "idle":
            # Robot stays still
            pass

        elif self.current_behavior == "navigate":
            # Navigate toward goal
            angle_to_goal = self.calculate_angle_to_goal()
            cmd_vel.linear.x = self.forward_speed
            cmd_vel.angular.z = self.rotation_speed * angle_to_goal

        elif self.current_behavior == "avoid":
            # Avoid obstacles using sensor data
            cmd_vel = self.avoid_obstacles()

        elif self.current_behavior == "avoid_emergency":
            # Emergency stop and turn
            cmd_vel.angular.z = self.rotation_speed  # Turn to clear obstacle

        elif self.current_behavior == "goal_reached":
            # Stop at goal
            self.stop_robot()

        return cmd_vel

    def avoid_obstacles(self):
        """Implement obstacle avoidance behavior"""
        cmd_vel = Twist()

        # Find the direction with the most clearance
        ranges = self.scan_data['ranges']
        angle_min = self.scan_data['angle_min']
        angle_increment = self.scan_data['angle_increment']

        # Look at forward 90-degree sector
        center_idx = len(ranges) // 2
        sector_size = len(ranges) // 4  # 90 degrees

        left_clearance = sum(ranges[center_idx - sector_size//2:center_idx])
        right_clearance = sum(ranges[center_idx:center_idx + sector_size//2])

        if left_clearance > right_clearance:
            cmd_vel.angular.z = self.rotation_speed * 0.5  # Turn left
        else:
            cmd_vel.angular.z = -self.rotation_speed * 0.5  # Turn right

        # Reduce forward speed when avoiding
        cmd_vel.linear.x = self.forward_speed * 0.3

        return cmd_vel

    def calculate_distance_to_goal(self):
        """Calculate distance to current goal"""
        if not self.current_goal:
            return float('inf')

        dx = self.current_goal[0] - self.robot_pose[0]
        dy = self.current_goal[1] - self.robot_pose[1]
        return math.sqrt(dx*dx + dy*dy)

    def calculate_angle_to_goal(self):
        """Calculate angle to current goal"""
        if not self.current_goal:
            return 0.0

        dx = self.current_goal[0] - self.robot_pose[0]
        dy = self.current_goal[1] - self.robot_pose[1]

        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.robot_pose[2]

        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        return angle_diff

    def stop_robot(self):
        """Stop robot movement"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    navigator = BehaviorBasedNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down navigator...')
    finally:
        navigator.stop_robot()
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Configuration

Create a launch file to run the complete humanoid system:

```python
# launch/humanoid_complete.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Walking controller node
    walking_controller = Node(
        package='humanoid_control',
        executable='walking_controller',
        name='walking_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'step_height': 0.05},
            {'step_length': 0.1}
        ],
        output='screen'
    )

    # Arm controller node
    arm_controller = Node(
        package='humanoid_control',
        executable='arm_controller',
        name='arm_controller',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Sensor fusion node
    sensor_fusion = Node(
        package='humanoid_perception',
        executable='sensor_fusion',
        name='sensor_fusion',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Navigation node
    navigator = Node(
        package='humanoid_navigation',
        executable='behavior_navigator',
        name='behavior_navigator',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'safety_distance': 0.5}
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),

        walking_controller,
        arm_controller,
        sensor_fusion,
        navigator
    ])
```

## Testing and Validation

### Unit Testing Example

```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TestWalkingController(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = WalkingController()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_trajectory_generation(self):
        """Test trajectory generation functionality"""
        # Set test conditions
        self.node.target_velocity.linear.x = 0.5
        self.node.is_walking = True

        # Generate trajectory
        trajectory = self.node.generate_walking_trajectory()

        # Check that trajectory has expected structure
        self.assertEqual(len(trajectory.joint_names), 6)  # 3 joints per leg
        self.assertEqual(len(trajectory.points), 1)
        self.assertEqual(len(trajectory.points[0].positions), 6)

    def test_stop_functionality(self):
        """Test that robot stops when not walking"""
        self.node.is_walking = False
        self.node.current_phase = 0.5

        # Call control loop
        self.node.control_loop()

        # Check that zero trajectory was generated
        # (would need to add mocking of publisher to fully test)

if __name__ == '__main__':
    unittest.main()
```

## Best Practices Summary

1. **Modular Design**: Separate functionality into distinct nodes
2. **Error Handling**: Implement robust error handling and recovery
3. **Parameter Configuration**: Use ROS 2 parameters for configuration
4. **Logging**: Use appropriate logging levels for debugging
5. **Safety**: Implement safety checks and emergency stops
6. **Performance**: Optimize for real-time performance requirements
7. **Testing**: Include unit tests and integration tests
8. **Documentation**: Document interfaces and expected behavior

These practical examples demonstrate how to implement complex humanoid robot behaviors using ROS 2. Each example combines multiple ROS 2 concepts to create functional systems that can be extended and customized for specific humanoid robot platforms.