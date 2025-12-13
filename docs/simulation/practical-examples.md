---
title: Practical Simulation Examples
sidebar_position: 6
---

# Practical Simulation Examples

This section provides hands-on examples of humanoid robot simulation scenarios. These examples demonstrate the practical application of simulation concepts in real-world robotics problems.

## Example 1: Humanoid Walking in Gazebo

This example demonstrates simulating humanoid walking behavior in Gazebo with proper physics and control.

### Robot Model Setup

First, let's create a simplified humanoid model for simulation:

```xml
<!-- simple_humanoid.urdf -->
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Body -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.027" ixy="0.0" ixz="0.0" iyy="0.027" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.05 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.0008"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="80" velocity="1.0"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="brown">
        <color rgba="0.6 0.4 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0004" ixy="0.0" ixz="0.0" iyy="0.0006" iyz="0.0" izz="0.0007"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="2.0"/>
  </joint>

  <!-- Right Leg (similar to left) -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.027" ixy="0.0" ixz="0.0" iyy="0.027" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.05 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.0008"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="80" velocity="1.0"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="brown">
        <color rgba="0.6 0.4 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0004" ixy="0.0" ixz="0.0" iyy="0.0006" iyz="0.0" izz="0.0007"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="2.0"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="ros_control" filename="libgazebo_ros2_control.so">
      <robot_namespace>/simple_humanoid</robot_namespace>
    </plugin>
  </gazebo>
</robot>
```

### Walking Controller Implementation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')

        # Publishers for joint trajectory controller
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/simple_humanoid/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for control loop (50Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)

        # Walking parameters
        self.walking = False
        self.target_velocity = Twist()
        self.step_phase = 0.0
        self.step_height = 0.05
        self.step_length = 0.1
        self.step_duration = 2.0  # seconds per step

        # Joint names for the humanoid legs
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

        self.get_logger().info('Walking controller initialized')

    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.target_velocity = msg
        self.walking = (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01)

    def control_loop(self):
        """Main control loop"""
        if not self.walking:
            # Publish zero trajectory to stop
            self.publish_zero_trajectory()
            return

        # Update walking phase
        self.step_phase += 0.02 / self.step_duration  # dt / duration
        if self.step_phase > 1.0:
            self.step_phase = 0.0

        # Generate walking trajectory
        trajectory_msg = self.generate_walking_trajectory()
        self.trajectory_pub.publish(trajectory_msg)

    def generate_walking_trajectory(self):
        """Generate joint trajectory for walking"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        # Calculate joint angles based on walking phase
        left_angles = self.calculate_leg_angles(
            self.step_phase,
            self.target_velocity.linear.x,
            is_left=True
        )
        right_angles = self.calculate_leg_angles(
            (self.step_phase + 0.5) % 1.0,  # Phase offset for alternating legs
            self.target_velocity.linear.x,
            is_left=False
        )

        # Apply turning adjustments
        if self.target_velocity.angular.z > 0:
            # Turning right - adjust right leg more
            right_angles[0] *= 0.8  # hip
        elif self.target_velocity.angular.z < 0:
            # Turning left - adjust left leg more
            left_angles[0] *= 0.8  # hip

        point.positions = left_angles + right_angles
        point.time_from_start = Duration(sec=0, nanosec=20000000)  # 20ms

        msg.points = [point]
        return msg

    def calculate_leg_angles(self, phase, forward_vel, is_left):
        """Calculate joint angles for a leg based on walking phase"""
        # Simplified inverse kinematics for walking
        # Scale with forward velocity
        scale = min(abs(forward_vel) * 2.0, 1.0)

        # Hip angle (forward/back motion)
        hip_angle = math.sin(phase * 2 * math.pi) * 0.3 * scale

        # Knee angle (bend for step motion)
        knee_angle = math.sin(phase * 2 * math.pi + math.pi) * 0.4 * scale

        # Ankle angle (compensation)
        ankle_angle = -hip_angle * 0.5

        return [hip_angle, knee_angle, ankle_angle]

    def publish_zero_trajectory(self):
        """Publish zero trajectory to stop walking"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [0.0] * 6  # 6 joints (3 per leg)
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

## Example 2: Object Manipulation Simulation

This example demonstrates simulating object manipulation with a humanoid robot arm.

### Robot Arm Model with Gripper

```xml
<!-- simple_arm.urdf -->
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.017" ixy="0.0" ixz="0.0" iyy="0.017" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Shoulder -->
  <link name="shoulder_link">
    <visual>
      <geometry>
        <cylinder length="0.15" radius="0.06"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.15" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <!-- Upper Arm -->
  <link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.006" ixy="0.0" ixz="0.0" iyy="0.006" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="80" velocity="1.0"/>
  </joint>

  <!-- Lower Arm -->
  <link name="lower_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.0003"/>
    </inertial>
  </link>

  <joint name="wrist_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="lower_arm_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <!-- Gripper Base -->
  <link name="gripper_base">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
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

  <joint name="gripper_joint" type="fixed">
    <parent link="lower_arm_link"/>
    <child link="gripper_base"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
  </joint>

  <!-- Left Gripper Finger -->
  <link name="left_finger">
    <visual>
      <geometry>
        <box size="0.08 0.01 0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.01 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.02"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
    </inertial>
  </link>

  <joint name="left_finger_joint" type="prismatic">
    <parent link="gripper_base"/>
    <child link="left_finger"/>
    <origin xyz="0.025 0.03 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="0.03" effort="20" velocity="0.1"/>
  </joint>

  <!-- Right Gripper Finger -->
  <link name="right_finger">
    <visual>
      <geometry>
        <box size="0.08 0.01 0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.01 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.02"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
    </inertial>
  </link>

  <joint name="right_finger_joint" type="prismatic">
    <parent link="gripper_base"/>
    <child link="right_finger"/>
    <origin xyz="0.025 -0.03 0" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="0" upper="0.03" effort="20" velocity="0.1"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="ros_control" filename="libgazebo_ros2_control.so">
      <robot_namespace>/simple_arm</robot_namespace>
    </plugin>
  </gazebo>
</robot>
```

### Manipulation Controller

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class ManipulationController(Node):
    def __init__(self):
        super().__init__('manipulation_controller')

        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/simple_arm/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'manipulation_goal',
            self.goal_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_publisher(
            JointTrajectory,
            '/simple_arm/gripper_controller/joint_trajectory',
            10
        )

        # State variables
        self.current_joint_states = None
        self.target_pose = None
        self.manipulation_mode = "idle"  # idle, reaching, grasping, releasing

        # Joint names
        self.arm_joint_names = [
            'shoulder_joint', 'elbow_joint', 'wrist_joint'
        ]
        self.gripper_joint_names = [
            'left_finger_joint', 'right_finger_joint'
        ]

        self.get_logger().info('Manipulation controller initialized')

    def goal_callback(self, msg):
        """Handle manipulation goals"""
        self.target_pose = msg.pose
        self.manipulation_mode = "reaching"
        self.execute_manipulation_task()

    def execute_manipulation_task(self):
        """Execute manipulation based on current mode"""
        if self.manipulation_mode == "reaching":
            self.move_to_target()
        elif self.manipulation_mode == "grasping":
            self.close_gripper()
        elif self.manipulation_mode == "releasing":
            self.open_gripper()

    def move_to_target(self):
        """Move arm to target position"""
        if not self.target_pose:
            return

        # Simple inverse kinematics for reaching
        # In a real implementation, this would use a proper IK solver
        shoulder_angle = self.calculate_shoulder_angle(self.target_pose)
        elbow_angle = self.calculate_elbow_angle(self.target_pose)
        wrist_angle = self.calculate_wrist_angle(self.target_pose)

        # Create trajectory message
        msg = JointTrajectory()
        msg.joint_names = self.arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = [shoulder_angle, elbow_angle, wrist_angle]
        point.time_from_start = Duration(sec=2, nanosec=0)

        msg.points = [point]
        self.trajectory_pub.publish(msg)

    def calculate_shoulder_angle(self, target_pose):
        """Calculate shoulder joint angle to reach target"""
        # Simplified calculation
        return 0.5  # rad

    def calculate_elbow_angle(self, target_pose):
        """Calculate elbow joint angle to reach target"""
        # Simplified calculation
        return 1.0  # rad

    def calculate_wrist_angle(self, target_pose):
        """Calculate wrist joint angle to reach target"""
        # Simplified calculation
        return 0.2  # rad

    def close_gripper(self):
        """Close the gripper to grasp object"""
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joint_names

        point = JointTrajectoryPoint()
        point.positions = [0.03, 0.03]  # Close fingers
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points = [point]
        self.trajectory_pub.publish(msg)

    def open_gripper(self):
        """Open the gripper to release object"""
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joint_names

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]  # Open fingers
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points = [point]
        self.trajectory_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = ManipulationController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down manipulation controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 3: Navigation and Obstacle Avoidance

This example demonstrates navigation and obstacle avoidance in simulation.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from std_msgs.msg import String
import math
import numpy as np

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'move_base_simple/goal',
            self.goal_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Timer for navigation loop
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)

        # State variables
        self.current_goal = None
        self.scan_data = None
        self.navigating = False
        self.safety_distance = 0.5  # meters

        # Navigation parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.arrival_threshold = 0.3

        self.get_logger().info('Navigation controller initialized')

    def goal_callback(self, msg):
        """Handle navigation goals"""
        self.current_goal = msg.pose
        self.navigating = True
        self.get_logger().info(f'New navigation goal: ({msg.pose.position.x}, {msg.pose.position.y})')

    def scan_callback(self, msg):
        """Handle laser scan data"""
        self.scan_data = msg

    def navigation_loop(self):
        """Main navigation loop"""
        if not self.current_goal or not self.scan_data:
            self.stop_robot()
            return

        # Check for obstacles
        if self.is_obstacle_ahead():
            self.handle_obstacle()
        else:
            self.navigate_to_goal()

    def is_obstacle_ahead(self):
        """Check if there's an obstacle in front of the robot"""
        if not self.scan_data:
            return False

        # Check the front 30-degree sector
        front_start = len(self.scan_data.ranges) // 2 - 15
        front_end = len(self.scan_data.ranges) // 2 + 15

        for i in range(front_start, front_end):
            if i < len(self.scan_data.ranges) and 0 < self.scan_data.ranges[i] < self.safety_distance:
                return True
        return False

    def navigate_to_goal(self):
        """Navigate towards the goal"""
        cmd_vel = Twist()

        # Calculate direction to goal
        goal_x = self.current_goal.position.x
        goal_y = self.current_goal.position.y

        # Simple proportional controller
        dx = goal_x
        dy = goal_y

        distance_to_goal = math.sqrt(dx*dx + dy*dy)

        if distance_to_goal < self.arrival_threshold:
            self.navigating = False
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.get_logger().info('Reached goal!')
        else:
            # Move toward goal
            cmd_vel.linear.x = min(self.linear_speed, distance_to_goal * 0.5)
            cmd_vel.angular.z = math.atan2(dy, dx) * 0.5

        self.cmd_vel_pub.publish(cmd_vel)

    def handle_obstacle(self):
        """Handle obstacle avoidance"""
        cmd_vel = Twist()

        # Simple obstacle avoidance: turn away from obstacle
        cmd_vel.angular.z = self.angular_speed
        cmd_vel.linear.x = 0.0  # Stop forward motion

        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().warn('Obstacle detected, turning...')

    def stop_robot(self):
        """Stop robot movement"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = NavigationController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down navigation controller...')
        controller.stop_robot()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 4: Sensor Fusion for Localization

This example demonstrates combining multiple sensors for improved localization.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R

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

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Timer for fusion loop
        self.fusion_timer = self.create_timer(0.05, self.fusion_loop)

        # State estimation
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.imu_data = None
        self.scan_data = None

        # Covariance matrix (3x3 for position, 3x3 for orientation)
        self.position_covariance = np.eye(3) * 0.1
        self.orientation_covariance = np.eye(3) * 0.05

        self.get_logger().info('Sensor fusion node initialized')

    def imu_callback(self, msg):
        """Handle IMU data"""
        self.imu_data = {
            'orientation': [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ],
            'angular_velocity': [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ],
            'linear_acceleration': [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]
        }

    def scan_callback(self, msg):
        """Handle laser scan data"""
        self.scan_data = {
            'ranges': msg.ranges,
            'intensities': msg.intensities,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }

    def fusion_loop(self):
        """Main sensor fusion loop"""
        # Predict state using IMU data
        if self.imu_data:
            self.predict_from_imu()

        # Update with scan-based localization if available
        if self.scan_data:
            self.update_with_scan()

        # Publish fused pose estimate
        self.publish_fused_pose()

    def predict_from_imu(self):
        """Predict state using IMU data"""
        # Integrate angular velocity to get orientation change
        ang_vel = np.array(self.imu_data['angular_velocity'])
        dt = 0.05  # timer period

        # Convert angular velocity to quaternion derivative
        omega_quat = np.array([0, *ang_vel]) * 0.5
        current_quat = np.array(self.imu_data['orientation'])

        # Integrate quaternion (simplified)
        # In practice, use proper quaternion integration
        dq = omega_quat * dt
        new_quat = current_quat + dq
        new_quat = new_quat / np.linalg.norm(new_quat)

        self.orientation = new_quat

        # Integrate linear acceleration to get velocity and position
        linear_acc = np.array(self.imu_data['linear_acceleration'])

        # Transform acceleration to world frame (simplified)
        world_acc = self.transform_vector_to_world(linear_acc, self.orientation)

        # Integrate acceleration to get velocity
        self.velocity += world_acc * dt

        # Integrate velocity to get position
        self.position += self.velocity * dt

    def transform_vector_to_world(self, vector, quaternion):
        """Transform vector from body frame to world frame using quaternion"""
        # Convert quaternion to rotation matrix
        r = R.from_quat([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
        return r.apply(vector)

    def update_with_scan(self):
        """Update state estimate with scan data"""
        # This would typically involve scan matching or landmark detection
        # For this example, we'll just adjust uncertainty based on scan quality
        valid_ranges = [r for r in self.scan_data['ranges'] if 0 < r < 10.0]

        if len(valid_ranges) > 50:  # If we have good scan data
            # Reduce uncertainty
            self.position_covariance *= 0.9
            self.orientation_covariance *= 0.9
        else:
            # Increase uncertainty if scan is poor
            self.position_covariance *= 1.1
            self.orientation_covariance *= 1.1

    def publish_fused_pose(self):
        """Publish the fused pose estimate"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Set position
        msg.pose.pose.position.x = float(self.position[0])
        msg.pose.pose.position.y = float(self.position[1])
        msg.pose.pose.position.z = float(self.position[2])

        # Set orientation
        msg.pose.pose.orientation.x = float(self.orientation[0])
        msg.pose.pose.orientation.y = float(self.orientation[1])
        msg.pose.pose.orientation.z = float(self.orientation[2])
        msg.pose.pose.orientation.w = float(self.orientation[3])

        # Set covariance (flattened)
        cov_matrix = np.zeros(36)
        cov_matrix[0:9] = self.position_covariance.flatten()  # x, y, z
        cov_matrix[21:27] = self.orientation_covariance.flatten()  # roll, pitch, yaw
        msg.pose.covariance = cov_matrix.tolist()

        self.fused_pose_pub.publish(msg)

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

## Running the Examples

### 1. Setup Simulation Environment

```bash
# Terminal 1: Start Gazebo
gz sim -r -v4

# Terminal 2: Spawn robot
ros2 run gazebo_ros spawn_entity.py -file simple_humanoid.urdf -entity humanoid_robot

# Terminal 3: Run controller
python3 walking_controller.py
```

### 2. Launch File Example

Create a launch file to run the complete simulation:

```python
# launch/simulation_example.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'simple_humanoid.urdf'
    )

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v4'],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_file, '-entity', 'humanoid_robot'],
            output='screen'
        ),

        # Start controllers
        Node(
            package='my_robot_controller',
            executable='walking_controller',
            name='walking_controller',
            output='screen'
        ),

        # Start navigation
        Node(
            package='my_robot_navigation',
            executable='navigation_controller',
            name='navigation_controller',
            output='screen'
        )
    ])
```

## Troubleshooting Common Issues

### 1. Robot Falls Through Ground
- Check that inertial properties are properly defined
- Verify collision geometry is present
- Adjust physics parameters in world file

### 2. Control Instability
- Reduce control loop frequency
- Add low-pass filtering to sensor data
- Tune PID controller parameters

### 3. Performance Issues
- Simplify collision geometry
- Reduce sensor update rates
- Use simpler physics engine settings

### 4. Sensor Noise Problems
- Verify noise parameters match real sensors
- Check frame transformations
- Validate sensor mounting positions

## Summary

These practical examples demonstrate the implementation of common humanoid robotics scenarios in simulation. Each example shows how to:

1. Create appropriate robot models for simulation
2. Implement control algorithms that work in simulated environments
3. Integrate multiple sensors for complex behaviors
4. Handle real-world challenges like obstacle avoidance and localization

The examples provide a foundation that can be extended and customized for specific humanoid robotics applications. Remember to validate your simulation results against real-world behavior to ensure effective sim-to-real transfer.