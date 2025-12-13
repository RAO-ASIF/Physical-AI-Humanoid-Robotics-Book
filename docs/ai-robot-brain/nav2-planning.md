---
title: Nav2 Path Planning for Humanoid Robots
sidebar_position: 5
---

# Nav2 Path Planning for Humanoid Robots

Navigation 2 (Nav2) provides the standard navigation stack for ROS 2, enabling robots to autonomously navigate through environments. For humanoid robots, Nav2 requires special configuration to handle bipedal locomotion and human-scale environments.

## Overview of Nav2 for Humanoid Robots

Nav2 in humanoid robotics involves planning and executing paths that account for the unique kinematic constraints of bipedal locomotion. Unlike wheeled robots, humanoid robots must maintain balance while navigating, making path planning more complex.

### Key Components for Humanoid Navigation

#### 1. Global Planner
- **A* Algorithm**: For optimal pathfinding with cost considerations
- **Dijkstra's Algorithm**: For reliable pathfinding in complex environments
- **Potential Fields**: For handling dynamic obstacles

#### 2. Local Planner
- **DWA (Dynamic Window Approach)**: For real-time trajectory planning
- **Teb Local Planner**: For time-elastic band planning with smooth trajectories
- **Humanoid-specific controllers**: For balance-aware navigation

#### 3. Costmap Management
- **Static Layer**: For permanent obstacles
- **Obstacle Layer**: For dynamic obstacle detection
- **Inflation Layer**: For safety margins around obstacles
- **Voxel Layer**: For 3D obstacle representation

## Nav2 Configuration for Humanoid Robots

### Costmap Configuration

For humanoid robots, costmaps need to account for the robot's size and balance constraints:

```yaml
# costmap_common_params_humanoid.yaml
map_type: costmap
origin_z: 0.0
z_resolution: 0.2
z_voxels: 10

obstacle_range: 3.0
raytrace_range: 4.0

# Footprint for humanoid robot (approximate rectangular base)
footprint: [[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]
footprint_padding: 0.05

# Robot dimensions for collision checking
robot_radius: 0.3

inflation:
  enabled: true
  cost_scaling_factor: 3.0
  inflation_radius: 0.5
  inflate_to_max: false

observation_sources: scan camera_depth

scan:
  sensor_frame: base_scan
  data_type: LaserScan
  topic: /scan
  marking: true
  clearing: true
  obstacle_range: 3.0
  raytrace_range: 4.0

camera_depth:
  sensor_frame: camera_depth_frame
  data_type: PointCloud2
  topic: /camera/depth/points
  marking: true
  clearing: true
  obstacle_range: 3.0
  raytrace_range: 4.0
```

### Planner Configuration

```yaml
# nav2_params_humanoid.yaml
amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      roll: 0.0
      pitch: 0.0
      yaw: 0.0

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_aligner_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 50
      model_dt: 0.1
      batch_size: 1000
      vx_std: 0.2
      vy_std: 0.05
      wz_std: 0.3
      vx_max: 0.5
      vx_min: -0.1
      vy_max: 0.1
      wz_max: 0.5
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      state_reset_tol: 0.5
      ctrl_freq: 10.0
      sim_freq: 50.0
      horizon_dt: 1.0
      replan_on_exception: true
      motion_model: "DiffDrive"
      reference_frame: "odom"
      odom_topic: "/odom"
      cmd_vel_topic: "/cmd_vel"
      transform_tolerance: 0.1
      goal_tolerance: 0.1
      xy_moving_average_length: 5
      theta_moving_average_length: 5
      scale_smooth_control: false
      smooth_vel: false
      frequency: 20.0
      forward_penalty: 0.2
      curvature_penalty: 0.2
      goal_angle_penalty: 0.2
      obstacle_cost_penalty: 1.0
      reference_cost_penalty: 0.5
      obstacle_heading_threshold: 0.4

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        map_subscribe_transient_local: true
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      robot_radius: 0.3
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: true

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors::BackUp"
      backup_dist: 0.15
      backup_speed: 0.025
    wait:
      plugin: "nav2_behaviors::Wait"
      wait_duration: 1.0
```

## Navigation Node Implementation

### Basic Navigation Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

class HumanoidNavigationNode(Node):
    def __init__(self):
        super().__init__('humanoid_navigation_node')

        # Create action client for navigation
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # TF buffer and listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Navigation state
        self.current_goal = None
        self.is_navigating = False

        self.get_logger().info('Humanoid Navigation Node initialized')

    def send_navigation_goal(self, x, y, theta):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()

        # Set the goal pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        from math import sin, cos
        from geometry_msgs.msg import Quaternion

        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = sin(theta / 2.0)
        quat.w = cos(theta / 2.0)
        goal_msg.pose.pose.orientation = quat

        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Send goal
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.current_goal = goal_msg
        self.is_navigating = True

        return True

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted')

        # Get result future
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # Log navigation progress
        self.get_logger().info(
            f'Navigation progress: {feedback.distance_remaining:.2f}m remaining'
        )

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation completed with result: {result}')
        self.is_navigating = False

    def cancel_current_goal(self):
        """Cancel the current navigation goal"""
        if self.is_navigating:
            # Cancel goal logic here
            self.get_logger().info('Canceling current navigation goal')
            self.is_navigating = False

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidNavigationNode()

    try:
        # Example: Navigate to a specific location
        node.send_navigation_goal(1.0, 1.0, 0.0)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down navigation node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Gait-Aware Navigation

Humanoid robots require gait-aware navigation that considers their walking patterns and balance:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import numpy as np
from scipy import interpolate

class GaitAwareNavigationNode(Node):
    def __init__(self):
        super().__init__('gait_aware_navigation_node')

        # Publishers and subscribers
        self.path_sub = self.create_subscription(
            Path, 'global_plan', self.path_callback, 10
        )
        self.smoothed_path_pub = self.create_publisher(
            Path, 'smoothed_plan', 10
        )

        # Gait parameters
        self.step_length = 0.3  # meters
        self.step_height = 0.05  # meters
        self.step_duration = 1.0  # seconds
        self.gait_frequency = 1.0  # Hz

        # Walking state
        self.is_walking = False
        self.current_step = 0

        self.get_logger().info('Gait-Aware Navigation Node initialized')

    def path_callback(self, msg):
        """Process incoming path and make it gait-aware"""
        if len(msg.poses) < 2:
            return

        # Smooth the path using cubic splines
        smoothed_path = self.smooth_path(msg)

        # Adjust for gait constraints
        gait_compatible_path = self.make_gait_compatible(smoothed_path)

        # Publish the gait-aware path
        self.smoothed_path_pub.publish(gait_compatible_path)

    def smooth_path(self, path):
        """Smooth the path using cubic splines"""
        if len(path.poses) < 3:
            return path

        # Extract x and y coordinates
        x_coords = [pose.pose.position.x for pose in path.poses]
        y_coords = [pose.pose.position.y for pose in path.poses]

        # Create parameter for interpolation
        distances = [0]
        for i in range(1, len(x_coords)):
            dist = np.sqrt(
                (x_coords[i] - x_coords[i-1])**2 +
                (y_coords[i] - y_coords[i-1])**2
            )
            distances.append(distances[-1] + dist)

        # Create cubic spline interpolator
        fx = interpolate.interp1d(distances, x_coords, kind='cubic')
        fy = interpolate.interp1d(distances, y_coords, kind='cubic')

        # Generate smooth path with higher resolution
        new_distances = np.linspace(distances[0], distances[-1],
                                   len(distances) * 3)
        smooth_x = fx(new_distances)
        smooth_y = fy(new_distances)

        # Create new path message
        smooth_path = Path()
        smooth_path.header = path.header

        for i in range(len(smooth_x)):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = smooth_x[i]
            pose.pose.position.y = smooth_y[i]
            pose.pose.position.z = 0.0

            # Calculate orientation from path direction
            if i < len(smooth_x) - 1:
                dx = smooth_x[i+1] - smooth_x[i]
                dy = smooth_y[i+1] - smooth_y[i]
                theta = np.arctan2(dy, dx)

                from math import sin, cos
                pose.pose.orientation.z = sin(theta / 2.0)
                pose.pose.orientation.w = cos(theta / 2.0)
            else:
                # Use previous orientation
                if len(path.poses) > 0:
                    pose.pose.orientation = path.poses[-1].pose.orientation

            smooth_path.poses.append(pose)

        return smooth_path

    def make_gait_compatible(self, path):
        """Adjust path for humanoid gait constraints"""
        gait_path = Path()
        gait_path.header = path.header

        # Ensure path points are spaced according to step length
        current_pos = None
        for i, pose in enumerate(path.poses):
            if current_pos is None:
                current_pos = np.array([
                    pose.pose.position.x,
                    pose.pose.position.y
                ])
                gait_path.poses.append(pose)
                continue

            pos = np.array([pose.pose.position.x, pose.pose.position.y])
            distance = np.linalg.norm(pos - current_pos)

            if distance >= self.step_length * 0.8:  # Allow some flexibility
                gait_path.poses.append(pose)
                current_pos = pos

        return gait_path

def main(args=None):
    rclpy.init(args=args)
    node = GaitAwareNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gait-aware navigation node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Recovery Behaviors for Humanoid Robots

Humanoid robots need specialized recovery behaviors that account for their balance and walking capabilities:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class HumanoidRecoveryNode(Node):
    def __init__(self):
        super().__init__('humanoid_recovery_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.recovery_status_pub = self.create_publisher(
            String, 'recovery_status', 10
        )

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # Recovery parameters
        self.recovery_timeout = 30.0  # seconds
        self.min_clearance = 0.4  # meters
        self.backup_distance = 0.3  # meters
        self.spin_angle = 45.0  # degrees

        # State variables
        self.scan_data = None
        self.recovery_active = False
        self.recovery_start_time = None

        self.get_logger().info('Humanoid Recovery Node initialized')

    def scan_callback(self, msg):
        """Update scan data for obstacle detection"""
        self.scan_data = msg

    def check_obstacle_clearance(self):
        """Check if there's sufficient clearance around robot"""
        if self.scan_data is None:
            return True

        # Check for obstacles in front
        front_ranges = self.scan_data.ranges[
            len(self.scan_data.ranges)//4 : 3*len(self.scan_data.ranges)//4
        ]

        min_front_dist = min([r for r in front_ranges if r > 0.0 and r < 10.0],
                            default=float('inf'))

        return min_front_dist > self.min_clearance

    def execute_backup_recovery(self):
        """Execute backup recovery behavior"""
        if not self.recovery_active:
            self.recovery_active = True
            self.recovery_start_time = self.get_clock().now().nanoseconds / 1e9

        # Create backup command
        cmd = Twist()
        cmd.linear.x = -0.1  # Back up slowly
        cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

        # Stop after backup distance
        if self.get_clock().now().nanoseconds / 1e9 - self.recovery_start_time > 3.0:
            self.stop_recovery()
            return True  # Recovery completed

        return False  # Recovery still in progress

    def execute_spin_recovery(self):
        """Execute spin recovery behavior"""
        if not self.recovery_active:
            self.recovery_active = True
            self.recovery_start_time = self.get_clock().now().nanoseconds / 1e9

        # Create spin command
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.3  # Spin slowly

        self.cmd_vel_pub.publish(cmd)

        # Stop after spinning 90 degrees
        elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.recovery_start_time
        if elapsed_time > 2.5:  # ~90 degrees at 0.3 rad/s
            self.stop_recovery()
            return True  # Recovery completed

        return False  # Recovery still in progress

    def stop_recovery(self):
        """Stop recovery behavior and reset state"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

        self.recovery_active = False
        self.recovery_start_time = None

    def execute_recovery_behavior(self, behavior_type):
        """Execute specified recovery behavior"""
        if behavior_type == 'backup':
            return self.execute_backup_recovery()
        elif behavior_type == 'spin':
            return self.execute_spin_recovery()
        else:
            self.get_logger().warn(f'Unknown recovery behavior: {behavior_type}')
            return True  # Return true to indicate failure

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidRecoveryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down recovery node...')
    finally:
        node.stop_recovery()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Humanoid Control Systems

### Walking Controller Interface

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np

class WalkingControllerInterface(Node):
    def __init__(self):
        super().__init__('walking_controller_interface')

        # Publishers for walking commands
        self.step_cmd_pub = self.create_publisher(
            Float64MultiArray, 'walking_step_commands', 10
        )
        self.balance_cmd_pub = self.create_publisher(
            Float64MultiArray, 'balance_commands', 10
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        # Walking parameters
        self.step_frequency = 1.0  # steps per second
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 0.5  # rad/s
        self.step_size = 0.3  # meters per step

        # State variables
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.current_pose = Pose()

        # Timer for walking control
        self.walking_timer = self.create_timer(
            1.0 / self.step_frequency, self.walking_control_callback
        )

        self.get_logger().info('Walking Controller Interface initialized')

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from Nav2"""
        # Limit velocities to humanoid capabilities
        self.current_linear_vel = max(
            -self.max_linear_speed,
            min(self.max_linear_speed, msg.linear.x)
        )
        self.current_angular_vel = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, msg.angular.z)
        )

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.current_pose = msg.pose.pose

    def walking_control_callback(self):
        """Generate walking step commands based on desired velocity"""
        if abs(self.current_linear_vel) < 0.01 and abs(self.current_angular_vel) < 0.01:
            # Stop walking if no velocity command
            self.publish_stop_walking()
            return

        # Calculate step parameters based on desired velocity
        step_duration = 1.0 / self.step_frequency

        # Calculate step size based on linear velocity
        step_size = self.current_linear_vel * step_duration

        # Calculate step rotation based on angular velocity
        step_rotation = self.current_angular_vel * step_duration

        # Generate walking command
        walking_cmd = Float64MultiArray()
        walking_cmd.data = [
            step_size,      # Step size in meters
            step_rotation,  # Step rotation in radians
            0.0,           # Step height (fixed for now)
            0.5,           # Step timing (50% support phase)
            0.0            # Additional balance parameters
        ]

        self.step_cmd_pub.publish(walking_cmd)

        # Generate balance commands
        balance_cmd = Float64MultiArray()
        balance_cmd.data = [
            0.0,  # Desired COM x offset
            0.0,  # Desired COM y offset
            0.8,  # Desired COM height
            0.0,  # Desired torso pitch
            0.0   # Desired torso roll
        ]

        self.balance_cmd_pub.publish(balance_cmd)

    def publish_stop_walking(self):
        """Publish stop command to walking controller"""
        walking_cmd = Float64MultiArray()
        walking_cmd.data = [0.0, 0.0, 0.0, 0.5, 0.0]  # All zeros to stop
        self.step_cmd_pub.publish(walking_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WalkingControllerInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down walking controller interface...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Examples and Best Practices

### Example 1: Simple Navigation with Waypoints

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Define waypoints
        self.waypoints = [
            (1.0, 1.0, 0.0),    # x, y, theta
            (2.0, 2.0, 1.57),   # x, y, theta
            (1.0, 3.0, 3.14),   # x, y, theta
            (0.0, 2.0, -1.57),  # x, y, theta
        ]

        self.current_waypoint = 0
        self.navigation_active = False

        # Timer to start navigation
        self.nav_timer = self.create_timer(2.0, self.navigate_to_waypoints)

    def navigate_to_waypoints(self):
        """Navigate through predefined waypoints"""
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            return

        if not self.navigation_active:
            x, y, theta = self.waypoints[self.current_waypoint]
            self.get_logger().info(f'Navigating to waypoint {self.current_waypoint}: ({x}, {y})')

            if self.send_navigation_goal(x, y, theta):
                self.navigation_active = True

    def send_navigation_goal(self, x, y, theta):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        from math import sin, cos
        from geometry_msgs.msg import Quaternion

        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = sin(theta / 2.0)
        quat.w = cos(theta / 2.0)
        goal_msg.pose.pose.orientation = quat

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.navigation_active = False
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Waypoint {self.current_waypoint} progress: {feedback.distance_remaining:.2f}m remaining'
        )

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Waypoint {self.current_waypoint} reached')

        # Move to next waypoint
        self.current_waypoint += 1
        self.navigation_active = False

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down waypoint navigator...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Dynamic Obstacle Avoidance

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class DynamicObstacleAvoider(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_avoider')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # Parameters
        self.safe_distance = 0.5  # meters
        self.avoidance_threshold = 0.8  # meters
        self.max_linear_speed = 0.3  # m/s
        self.max_angular_speed = 0.5  # rad/s

        # State
        self.scan_data = None
        self.obstacle_detected = False

        # Timer for obstacle avoidance
        self.avoidance_timer = self.create_timer(0.1, self.obstacle_avoidance_callback)

    def scan_callback(self, msg):
        """Update scan data"""
        self.scan_data = msg

    def obstacle_avoidance_callback(self):
        """Main obstacle avoidance logic"""
        if self.scan_data is None:
            return

        # Check for obstacles in front
        front_ranges = self.scan_data.ranges[
            len(self.scan_data.ranges)//4 : 3*len(self.scan_data.ranges)//4
        ]

        min_front_dist = min([r for r in front_ranges if r > 0.0 and r < 10.0],
                            default=float('inf'))

        cmd = Twist()

        if min_front_dist < self.safe_distance:
            # Emergency stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)

            self.get_logger().warn('Emergency stop - obstacle too close!')
        elif min_front_dist < self.avoidance_threshold:
            # Obstacle avoidance mode
            cmd.linear.x = 0.0  # Stop forward motion
            cmd.angular.z = self.calculate_avoidance_direction()
            self.cmd_vel_pub.publish(cmd)
        else:
            # Normal navigation - let Nav2 handle it
            emergency_msg = Bool()
            emergency_msg.data = False
            self.emergency_stop_pub.publish(emergency_msg)

    def calculate_avoidance_direction(self):
        """Calculate best avoidance direction"""
        if self.scan_data is None:
            return 0.0

        # Divide scan into left and right halves
        mid_idx = len(self.scan_data.ranges) // 2
        left_ranges = self.scan_data.ranges[:mid_idx]
        right_ranges = self.scan_data.ranges[mid_idx:]

        # Calculate average distance on each side
        left_avg = np.mean([r for r in left_ranges if r > 0.0 and r < 10.0])
        right_avg = np.mean([r for r in right_ranges if r > 0.0 and r < 10.0])

        # Choose direction with more clearance
        if left_avg > right_avg:
            return self.max_angular_speed  # Turn left
        else:
            return -self.max_angular_speed  # Turn right

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleAvoider()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down obstacle avoider...')
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

## Troubleshooting Common Issues

### 1. Navigation Fails to Start
- **Cause**: TF tree issues between map, odom, and base_link
- **Solution**: Verify all required transforms are published
- **Check**: Use `ros2 run tf2_tools view_frames` to visualize the transform tree

### 2. Robot Gets Stuck in Local Minima
- **Cause**: Poor global planner configuration
- **Solution**: Adjust costmap inflation parameters or use a different global planner
- **Check**: Increase inflation radius in costmap configuration

### 3. Excessive Oscillation
- **Cause**: Local planner parameters not tuned for humanoid robot
- **Solution**: Reduce maximum velocities and adjust controller parameters
- **Check**: Lower linear and angular velocity limits in controller configuration

### 4. Recovery Behaviors Not Working
- **Cause**: Recovery behavior parameters not set correctly
- **Solution**: Adjust recovery behavior timeouts and distances
- **Check**: Verify behavior server configuration in Nav2 parameters

## Best Practices for Humanoid Navigation

### 1. Parameter Tuning
- Start with conservative velocity limits and gradually increase
- Tune costmap inflation based on robot size and balance requirements
- Test recovery behaviors in controlled environments first

### 2. Safety Considerations
- Implement emergency stop mechanisms for obstacle avoidance
- Use multiple sensor modalities for robust perception
- Add balance monitoring during navigation

### 3. Performance Optimization
- Use appropriate costmap resolution for humanoid navigation
- Implement efficient path smoothing for gait compatibility
- Monitor computational load during navigation

### 4. Testing Strategy
- Test in simulation before real-world deployment
- Start with simple environments and gradually increase complexity
- Validate navigation performance with various gait patterns

## Summary

Nav2 provides a comprehensive navigation solution for humanoid robots, but requires careful configuration to account for the unique challenges of bipedal locomotion. Key considerations include:

1. **Costmap Configuration**: Properly sized footprints and inflation parameters for humanoid dimensions
2. **Controller Tuning**: Velocities and parameters appropriate for walking gaits
3. **Gait Integration**: Path smoothing and timing compatible with walking patterns
4. **Balance Considerations**: Recovery behaviors that maintain stability
5. **Safety Systems**: Emergency stop and obstacle avoidance mechanisms

By following these guidelines and implementing the provided code examples, humanoid robots can achieve reliable autonomous navigation in human environments.

## References

- ROS Navigation Stack Documentation
- Nav2 User Manual
- Humanoid Robotics Navigation Best Practices
- Mobile Robot Navigation Algorithms