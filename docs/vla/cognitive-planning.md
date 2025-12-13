---
title: Cognitive Planning for VLA Systems
sidebar_position: 3
---

# Cognitive Planning for VLA Systems

Cognitive planning forms the decision-making core of Vision-Language-Action systems, enabling humanoid robots to reason about complex tasks, plan multi-step sequences, and adapt to changing conditions. This section covers the implementation of sophisticated planning systems that bridge high-level goals with low-level actions.

## Overview of Cognitive Planning

Cognitive planning in humanoid robots involves reasoning about actions, their consequences, and the best sequence to achieve desired goals. Unlike simple reactive systems, cognitive planning considers long-term objectives, environmental constraints, and potential future states when making decisions.

### Key Components of Cognitive Planning

#### 1. Task Planning
- **Goal Decomposition**: Breaking complex goals into manageable subtasks
- **Temporal Reasoning**: Understanding the timing and sequence of actions
- **Resource Management**: Managing robot capabilities and limitations
- **Constraint Handling**: Respecting physical and environmental constraints

#### 2. Motion Planning
- **Path Planning**: Finding collision-free trajectories through space
- **Manipulation Planning**: Planning arm and hand movements for object interaction
- **Gait Planning**: Coordinating walking patterns for locomotion
- **Balance Planning**: Maintaining stability during complex actions

#### 3. Adaptive Planning
- **Plan Monitoring**: Tracking plan execution and detecting deviations
- **Plan Repair**: Adjusting plans when execution fails
- **Contingency Planning**: Preparing alternative plans for likely failures
- **Learning from Experience**: Improving plans based on execution results

## Planning Architecture for Humanoid Robots

### Hierarchical Planning Structure

Cognitive planning systems typically use a hierarchical approach:

```
┌─────────────────────────────────────────────────────────────────┐
│                    TASK PLANNER                                 │
│  - High-level goal decomposition                                │
│  - Long-term strategic planning                                 │
│  - Resource allocation                                          │
└─────────────────┬───────────────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                 BEHAVIOR PLANNER                                │
│  - Mid-level behavior composition                               │
│  - Context-aware action selection                               │
│  - Safety constraint enforcement                                │
└─────────────────┬───────────────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                 MOTION PLANNER                                  │
│  - Low-level trajectory generation                              │
│  - Collision avoidance                                          │
│  - Kinematic constraint satisfaction                            │
└─────────────────┬───────────────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                 EXECUTION LAYER                                 │
│  - Joint control and motor commands                             │
│  - Real-time feedback processing                                │
│  - Emergency stop and safety                                          │
└─────────────────────────────────────────────────────────────────┘
```

### Integration with VLA Components

Cognitive planning integrates with other VLA components:

- **Vision Integration**: Uses visual perception for world state estimation
- **Language Integration**: Accepts goals and constraints through natural language
- **Action Integration**: Generates executable action sequences

## Task Planning Implementation

### Classical Planning Approaches

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from action_msgs.msg import GoalStatus
from enum import Enum
from typing import List, Dict, Any, Optional
import heapq

class TaskStatus(Enum):
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"

class Task:
    def __init__(self, name: str, goal: Dict[str, Any],
                 prerequisites: List[str] = None, priority: int = 0):
        self.name = name
        self.goal = goal  # Dictionary describing goal conditions
        self.prerequisites = prerequisites or []
        self.priority = priority
        self.status = TaskStatus.PENDING
        self.dependencies = []  # Tasks that depend on this one
        self.created_time = self.get_clock().now().nanoseconds / 1e9

    def is_ready(self, completed_tasks: List[str]) -> bool:
        """Check if all prerequisites are met"""
        return all(prereq in completed_tasks for prereq in self.prerequisites)

class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')

        # Publishers
        self.task_status_pub = self.create_publisher(String, 'task_status', 10)
        self.plan_request_pub = self.create_publisher(String, 'plan_request', 10)

        # Subscribers
        self.goal_sub = self.create_subscription(
            String, 'high_level_goal', self.goal_callback, 10
        )
        self.execution_status_sub = self.create_subscription(
            String, 'execution_status', self.execution_status_callback, 10
        )

        # Task planning state
        self.tasks = {}  # name -> Task
        self.completed_tasks = set()
        self.current_task = None
        self.planning_queue = []  # Priority queue for tasks

        # Timer for planning loop
        self.planning_timer = self.create_timer(0.5, self.planning_loop)

        self.get_logger().info('Task Planner Node initialized')

    def goal_callback(self, msg):
        """Handle high-level goals and decompose into tasks"""
        goal_text = msg.data
        self.get_logger().info(f'Received high-level goal: {goal_text}')

        # Decompose goal into subtasks (simplified)
        tasks = self.decompose_goal(goal_text)

        for task in tasks:
            self.add_task(task)

        # Publish planning request for current task
        if self.planning_queue:
            self.request_plan()

    def decompose_goal(self, goal_text: str) -> List[Task]:
        """Decompose high-level goal into tasks"""
        tasks = []

        if 'bring me' in goal_text or 'fetch' in goal_text:
            # Fetch task: go to object -> grasp -> bring back
            tasks.append(Task(
                name='find_object',
                goal={'object_detected': True},
                priority=1
            ))
            tasks.append(Task(
                name='navigate_to_object',
                goal={'at_location': 'object_location'},
                prerequisites=['find_object'],
                priority=2
            ))
            tasks.append(Task(
                name='grasp_object',
                goal={'object_grasped': True},
                prerequisites=['navigate_to_object'],
                priority=3
            ))
            tasks.append(Task(
                name='return_with_object',
                goal={'at_location': 'home_location'},
                prerequisites=['grasp_object'],
                priority=4
            ))

        elif 'go to' in goal_text:
            # Navigation task
            tasks.append(Task(
                name='navigate_to_location',
                goal={'at_location': self.extract_location(goal_text)},
                priority=1
            ))

        elif 'follow' in goal_text:
            # Following task
            tasks.append(Task(
                name='start_following',
                goal={'following_active': True},
                priority=1
            ))

        return tasks

    def extract_location(self, goal_text: str) -> str:
        """Extract target location from goal text"""
        # Simple location extraction (in practice, use NLP)
        if 'kitchen' in goal_text:
            return 'kitchen'
        elif 'living room' in goal_text:
            return 'living_room'
        elif 'bedroom' in goal_text:
            return 'bedroom'
        else:
            return 'unknown_location'

    def add_task(self, task: Task):
        """Add a task to the planning system"""
        if task.name not in self.tasks:
            self.tasks[task.name] = task
            heapq.heappush(self.planning_queue, (-task.priority, task.name))
            self.get_logger().info(f'Added task: {task.name} with priority {task.priority}')

    def planning_loop(self):
        """Main planning loop"""
        if not self.planning_queue:
            return

        # Get highest priority task that's ready
        _, task_name = self.planning_queue[0]
        task = self.tasks[task_name]

        if task.is_ready(list(self.completed_tasks)) and task.status == TaskStatus.PENDING:
            self.start_task(task)
        elif task.status == TaskStatus.IN_PROGRESS:
            # Task is already in progress, monitor it
            self.monitor_task(task)

    def start_task(self, task: Task):
        """Start executing a task"""
        task.status = TaskStatus.IN_PROGRESS
        self.current_task = task

        self.get_logger().info(f'Starting task: {task.name}')

        # Request plan for this task
        plan_request = String()
        plan_request.data = f"plan_for_task:{task.name}"
        self.plan_request_pub.publish(plan_request)

        # Publish task status
        status_msg = String()
        status_msg.data = f"task_started:{task.name}"
        self.task_status_pub.publish(status_msg)

    def monitor_task(self, task: Task):
        """Monitor ongoing task"""
        # In practice, this would check execution status
        # For now, simulate task completion
        if self.current_task and self.current_task.name == task.name:
            # Simulate completion after some time
            pass

    def execution_status_callback(self, msg):
        """Handle execution status updates"""
        status = msg.data
        self.get_logger().info(f'Execution status: {status}')

        if 'completed' in status and self.current_task:
            self.complete_task(self.current_task)
        elif 'failed' in status and self.current_task:
            self.fail_task(self.current_task)

    def complete_task(self, task: Task):
        """Mark a task as completed"""
        task.status = TaskStatus.COMPLETED
        self.completed_tasks.add(task.name)

        self.get_logger().info(f'Task completed: {task.name}')

        # Remove from planning queue
        if task.name in [t[1] for t in self.planning_queue]:
            self.planning_queue = [t for t in self.planning_queue if t[1] != task.name]

        # Publish completion
        status_msg = String()
        status_msg.data = f"task_completed:{task.name}"
        self.task_status_pub.publish(status_msg)

        # Clear current task
        if self.current_task and self.current_task.name == task.name:
            self.current_task = None

    def fail_task(self, task: Task):
        """Handle task failure"""
        task.status = TaskStatus.FAILED
        self.get_logger().warn(f'Task failed: {task.name}')

        # Publish failure
        status_msg = String()
        status_msg.data = f"task_failed:{task.name}"
        self.task_status_pub.publish(status_msg)

        # Clear current task
        if self.current_task and self.current_task.name == task.name:
            self.current_task = None

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down task planner node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Behavior Planning for Humanoid Robots

### Finite State Machine Approach

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from enum import Enum
from typing import Dict, Any
import time

class BehaviorState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    MANIPULATING = "manipulating"
    FOLLOWING = "following"
    AVOIDING = "avoiding"
    SPEAKING = "speaking"
    WAITING = "waiting"

class BehaviorPlannerNode(Node):
    def __init__(self):
        super().__init__('behavior_planner_node')

        # Publishers
        self.behavior_status_pub = self.create_publisher(String, 'behavior_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)

        # Subscribers
        self.task_sub = self.create_subscription(
            String, 'task_request', self.task_callback, 10
        )
        self.safety_sub = self.create_subscription(
            String, 'safety_status', self.safety_callback, 10
        )
        self.perception_sub = self.create_subscription(
            String, 'perception_data', self.perception_callback, 10
        )

        # Behavior state
        self.current_state = BehaviorState.IDLE
        self.previous_state = None
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9

        # State-specific data
        self.target_location = None
        self.target_object = None
        self.following_target = None

        # Timer for state transitions
        self.state_timer = self.create_timer(0.1, self.state_machine_update)

        self.get_logger().info('Behavior Planner Node initialized')

    def task_callback(self, msg):
        """Handle task requests and state transitions"""
        task_request = msg.data
        self.get_logger().info(f'Received task request: {task_request}')

        if 'navigate_to' in task_request:
            self.request_state_change(BehaviorState.NAVIGATING)
            self.target_location = task_request.split(':')[-1] if ':' in task_request else 'unknown'
        elif 'grasp' in task_request or 'manipulate' in task_request:
            self.request_state_change(BehaviorState.MANIPULATING)
            self.target_object = task_request.split(':')[-1] if ':' in task_request else 'unknown'
        elif 'follow' in task_request:
            self.request_state_change(BehaviorState.FOLLOWING)
            self.following_target = task_request.split(':')[-1] if ':' in task_request else 'unknown'
        elif 'speak' in task_request:
            self.request_state_change(BehaviorState.SPEAKING)

    def safety_callback(self, msg):
        """Handle safety-related state changes"""
        safety_status = msg.data

        if 'obstacle' in safety_status and 'detected' in safety_status:
            self.request_state_change(BehaviorState.AVOIDING)
        elif 'safe' in safety_status and self.current_state == BehaviorState.AVOIDING:
            # Return to previous state after obstacle is cleared
            if self.previous_state and self.previous_state != BehaviorState.AVOIDING:
                self.request_state_change(self.previous_state)

    def perception_callback(self, msg):
        """Handle perception updates that may trigger state changes"""
        perception_data = msg.data
        # Process perception data and potentially change state
        pass

    def request_state_change(self, new_state: BehaviorState):
        """Request a state change"""
        if self.current_state != new_state:
            self.get_logger().info(f'State change: {self.current_state.value} -> {new_state.value}')

            # Exit current state
            self.exit_state(self.current_state)

            # Store previous state
            self.previous_state = self.current_state

            # Change to new state
            self.current_state = new_state
            self.state_start_time = self.get_clock().now().nanoseconds / 1e9

            # Enter new state
            self.enter_state(new_state)

            # Publish state change
            status_msg = String()
            status_msg.data = f"state_changed:{new_state.value}"
            self.behavior_status_pub.publish(status_msg)

    def enter_state(self, state: BehaviorState):
        """Enter a state - perform initialization"""
        if state == BehaviorState.NAVIGATING:
            self.enter_navigating_state()
        elif state == BehaviorState.MANIPULATING:
            self.enter_manipulating_state()
        elif state == BehaviorState.FOLLOWING:
            self.enter_following_state()
        elif state == BehaviorState.AVOIDING:
            self.enter_avoiding_state()
        elif state == BehaviorState.SPEAKING:
            self.enter_speaking_state()

    def exit_state(self, state: BehaviorState):
        """Exit a state - perform cleanup"""
        if state == BehaviorState.NAVIGATING:
            self.exit_navigating_state()
        elif state == BehaviorState.MANIPULATING:
            self.exit_manipulating_state()
        elif state == BehaviorState.FOLLOWING:
            self.exit_following_state()
        elif state == BehaviorState.AVOIDING:
            self.exit_avoiding_state()
        elif state == BehaviorState.SPEAKING:
            self.exit_speaking_state()

    def enter_navigating_state(self):
        """Enter navigating state"""
        self.get_logger().info(f'Starting navigation to {self.target_location}')
        # In practice, send goal to navigation system

    def exit_navigating_state(self):
        """Exit navigating state"""
        # Stop any ongoing navigation
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def enter_manipulating_state(self):
        """Enter manipulating state"""
        self.get_logger().info(f'Starting manipulation of {self.target_object}')
        # In practice, send commands to manipulation system

    def exit_manipulating_state(self):
        """Exit manipulating state"""
        # Stop any ongoing manipulation
        pass

    def enter_following_state(self):
        """Enter following state"""
        self.get_logger().info(f'Starting to follow {self.following_target}')
        # In practice, start person following system

    def exit_following_state(self):
        """Exit following state"""
        # Stop following
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def enter_avoiding_state(self):
        """Enter obstacle avoidance state"""
        self.get_logger().info('Starting obstacle avoidance')
        # In practice, activate obstacle avoidance

    def exit_avoiding_state(self):
        """Exit obstacle avoidance state"""
        # Stop obstacle avoidance behavior
        pass

    def enter_speaking_state(self):
        """Enter speaking state"""
        self.get_logger().info('Starting speech output')
        # In practice, activate text-to-speech

    def exit_speaking_state(self):
        """Exit speaking state"""
        # Stop speech if needed
        pass

    def state_machine_update(self):
        """Update current state"""
        if self.current_state == BehaviorState.NAVIGATING:
            self.update_navigating_state()
        elif self.current_state == BehaviorState.MANIPULATING:
            self.update_manipulating_state()
        elif self.current_state == BehaviorState.FOLLOWING:
            self.update_following_state()
        elif self.current_state == BehaviorState.AVOIDING:
            self.update_avoiding_state()
        elif self.current_state == BehaviorState.SPEAKING:
            self.update_speaking_state()

    def update_navigating_state(self):
        """Update navigating state"""
        # In practice, monitor navigation progress
        # Check if goal reached
        pass

    def update_manipulating_state(self):
        """Update manipulating state"""
        # In practice, monitor manipulation progress
        pass

    def update_following_state(self):
        """Update following state"""
        # In practice, continue following behavior
        pass

    def update_avoiding_state(self):
        """Update avoiding state"""
        # In practice, continue obstacle avoidance
        pass

    def update_speaking_state(self):
        """Update speaking state"""
        # In practice, monitor speech output
        pass

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down behavior planner node...')
        # Stop all motion
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

## Motion Planning for Humanoid Locomotion

### Walking Pattern Generation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Twist
from builtin_interfaces.msg import Duration
import numpy as np
from scipy import interpolate
import math

class HumanoidMotionPlannerNode(Node):
    def __init__(self):
        super().__init__('humanoid_motion_planner_node')

        # Publishers
        self.step_plan_pub = self.create_publisher(Float64MultiArray, 'step_plan', 10)
        self.com_trajectory_pub = self.create_publisher(Float64MultiArray, 'com_trajectory', 10)
        self.joint_trajectory_pub = self.create_publisher(Float64MultiArray, 'joint_trajectory', 10)

        # Subscribers
        self.motion_request_sub = self.create_subscription(
            Twist, 'motion_request', self.motion_request_callback, 10
        )

        # Walking parameters
        self.step_length = 0.3  # meters
        self.step_width = 0.2   # meters (distance between feet)
        self.step_height = 0.05 # meters (clearance height)
        self.step_duration = 1.0 # seconds
        self.com_height = 0.8   # meters (center of mass height)

        # Walking state
        self.current_com_x = 0.0
        self.current_com_y = 0.0
        self.current_com_z = self.com_height
        self.support_foot = 'left'  # Which foot supports weight

        # Motion planning state
        self.trajectory_queue = []
        self.is_executing = False

        self.get_logger().info('Humanoid Motion Planner Node initialized')

    def motion_request_callback(self, msg):
        """Handle motion requests and generate walking patterns"""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        self.get_logger().info(f'Motion request: linear={linear_vel}, angular={angular_vel}')

        # Generate walking pattern based on requested motion
        step_sequence = self.generate_walking_pattern(linear_vel, angular_vel)

        # Publish step plan
        step_plan_msg = Float64MultiArray()
        step_plan_msg.data = step_sequence
        self.step_plan_pub.publish(step_plan_msg)

        # Generate COM trajectory
        com_trajectory = self.generate_com_trajectory(step_sequence)
        com_msg = Float64MultiArray()
        com_msg.data = com_trajectory
        self.com_trajectory_pub.publish(com_msg)

    def generate_walking_pattern(self, linear_vel, angular_vel):
        """Generate walking pattern for requested motion"""
        # Calculate number of steps based on desired movement
        num_steps = max(1, int(abs(linear_vel) * 2))  # Simple calculation

        step_sequence = []

        for i in range(num_steps):
            # Calculate step parameters
            step_x = self.step_length if linear_vel >= 0 else -self.step_length
            step_y = 0.0  # For straight walking

            # For turning, adjust step placement
            if abs(angular_vel) > 0.1:
                if i % 2 == 0:  # Left foot step
                    step_y = self.step_width / 2
                else:  # Right foot step
                    step_y = -self.step_width / 2

                # Adjust for turning radius
                turn_compensation = angular_vel * 0.1  # Adjust turning effect
                step_x *= (1 - abs(turn_compensation) * 0.1)
                step_y += turn_compensation

            # Add step to sequence
            step_params = [
                step_x,           # Step x displacement
                step_y,           # Step y displacement
                self.step_height, # Step height
                self.step_duration, # Step duration
                1.0 if i % 2 == 0 else -1.0  # Foot side (1.0 = left, -1.0 = right)
            ]

            step_sequence.extend(step_params)

        return step_sequence

    def generate_com_trajectory(self, step_sequence):
        """Generate Center of Mass trajectory for stable walking"""
        # This is a simplified COM trajectory generation
        # In practice, use inverted pendulum model or other advanced methods

        com_trajectory = []

        # Generate trajectory points for each step
        num_steps = len(step_sequence) // 5  # 5 parameters per step

        for i in range(num_steps):
            # Basic COM trajectory following the support foot
            # This maintains stability during walking

            # Lateral COM movement (side-to-side balance)
            lateral_offset = self.step_width / 4  # Keep COM over support polygon

            # Support foot determines COM position
            step_data = step_sequence[i*5:(i+1)*5]
            foot_side = step_data[4]  # 1.0 for left, -1.0 for right

            # Calculate COM position for this step
            com_x = self.current_com_x + step_data[0]  # Add step displacement
            com_y = lateral_offset if foot_side > 0 else -lateral_offset  # Opposite to support foot
            com_z = self.com_height  # Keep constant height

            # Add to trajectory
            com_trajectory.extend([com_x, com_y, com_z])

            # Update current COM position
            self.current_com_x = com_x
            self.current_com_y = com_y

        return com_trajectory

    def generate_joint_trajectory(self, step_sequence, com_trajectory):
        """Generate joint angle trajectories for walking"""
        # This would generate the actual joint movements
        # In practice, use inverse kinematics and walking pattern generators

        joint_trajectory = []

        # For each step, calculate required joint angles
        for i, step in enumerate(step_sequence):
            # This is a placeholder - real implementation would use
            # inverse kinematics to calculate joint angles for each step
            pass

        return joint_trajectory

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidMotionPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down humanoid motion planner node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Adaptive Planning and Plan Repair

### Handling Execution Failures

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from typing import Dict, List, Any, Optional
import time

class AdaptivePlannerNode(Node):
    def __init__(self):
        super().__init__('adaptive_planner_node')

        # Publishers
        self.recovery_command_pub = self.create_publisher(String, 'recovery_command', 10)
        self.plan_update_pub = self.create_publisher(String, 'plan_update', 10)

        # Subscribers
        self.execution_status_sub = self.create_subscription(
            String, 'execution_status', self.execution_status_callback, 10
        )
        self.failure_sub = self.create_subscription(
            String, 'failure_report', self.failure_callback, 10
        )

        # Planning state
        self.current_plan = []
        self.current_step = 0
        self.plan_history = []
        self.failure_history = []

        # Recovery strategies
        self.recovery_strategies = {
            'obstacle_detected': self.handle_obstacle,
            'stuck': self.handle_stuck,
            'unreachable': self.handle_unreachable,
            'kinematic_error': self.handle_kinematic_error,
            'balance_lost': self.handle_balance_loss
        }

        # Timer for plan monitoring
        self.monitor_timer = self.create_timer(0.5, self.plan_monitor)

        self.get_logger().info('Adaptive Planner Node initialized')

    def execution_status_callback(self, msg):
        """Monitor plan execution status"""
        status = msg.data
        self.get_logger().info(f'Execution status: {status}')

        if 'success' in status:
            self.current_step += 1
            if self.current_step >= len(self.current_plan):
                self.get_logger().info('Plan completed successfully')
        elif 'failed' in status:
            self.handle_execution_failure(status)

    def failure_callback(self, msg):
        """Handle specific failure reports"""
        failure_info = msg.data
        self.get_logger().warn(f'Failure reported: {failure_info}')

        # Add to failure history
        self.failure_history.append({
            'timestamp': time.time(),
            'failure': failure_info,
            'context': self.get_current_context()
        })

        # Determine appropriate recovery
        recovery_type = self.classify_failure(failure_info)
        if recovery_type in self.recovery_strategies:
            self.recovery_strategies[recovery_type](failure_info)

    def classify_failure(self, failure_info: str) -> str:
        """Classify failure type for appropriate recovery"""
        if 'obstacle' in failure_info or 'collision' in failure_info:
            return 'obstacle_detected'
        elif 'stuck' in failure_info or 'blocked' in failure_info:
            return 'stuck'
        elif 'reach' in failure_info or 'access' in failure_info:
            return 'unreachable'
        elif 'joint' in failure_info or 'kinematic' in failure_info:
            return 'kinematic_error'
        elif 'balance' in failure_info or 'fall' in failure_info:
            return 'balance_lost'
        else:
            return 'unknown'

    def handle_obstacle(self, failure_info: str):
        """Handle obstacle detection during navigation"""
        self.get_logger().info('Handling obstacle...')

        # Request replanning with obstacle avoidance
        recovery_msg = String()
        recovery_msg.data = 'replan_with_obstacle_avoidance'
        self.recovery_command_pub.publish(recovery_msg)

    def handle_stuck(self, failure_info: str):
        """Handle robot being stuck"""
        self.get_logger().info('Handling stuck condition...')

        # Try alternative approach
        recovery_msg = String()
        recovery_msg.data = 'backup_and_retry'
        self.recovery_command_pub.publish(recovery_msg)

    def handle_unreachable(self, failure_info: str):
        """Handle unreachable goal"""
        self.get_logger().info('Handling unreachable goal...')

        # Find alternative goal or report failure
        recovery_msg = String()
        recovery_msg.data = 'find_alternative_goal'
        self.recovery_command_pub.publish(recovery_msg)

    def handle_kinematic_error(self, failure_info: str):
        """Handle kinematic errors"""
        self.get_logger().info('Handling kinematic error...')

        # Adjust plan for kinematic constraints
        recovery_msg = String()
        recovery_msg.data = 'adjust_kinematic_constraints'
        self.recovery_command_pub.publish(recovery_msg)

    def handle_balance_loss(self, failure_info: str):
        """Handle balance loss"""
        self.get_logger().info('Handling balance loss...')

        # Emergency stop and recovery
        recovery_msg = String()
        recovery_msg.data = 'emergency_balance_recovery'
        self.recovery_command_pub.publish(recovery_msg)

    def handle_execution_failure(self, status: str):
        """Handle general execution failure"""
        self.get_logger().warn(f'Execution failed at step {self.current_step}: {status}')

        # Store current plan for learning
        self.plan_history.append({
            'plan': self.current_plan[:],
            'completed_steps': self.current_step,
            'failure': status
        })

        # Try to repair plan or generate new one
        self.attempt_plan_repair()

    def attempt_plan_repair(self):
        """Attempt to repair the current plan"""
        if not self.current_plan:
            return

        self.get_logger().info('Attempting plan repair...')

        # Analyze failure and create modified plan
        modified_plan = self.current_plan[self.current_step:]  # Remaining steps
        modified_plan = self.modify_plan_for_repair(modified_plan)

        if modified_plan:
            # Update current plan
            self.current_plan = modified_plan
            self.current_step = 0  # Start from beginning of modified plan

            # Publish plan update
            update_msg = String()
            update_msg.data = f"plan_repaired:steps_remaining={len(modified_plan)}"
            self.plan_update_pub.publish(update_msg)
        else:
            # Plan repair failed, request complete replanning
            self.request_replanning()

    def modify_plan_for_repair(self, plan: List[Any]) -> Optional[List[Any]]:
        """Modify plan to handle known failure patterns"""
        # This is a simplified plan modification
        # In practice, use more sophisticated plan repair algorithms

        if not plan:
            return None

        # Example: If first step involves navigation, add obstacle avoidance
        modified_plan = plan.copy()

        # Add safety checks or alternative approaches
        for i, step in enumerate(modified_plan):
            if 'navigate' in str(step):
                # Insert safety check before navigation
                modified_plan.insert(i, 'check_environment')
                break

        return modified_plan

    def request_replanning(self):
        """Request complete replanning from higher level"""
        self.get_logger().info('Requesting complete replanning')

        # Publish replanning request
        request_msg = String()
        request_msg.data = 'replanning_required'
        self.plan_update_pub.publish(request_msg)

    def get_current_context(self) -> Dict[str, Any]:
        """Get current planning context for failure analysis"""
        return {
            'current_step': self.current_step,
            'plan_length': len(self.current_plan) if self.current_plan else 0,
            'execution_time': time.time(),
            'previous_failures': len(self.failure_history)
        }

    def plan_monitor(self):
        """Monitor plan execution and detect potential issues"""
        # In practice, this would monitor various metrics
        # like execution time, deviation from plan, etc.
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AdaptivePlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down adaptive planner node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Planning with Uncertainty

### Probabilistic Planning Approaches

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from typing import Dict, List, Tuple
import numpy as np
from scipy.stats import norm

class UncertaintyAwarePlannerNode(Node):
    def __init__(self):
        super().__init__('uncertainty_aware_planner_node')

        # Publishers
        self.uncertainty_pub = self.create_publisher(Float64, 'uncertainty_metric', 10)
        self.probabilistic_plan_pub = self.create_publisher(String, 'probabilistic_plan', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'robot_pose', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # Uncertainty tracking
        self.pose_uncertainty = np.zeros((3, 3))  # 2D position + orientation
        self.sensor_uncertainty = 0.1  # Base sensor uncertainty
        self.motion_uncertainty = 0.05  # Base motion uncertainty

        # Planning with uncertainty parameters
        self.uncertainty_threshold = 0.5  # Maximum acceptable uncertainty
        self.replanning_margin = 0.3     # Safety margin for replanning

        # Timer for uncertainty assessment
        self.uncertainty_timer = self.create_timer(1.0, self.uncertainty_assessment)

        self.get_logger().info('Uncertainty Aware Planner Node initialized')

    def odom_callback(self, msg):
        """Update pose uncertainty from odometry with covariance"""
        # Extract covariance matrix from pose message
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        # Take position and orientation parts (x, y, theta)
        self.pose_uncertainty = cov[:3, :3]  # x, y, theta uncertainty

        # Publish current uncertainty metric
        uncertainty_metric = self.calculate_uncertainty_metric()
        uncertainty_msg = Float64()
        uncertainty_msg.data = uncertainty_metric
        self.uncertainty_pub.publish(uncertainty_msg)

    def scan_callback(self, msg):
        """Update sensor uncertainty based on scan quality"""
        # Calculate sensor uncertainty based on scan density and range
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            # More valid ranges = better localization
            scan_quality = len(valid_ranges) / len(msg.ranges)
            self.sensor_uncertainty = 0.1 * (1.0 - scan_quality) + 0.05

    def calculate_uncertainty_metric(self) -> float:
        """Calculate overall uncertainty metric"""
        # Calculate determinant of uncertainty matrix (volume of uncertainty ellipsoid)
        try:
            uncertainty_det = np.linalg.det(self.pose_uncertainty)
            # Take square root to get linear uncertainty measure
            linear_uncertainty = np.sqrt(max(0, uncertainty_det))
            return linear_uncertainty
        except np.linalg.LinAlgError:
            # If matrix is singular, use trace as approximation
            return np.trace(self.pose_uncertainty)

    def uncertainty_assessment(self):
        """Assess current uncertainty and adjust planning"""
        current_uncertainty = self.calculate_uncertainty_metric()

        self.get_logger().info(f'Current uncertainty: {current_uncertainty:.3f}')

        # Check if uncertainty is too high for current plan
        if current_uncertainty > self.uncertainty_threshold:
            self.get_logger().warn(f'High uncertainty detected: {current_uncertainty:.3f}')

            # Request replanning with uncertainty consideration
            self.request_uncertainty_aware_replanning()
        elif current_uncertainty > self.uncertainty_threshold - self.replanning_margin:
            self.get_logger().info('Uncertainty approaching threshold')
            # Prepare for potential replanning

    def request_uncertainty_aware_replanning(self):
        """Request replanning considering current uncertainties"""
        # Create probabilistic plan request
        plan_request = {
            'uncertainty_threshold': self.uncertainty_threshold,
            'current_uncertainty': self.calculate_uncertainty_metric(),
            'sensor_uncertainty': self.sensor_uncertainty,
            'motion_uncertainty': self.motion_uncertainty,
            'pose_covariance': self.pose_uncertainty.tolist()
        }

        plan_msg = String()
        plan_msg.data = str(plan_request)
        self.probabilistic_plan_pub.publish(plan_msg)

    def calculate_probabilistic_path(self, start_pose, goal_pose,
                                   uncertainty_map) -> List[Tuple[float, float]]:
        """Calculate path considering uncertainty"""
        # This would implement probabilistic roadmaps or other
        # uncertainty-aware path planning algorithms

        # Simplified example: adjust path based on uncertainty regions
        path = []

        # In practice, this would use algorithms like:
        # - Probabilistic Roadmaps (PRM) with uncertainty
        # - Chance-constrained planning
        # - Belief space planning

        # For now, return a simple straight line with uncertainty adjustments
        start_x, start_y = start_pose[:2]
        goal_x, goal_y = goal_pose[:2]

        # Break path into segments and check uncertainty along path
        steps = 10
        for i in range(steps + 1):
            t = i / steps
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)

            # Check uncertainty at this point (simplified)
            point_uncertainty = self.estimate_uncertainty_at_point(x, y)

            if point_uncertainty < self.uncertainty_threshold:
                path.append((x, y))
            else:
                # Add safety maneuver or reroute
                path.append((x, y))  # Simplified

        return path

    def estimate_uncertainty_at_point(self, x: float, y: float) -> float:
        """Estimate uncertainty at a specific point"""
        # This would use a spatial uncertainty map
        # For simplicity, return current robot uncertainty
        return self.calculate_uncertainty_metric()

def main(args=None):
    rclpy.init(args=args)
    node = UncertaintyAwarePlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down uncertainty aware planner node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with LLM-Based Planning

### Using Large Language Models for High-Level Planning

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import openai
import json
from typing import Dict, Any, List

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # Publishers
        self.high_level_plan_pub = self.create_publisher(String, 'high_level_plan', 10)
        self.llm_thought_pub = self.create_publisher(String, 'llm_thought_process', 10)

        # Subscribers
        self.natural_goal_sub = self.create_subscription(
            String, 'natural_language_goal', self.natural_goal_callback, 10
        )

        # OpenAI API configuration
        # In practice, set this via environment variable
        openai.api_key = "YOUR_OPENAI_API_KEY"

        # Robot context for LLM
        self.robot_context = {
            "capabilities": [
                "navigation", "manipulation", "object recognition",
                "speech interaction", "person following"
            ],
            "environment": {
                "rooms": ["kitchen", "living room", "bedroom", "office"],
                "objects": ["cup", "book", "bottle", "chair", "table"]
            }
        }

        self.get_logger().info('LLM Planner Node initialized')

    def natural_goal_callback(self, msg):
        """Process natural language goals with LLM"""
        natural_goal = msg.data
        self.get_logger().info(f'Processing natural goal: {natural_goal}')

        try:
            # Prepare context for LLM
            context = self.prepare_llm_context(natural_goal)

            # Call OpenAI API for planning
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": f"You are a planning assistant for a humanoid robot. "
                                  f"The robot has these capabilities: {self.robot_context['capabilities']}. "
                                  f"Available locations: {self.robot_context['environment']['rooms']}. "
                                  f"Available objects: {self.robot_context['environment']['objects']}. "
                                  f"Break down the user's request into executable steps. "
                                  f"Respond with a JSON array of steps, each with 'action', 'target', and 'description'."
                    },
                    {
                        "role": "user",
                        "content": context
                    }
                ],
                max_tokens=300,
                temperature=0.3
            )

            # Parse LLM response
            llm_response = response.choices[0].message['content'].strip()

            # Publish thought process
            thought_msg = String()
            thought_msg.data = llm_response
            self.llm_thought_pub.publish(thought_msg)

            # Extract JSON plan from response
            import re
            json_match = re.search(r'\[.*\]', llm_response, re.DOTALL)
            if json_match:
                plan_data = json.loads(json_match.group())

                # Publish high-level plan
                plan_msg = String()
                plan_msg.data = json.dumps(plan_data)
                self.high_level_plan_pub.publish(plan_msg)

                self.get_logger().info(f'Generated plan: {plan_data}')
            else:
                self.get_logger().warn('Could not extract plan from LLM response')

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in LLM response')
        except Exception as e:
            self.get_logger().error(f'LLM planning error: {e}')

    def prepare_llm_context(self, natural_goal: str) -> str:
        """Prepare context for LLM planning"""
        context = (
            f"The user wants: '{natural_goal}'. "
            f"Create a step-by-step plan using the robot's capabilities. "
            f"Consider the environment layout and available objects. "
            f"Make sure each step is specific and actionable."
        )
        return context

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM planner node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Planning Integration Example

### Complete Cognitive Planning System

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
import threading
import time

class CognitivePlanningSystemNode(Node):
    def __init__(self):
        super().__init__('cognitive_planning_system_node')

        # Publishers
        self.system_status_pub = self.create_publisher(String, 'planning_system_status', 10)

        # Subscribers for all planning components
        self.high_level_goal_sub = self.create_subscription(
            String, 'high_level_goal', self.high_level_goal_callback, 10
        )
        self.task_status_sub = self.create_subscription(
            String, 'task_status', self.task_status_callback, 10
        )
        self.behavior_status_sub = self.create_subscription(
            String, 'behavior_status', self.behavior_status_callback, 10
        )
        self.execution_status_sub = self.create_subscription(
            String, 'execution_status', self.execution_status_callback, 10
        )

        # Planning system state
        self.system_active = True
        self.current_plan = None
        self.plan_status = 'idle'
        self.active_components = {
            'task_planner': True,
            'behavior_planner': True,
            'motion_planner': True,
            'adaptive_planner': True,
            'llm_planner': True
        }

        # Timer for system monitoring
        self.system_timer = self.create_timer(0.5, self.system_monitor)

        # Initialize planning components (in practice, these would be separate nodes)
        self.initialize_planning_components()

        self.get_logger().info('Cognitive Planning System Node initialized')

    def initialize_planning_components(self):
        """Initialize all planning components"""
        self.get_logger().info('Initializing cognitive planning components...')

        # In practice, these would be separate ROS nodes
        # This is a simplified representation
        self.task_planner_active = True
        self.behavior_planner_active = True
        self.motion_planner_active = True
        self.adaptive_planner_active = True

    def high_level_goal_callback(self, msg):
        """Handle high-level goals for the entire system"""
        goal = msg.data
        self.get_logger().info(f'Received high-level goal: {goal}')

        # Distribute goal to appropriate planning components
        self.distribute_goal(goal)

    def distribute_goal(self, goal):
        """Distribute goal to appropriate planning components"""
        # This would publish the goal to different planning nodes
        # For example:
        # - Task planner for goal decomposition
        # - Behavior planner for behavior selection
        # - Motion planner for trajectory generation

        self.get_logger().info(f'Distributing goal to planning components: {goal}')

        # Update system status
        self.current_plan = goal
        self.plan_status = 'planning'

    def task_status_callback(self, msg):
        """Handle task planning status updates"""
        status = msg.data
        self.get_logger().info(f'Task status: {status}')

    def behavior_status_callback(self, msg):
        """Handle behavior planning status updates"""
        status = msg.data
        self.get_logger().info(f'Behavior status: {status}')

    def execution_status_callback(self, msg):
        """Handle execution status updates"""
        status = msg.data
        self.get_logger().info(f'Execution status: {status}')

        # Update plan status based on execution
        if 'completed' in status:
            self.plan_status = 'completed'
        elif 'failed' in status:
            self.plan_status = 'failed'
        elif 'executing' in status:
            self.plan_status = 'executing'

    def system_monitor(self):
        """Monitor overall system health and status"""
        status_msg = String()
        status_msg.data = f"system_status:active:{self.plan_status}:{self.current_plan}"
        self.system_status_pub.publish(status_msg)

        # Check if all components are active
        all_active = all(self.active_components.values())
        if not all_active:
            self.get_logger().warn('Some planning components are not active')

    def get_system_status(self) -> Dict:
        """Get current system status"""
        return {
            'system_active': self.system_active,
            'current_plan': self.current_plan,
            'plan_status': self.plan_status,
            'components': self.active_components
        }

def main(args=None):
    rclpy.init(args=args)
    node = CognitivePlanningSystemNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down cognitive planning system node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## System Integration and Launch

### Complete Cognitive Planning System Launch File

```xml
<!-- cognitive_planning_system.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        # Task planner node
        Node(
            package='your_robot_package',
            executable='task_planner_node',
            name='task_planner',
            parameters=[
                {'planning_frequency': 1.0},
                {'max_planning_time': 10.0}
            ],
            output='screen'
        ),

        # Behavior planner node
        Node(
            package='your_robot_package',
            executable='behavior_planner_node',
            name='behavior_planner',
            output='screen'
        ),

        # Humanoid motion planner node
        Node(
            package='your_robot_package',
            executable='humanoid_motion_planner_node',
            name='motion_planner',
            output='screen'
        ),

        # Adaptive planner node
        Node(
            package='your_robot_package',
            executable='adaptive_planner_node',
            name='adaptive_planner',
            output='screen'
        ),

        # Uncertainty aware planner node
        Node(
            package='your_robot_package',
            executable='uncertainty_aware_planner_node',
            name='uncertainty_planner',
            output='screen'
        ),

        # LLM planner node (if using cloud services)
        Node(
            package='your_robot_package',
            executable='llm_planner_node',
            name='llm_planner',
            output='screen'
        ),

        # Main cognitive planning system node
        Node(
            package='your_robot_package',
            executable='cognitive_planning_system_node',
            name='cognitive_planning_system',
            output='screen'
        )
    ])
```

## Testing Cognitive Planning Systems

### Unit Tests for Planning Components

```python
#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestCognitivePlanning(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_cognitive_planning_node')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_task_planning(self):
        """Test that task planning decomposes goals correctly"""
        self.assertTrue(True)  # Placeholder for actual test

    def test_behavior_selection(self):
        """Test that appropriate behaviors are selected"""
        self.assertTrue(True)  # Placeholder for actual test

    def test_plan_execution_monitoring(self):
        """Test that plan execution is properly monitored"""
        self.assertTrue(True)  # Placeholder for actual test

    def test_adaptive_replanning(self):
        """Test that replanning occurs when needed"""
        self.assertTrue(True)  # Placeholder for actual test

if __name__ == '__main__':
    unittest.main()
```

## Troubleshooting Planning Issues

### 1. Planning Timeouts
- **Cause**: Complex planning problems taking too long
- **Solution**: Implement hierarchical planning or anytime algorithms
- **Check**: Set appropriate timeout parameters

### 2. Unreachable Goals
- **Cause**: Goals outside robot capabilities
- **Solution**: Implement goal validation and alternative planning
- **Check**: Verify kinematic and environmental constraints

### 3. Plan Failures
- **Cause**: Execution failures causing plan abandonment
- **Solution**: Implement robust recovery strategies
- **Check**: Monitor execution status and adapt plans

### 4. Uncertainty Accumulation
- **Cause**: Sensor and motion uncertainty growing over time
- **Solution**: Implement localization and replanning
- **Check**: Monitor uncertainty metrics continuously

## Best Practices for Cognitive Planning

### 1. Modularity
- Keep planning components independent
- Use clear interfaces between components
- Design for easy replacement of individual components

### 2. Robustness
- Handle failure cases gracefully
- Implement multiple recovery strategies
- Plan for partial plan execution

### 3. Efficiency
- Use appropriate planning algorithms for each level
- Implement caching for common plans
- Optimize for real-time execution

### 4. Safety
- Include safety checks in all planning decisions
- Implement emergency stop capabilities
- Validate plans before execution

## Summary

Cognitive planning systems for VLA enable humanoid robots to reason about complex tasks and execute them successfully. Key components include:

1. **Task Planning**: Decomposing high-level goals into executable subtasks
2. **Behavior Planning**: Selecting appropriate behaviors based on context
3. **Motion Planning**: Generating safe and efficient trajectories
4. **Adaptive Planning**: Handling failures and adjusting plans dynamically
5. **Uncertainty Management**: Planning under uncertain conditions
6. **LLM Integration**: Using large language models for natural language understanding

These components work together to create intelligent robots capable of performing complex tasks in dynamic environments while adapting to changes and failures.