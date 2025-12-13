# Practical URDF Examples for Humanoid Components

## Humanoid Robot URDF Example

Here's a practical example of a simplified humanoid robot model with torso, head, arms, and legs:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_height" value="0.5" />
  <xacro:property name="torso_radius" value="0.15" />
  <xacro:property name="head_height" value="0.2" />
  <xacro:property name="head_radius" value="0.12" />
  <xacro:property name="arm_length" value="0.4" />
  <xacro:property name="arm_radius" value="0.05" />
  <xacro:property name="forearm_length" value="0.35" />
  <xacro:property name="forearm_radius" value="0.04" />
  <xacro:property name="leg_length" value="0.45" />
  <xacro:property name="leg_radius" value="0.08" />
  <xacro:property name="shin_length" value="0.45" />
  <xacro:property name="shin_radius" value="0.08" />

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 ${torso_height/2}"/>
      <geometry>
        <cylinder radius="${torso_radius}" length="${torso_height}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${torso_height/2}"/>
      <geometry>
        <cylinder radius="${torso_radius}" length="${torso_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 ${torso_height/2}"/>
      <inertia ixx="0.42" ixy="0.0" ixz="0.0" iyy="0.42" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 ${head_height/2}"/>
      <geometry>
        <cylinder radius="${head_radius}" length="${head_height}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${head_height/2}"/>
      <geometry>
        <cylinder radius="${head_radius}" length="${head_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 ${head_height/2}"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 ${torso_height}"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 ${arm_length/2}"/>
      <geometry>
        <cylinder radius="${arm_radius}" length="${arm_length}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${arm_length/2}"/>
      <geometry>
        <cylinder radius="${arm_radius}" length="${arm_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 ${arm_length/2}"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="${torso_radius} 0 ${torso_height*0.7}"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="50" velocity="1"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <origin xyz="0 0 ${forearm_length/2}"/>
      <geometry>
        <cylinder radius="${forearm_radius}" length="${forearm_length}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${forearm_length/2}"/>
      <geometry>
        <cylinder radius="${forearm_radius}" length="${forearm_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 ${forearm_length/2}"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 ${arm_length}"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="30" velocity="1"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 ${arm_length/2}"/>
      <geometry>
        <cylinder radius="${arm_radius}" length="${arm_length}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${arm_length/2}"/>
      <geometry>
        <cylinder radius="${arm_radius}" length="${arm_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 ${arm_length/2}"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="${-torso_radius} 0 ${torso_height*0.7}"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="50" velocity="1"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <origin xyz="0 0 ${forearm_length/2}"/>
      <geometry>
        <cylinder radius="${forearm_radius}" length="${forearm_length}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${forearm_length/2}"/>
      <geometry>
        <cylinder radius="${forearm_radius}" length="${forearm_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 ${forearm_length/2}"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 ${arm_length}"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="30" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_thigh">
    <visual>
      <origin xyz="0 0 ${leg_length/2}"/>
      <geometry>
        <cylinder radius="${leg_radius}" length="${leg_length}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${leg_length/2}"/>
      <geometry>
        <cylinder radius="${leg_radius}" length="${leg_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 ${leg_length/2}"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="${torso_radius/2} 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
  </joint>

  <link name="left_shin">
    <visual>
      <origin xyz="0 0 ${shin_length/2}"/>
      <geometry>
        <cylinder radius="${shin_radius}" length="${shin_length}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${shin_length/2}"/>
      <geometry>
        <cylinder radius="${shin_radius}" length="${shin_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 ${shin_length/2}"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.008"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 ${leg_length}"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_thigh">
    <visual>
      <origin xyz="0 0 ${leg_length/2}"/>
      <geometry>
        <cylinder radius="${leg_radius}" length="${leg_length}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${leg_length/2}"/>
      <geometry>
        <cylinder radius="${leg_radius}" length="${leg_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 ${leg_length/2}"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="${-torso_radius/2} 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
  </joint>

  <link name="right_shin">
    <visual>
      <origin xyz="0 0 ${shin_length/2}"/>
      <geometry>
        <cylinder radius="${shin_radius}" length="${shin_length}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${shin_length/2}"/>
      <geometry>
        <cylinder radius="${shin_radius}" length="${shin_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 ${shin_length/2}"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.008"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 ${leg_length}"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
  </joint>

</robot>
```

## Key Humanoid Design Considerations

### Joint Configuration
- **Revolute joints** provide rotational movement around a single axis
- **Limit constraints** prevent damage and ensure realistic movement
- **Effort and velocity limits** reflect real actuator capabilities

### Kinematic Chains
- **Right and left limbs** mirror each other for balance
- **Torso as central hub** connects all major components
- **Proper joint orientation** ensures natural movement patterns

### Inertial Properties
- **Mass values** should reflect realistic estimates
- **Inertia tensors** affect simulation stability
- **Center of mass** placement is critical for balance

## Testing URDF Files

To test your URDF file:

1. Validate the syntax:
```bash
check_urdf /path/to/your/robot.urdf
```

2. Visualize in RViz:
```bash
ros2 run rviz2 rviz2
```

3. Use joint state publisher for visualization:
```bash
ros2 run joint_state_publisher joint_state_publisher
```

4. Test in Gazebo simulation:
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

## Common Humanoid URDF Issues

1. **Self-collision**: Ensure proper spacing between links
2. **Inertial errors**: Use realistic mass and inertia values
3. **Joint limits**: Set appropriate ranges to prevent damage
4. **Mesh resolution**: Balance detail with performance
5. **Frame alignment**: Ensure proper TF tree structure