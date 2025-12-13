# Quickstart Guide: Physical AI & Humanoid Robotics Capstone Book

## Overview
This quickstart guide provides a rapid path to get started with the Physical AI & Humanoid Robotics Capstone Book. It covers the essential setup steps and a simple example to demonstrate the core concepts.

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **RAM**: 16GB minimum, 32GB recommended
- **CPU**: 8+ cores, modern architecture (Intel i7 or AMD Ryzen equivalent)
- **GPU**: NVIDIA GPU with 8GB+ VRAM (for Isaac Sim), integrated graphics for basic Gazebo
- **Storage**: 100GB+ free space
- **Network**: High-speed internet for downloading large simulation environments

### Software Requirements
- **ROS 2**: Humble Hawksbill (LTS version)
- **Python**: 3.10 or higher
- **Docker**: For containerized development (optional but recommended)
- **Git**: For version control and code examples

## Environment Setup

### 1. ROS 2 Installation (Ubuntu 22.04)
```bash
# Set up ROS 2 repositories
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop-full
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Workspace Setup
```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

### 3. Simulation Environment Setup

#### Option A: Gazebo (Open Source)
```bash
# Install Gazebo Garden
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo

# Verify installation
gz sim --version
```

#### Option B: Isaac Sim (NVIDIA - Requires Account)
```bash
# Download Isaac Sim from NVIDIA Developer website
# Follow NVIDIA's installation guide for Isaac Sim
# Ensure GPU drivers and CUDA are properly configured
```

### 4. Python Dependencies
```bash
# Install Python packages for the book examples
pip3 install rclpy transforms3d numpy matplotlib opencv-python openai pygame
```

## First Example: Simple ROS 2 Publisher-Subscriber

### 1. Create a Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_basics
cd robot_basics
```

### 2. Create Publisher Node
Create `robot_basics/robot_basics/simple_publisher.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello Robot: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Create Subscriber Node
Create `robot_basics/robot_basics/simple_subscriber.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Update Package Configuration
Edit `robot_basics/setup.py`:
```python
from setuptools import find_packages, setup

package_name = 'robot_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Simple ROS 2 publisher and subscriber example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = robot_basics.simple_publisher:main',
            'simple_subscriber = robot_basics.simple_subscriber:main',
        ],
    },
)
```

### 5. Build and Run
```bash
# From the workspace root
cd ~/ros2_ws
colcon build --packages-select robot_basics
source install/setup.bash

# Terminal 1: Run the publisher
ros2 run robot_basics simple_publisher

# Terminal 2: Run the subscriber (in a new terminal)
source ~/ros2_ws/install/setup.bash
ros2 run robot_basics simple_subscriber
```

## First Simulation: Basic Robot in Gazebo

### 1. Create a Simple Robot Model
Create `~/ros2_ws/src/robot_basics/urdf/simple_robot.urdf`:
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <link name="sensor_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="sensor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
    <origin xyz="0.3 0 0"/>
  </joint>
</robot>
```

### 2. Launch Gazebo with the Robot
Create `~/ros2_ws/src/robot_basics/launch/simple_robot.launch.py`:
```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('robot_basics'),
        'urdf',
        'simple_robot.urdf'
    )

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v4', 'empty.sdf'],
            output='screen'
        ),

        # Spawn the robot in Gazebo
        Node(
            package='ros_gz',
            executable='parameter_bridge',
            arguments=['/model/simple_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        )
    ])
```

## Running the Book Examples

### 1. Navigate to the Book Repository
```bash
cd ~/physical_ai_book
```

### 2. Set up the Docusaurus Environment
```bash
# Install Node.js dependencies
npm install

# Start the development server
npm start
```

### 3. Access the Book
Open your browser to `http://localhost:3000` to access the book content.

## Module-Specific Quickstarts

### Module 1: ROS 2 Quickstart
- Start with the publisher-subscriber example above
- Progress to service calls and action servers
- Learn about parameters and launch files
- Practice with URDF robot modeling

### Module 2: Simulation Quickstart
- Set up Gazebo with basic world files
- Load your URDF robot into simulation
- Add sensors to your robot model
- Control the robot in simulation

### Module 3: AI Integration Quickstart
- Install Isaac ROS packages
- Set up perception pipelines
- Configure navigation systems
- Test VSLAM algorithms

### Module 4: VLA Quickstart
- Set up speech-to-text capabilities
- Integrate LLMs for command interpretation
- Create action planning systems
- Connect VLA to robot control

## Troubleshooting

### Common Issues and Solutions

#### ROS 2 Installation Issues
- **Issue**: Package not found during installation
- **Solution**: Ensure your locale is set correctly: `export LANG=C`

#### Simulation Issues
- **Issue**: Gazebo fails to start with graphics errors
- **Solution**: Check GPU drivers and run with software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`

#### Python Import Issues
- **Issue**: Module not found errors
- **Solution**: Ensure you've sourced the ROS 2 environment: `source ~/ros2_ws/install/setup.bash`

#### Network Issues
- **Issue**: Cannot access external APIs for AI models
- **Solution**: Check firewall settings and proxy configurations

## Next Steps

1. Complete the full setup process described above
2. Run the simple publisher-subscriber example
3. Progress through Module 1: ROS 2 fundamentals
4. Set up simulation environment for Module 2
5. Explore the book content at `http://localhost:3000`
6. Follow the progressive learning path from beginner to system integrator level

## Getting Help

- Check the book's reference section for detailed explanations
- Visit the ROS Discourse forum for ROS 2 issues
- Use the book's GitHub repository for errata and updates
- Join the robotics community forums for additional support