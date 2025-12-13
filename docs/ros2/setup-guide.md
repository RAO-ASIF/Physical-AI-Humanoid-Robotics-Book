---
title: ROS 2 Setup Guide for Humanoid Robotics
sidebar_position: 6
---

# ROS 2 Setup Guide for Humanoid Robotics

This guide provides step-by-step instructions for setting up ROS 2 in the context of humanoid robotics development. Following this guide will prepare your environment for the examples and exercises in this module.

## Prerequisites

Before beginning the ROS 2 setup, ensure you have:

- Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- 16GB+ RAM (32GB recommended for simulation)
- Python 3.10 or higher
- Git installed
- Administrative access for system installations

## System Requirements

### Hardware Requirements
- **CPU**: 8+ cores, modern architecture (Intel i7 or AMD Ryzen equivalent)
- **RAM**: 16GB minimum, 32GB recommended for simulation work
- **Storage**: 50GB+ free space for ROS 2 and dependencies
- **Network**: High-speed internet for initial downloads

### Software Requirements
- **OS**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **ROS 2**: Humble Hawksbill (LTS version) - this guide focuses on this version
- **Python**: 3.10 or higher

## Installing ROS 2 Humble Hawksbill

### On Ubuntu 22.04

1. **Set up the ROS 2 repository**:
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

2. **Install ROS 2 packages**:
```bash
sudo apt update
sudo apt install -y ros-humble-desktop-full
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
```

3. **Initialize rosdep**:
```bash
sudo rosdep init
rosdep update
```

4. **Source ROS 2** (add to your `~/.bashrc` for permanent access):
```bash
source /opt/ros/humble/setup.bash
```

### On Windows with WSL2

1. Install WSL2 with Ubuntu 22.04
2. Follow the Ubuntu 22.04 instructions above within WSL2

## Setting Up Your Workspace

### 1. Create a Workspace Directory
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Source ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
```

### 3. Build the Workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

## Essential ROS 2 Tools for Humanoid Robotics

### 1. Install Additional Packages
```bash
sudo apt install -y ros-humble-ros2-control* ros-humble-ros2-controllers*
sudo apt install -y ros-humble-gazebo-ros2-control
sudo apt install -y ros-humble-moveit
```

### 2. Install Python Dependencies
```bash
pip3 install rclpy transforms3d numpy matplotlib opencv-python
```

## Testing Your Installation

### 1. Verify ROS 2 Installation
```bash
ros2 --version
```

### 2. Test Basic Communication
```bash
# Terminal 1: Start a publisher
source /opt/ros/humble/setup.bash
ros2 topic pub /chatter std_msgs/String "data: Hello ROS2"

# Terminal 2: Listen to the topic
source /opt/ros/humble/setup.bash
ros2 topic echo /chatter std_msgs/String
```

### 3. Check Available Packages
```bash
ros2 pkg list | grep -i ros
```

## ROS 2 for Humanoid Robotics Specific Setup

### 1. Install Humanoid-Specific Packages
```bash
sudo apt install -y ros-humble-joint-state-publisher-gui
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-xacro
```

### 2. Create a Humanoid Robot Package Template
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python humanoid_robot_base
```

## Troubleshooting Common Issues

### Issue: "command not found" after installation
**Solution**: Ensure you've sourced the ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

### Issue: Python import errors
**Solution**: Check your Python path and ensure ROS 2 is properly sourced:
```bash
echo $PYTHONPATH
source /opt/ros/humble/setup.bash
```

### Issue: Permission errors during installation
**Solution**: Use `sudo` for system package installations, but avoid `sudo` for workspace builds.

## Next Steps

After completing this setup guide, you should be able to:

1. Source your ROS 2 environment successfully
2. Run basic ROS 2 commands
3. Create and build a simple ROS 2 package
4. Run the examples in the subsequent sections of this module

## Verification Checklist

- [ ] ROS 2 Humble Hawksbill installed successfully
- [ ] Workspace created and built without errors
- [ ] Basic ROS 2 commands work (ros2, ros2 run, etc.)
- [ ] Python packages for robotics installed
- [ ] Humanoid-specific packages installed
- [ ] Environment variables properly set

Once you've completed this setup and verified all components are working, you're ready to proceed with the ROS 2 fundamentals examples in the next section.