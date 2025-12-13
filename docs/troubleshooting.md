---
title: Troubleshooting Guide
sidebar_label: Troubleshooting
description: Common issues and solutions for Physical AI and Humanoid Robotics development
---

# Troubleshooting Guide

This guide provides solutions to common issues encountered when working with the Physical AI and Humanoid Robotics textbook content.

## ROS 2 Common Issues

### 1. Node Communication Problems

**Issue**: Nodes cannot communicate with each other
- **Solution**: Check that nodes are on the same ROS domain ID
  ```bash
  # Check current domain ID
  echo $ROS_DOMAIN_ID

  # Set domain ID (0-101) for this session
  export ROS_DOMAIN_ID=42
  ```

**Issue**: Publisher/subscriber topics not connecting
- **Solution**: Verify topic names match exactly (including case)
  ```bash
  # List active topics
  ros2 topic list

  # Echo a topic to verify it's publishing
  ros2 topic echo /topic_name std_msgs/msg/String
  ```

### 2. Installation and Setup Issues

**Issue**: ROS 2 packages not found
- **Solution**: Source the ROS 2 setup script
  ```bash
  source /opt/ros/humble/setup.bash
  # Or for other distributions:
  # source /opt/ros/iron/setup.bash
  # source /opt/ros/rolling/setup.bash
  ```

**Issue**: Python modules not found (rclpy, etc.)
- **Solution**: Install ROS 2 Python packages
  ```bash
  sudo apt update
  sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
  sudo apt install python3-ros-humble-*
  ```

### 3. Permission and Environment Issues

**Issue**: Permission denied errors
- **Solution**: Ensure proper workspace permissions
  ```bash
  # Fix permissions on workspace
  sudo chown -R $USER:$USER ~/ros2_ws
  ```

## Gazebo Simulation Issues

### 1. Gazebo Won't Start

**Issue**: Gazebo fails to launch
- **Solution**: Check for graphics driver compatibility
  ```bash
  # Test graphics rendering
  glxinfo | grep "OpenGL renderer"

  # If using virtual machine, enable 3D acceleration
  # For NVIDIA GPUs, ensure proper drivers are installed
  ```

### 2. Robot Model Issues

**Issue**: Robot falls through the ground or behaves unexpectedly
- **Solution**: Check URDF inertial properties
  - Verify mass values are realistic (not 0 or negative)
  - Check that inertia tensors are properly calculated
  - Ensure collision geometries match visual geometries

**Issue**: Joints don't move as expected
- **Solution**: Verify joint limits and effort values in URDF
  ```xml
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  ```

## Perception System Issues

### 1. Camera/Visual Perception Problems

**Issue**: Camera not publishing images
- **Solution**: Check camera plugin configuration in URDF/SDF
  ```xml
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
  </sensor>
  ```

### 2. Object Detection Issues

**Issue**: Poor detection accuracy
- **Solution**:
  - Check lighting conditions in simulation
  - Verify camera calibration parameters
  - Adjust detection thresholds in perception pipeline

## AI Integration Issues

### 1. NVIDIA Isaac ROS Issues

**Issue**: Isaac ROS packages not found
- **Solution**: Install Isaac ROS packages
  ```bash
  sudo apt install ros-humble-isaac-ros-*
  ```

**Issue**: GPU acceleration not working
- **Solution**: Verify CUDA installation and GPU compatibility
  ```bash
  nvidia-smi
  nvcc --version
  ```

### 2. Navigation System Issues

**Issue**: Nav2 fails to plan paths
- **Solution**: Check costmap configuration and map availability
  ```bash
  # Verify map server is running
  ros2 run nav2_map_server map_server
  ```

## Voice Recognition Issues

### 1. Whisper Integration Problems

**Issue**: Audio not being processed
- **Solution**: Check audio input device and permissions
  ```bash
  # List audio devices
  arecord -l
  ```

**Issue**: Poor recognition accuracy
- **Solution**:
  - Use a high-quality microphone
  - Ensure quiet environment
  - Check audio format (WAV, 16kHz recommended)

## Development Environment Issues

### 1. Workspace Build Issues

**Issue**: colcon build fails
- **Solution**: Check package.xml dependencies and CMakeLists.txt
  ```bash
  # Clean build and rebuild
  rm -rf build/ install/ log/
  colcon build --packages-select package_name
  ```

### 2. Python Virtual Environment Issues

**Issue**: Python packages conflict
- **Solution**: Use virtual environments for ROS 2 projects
  ```bash
  python3 -m venv ros2_env
  source ros2_env/bin/activate
  pip install --upgrade pip
  ```

## Network and Performance Issues

### 1. Communication Delays

**Issue**: High latency in ROS 2 communication
- **Solution**: Optimize QoS settings for your use case
  ```python
  from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

  qos = QoSProfile(
      depth=10,
      reliability=ReliabilityPolicy.BEST_EFFORT,  # For sensor data
      durability=DurabilityPolicy.VOLATILE
  )
  ```

### 2. High CPU/GPU Usage

**Issue**: System resources are maxed out
- **Solution**:
  - Reduce simulation update rates
  - Lower rendering quality in Gazebo
  - Optimize perception pipeline complexity

## Common Error Messages

### 1. "Failed to create subscriber/publisher"
- **Cause**: Node not properly initialized
- **Solution**: Ensure `rclpy.init()` is called before creating subscribers/publishers

### 2. "Could not find a free port"
- **Cause**: Too many ROS 2 processes running
- **Solution**: Kill all ROS 2 processes and restart
  ```bash
  pkill -f ros
  # Or more specifically:
  pkill -f ros2
  ```

### 3. "Package not found"
- **Cause**: Package not built or not sourced
- **Solution**:
  ```bash
  cd ~/ros2_ws
  colcon build
  source install/setup.bash
  ```

## Getting Help

If you encounter issues not covered in this guide:

1. Check the official documentation:
   - [ROS 2 Documentation](https://docs.ros.org/)
   - [Gazebo Documentation](http://gazebosim.org/)
   - [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/)

2. Search the community:
   - [ROS Answers](https://answers.ros.org/)
   - [Gazebo Answers](http://answers.gazebosim.org/)
   - [Robotics Stack Exchange](https://robotics.stackexchange.com/)

3. Check the project repository for updates and known issues.