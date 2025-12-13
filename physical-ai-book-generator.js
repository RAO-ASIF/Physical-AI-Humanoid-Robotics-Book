// physical-ai-book-generator.js
// Generator for Physical AI & Humanoid Robotics Capstone Book using Docusaurus 3.9.2 and Context7 MCP

const DocusaurusMCPIntegration = require('./mcp-integration');

// Configuration for the Physical AI & Humanoid Robotics book
const bookConfig = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Embodied Intelligence',
  favicon: 'img/favicon.ico',
  url: 'https://your-project.github.io',
  baseUrl: '/physical-ai-humanoid-robotics/',
  organizationName: 'your-organization',
  projectName: 'physical-ai-humanoid-robotics',
  navbarTitle: 'Physical AI Book',
  logoAlt: 'Physical AI & Humanoid Robotics Logo',
  logoSrc: 'img/logo.svg',
  navbarLabel: 'Book Chapters'
};

// Initialize the MCP integration
const mcpIntegration = new DocusaurusMCPIntegration();

// Generate the complete Physical AI & Humanoid Robotics book
async function generatePhysicalAIBook() {
  try {
    console.log('Starting Physical AI & Humanoid Robotics book generation...');

    // Generate the main site structure
    const result = await mcpIntegration.generateDocusaurusSite(bookConfig);
    console.log('Site generation result:', result);

    // Generate Module 1: Robotic Nervous System (ROS 2)
    await generateROS2Module();

    // Generate Module 2: Digital Twin (Gazebo & Unity)
    await generateDigitalTwinModule();

    // Generate Module 3: AI-Robot Brain (NVIDIA Isaac)
    await generateAIRobotBrainModule();

    // Generate Module 4: Vision-Language-Action (VLA)
    await generateVLAModule();

    // Generate Capstone Project
    await generateCapstoneModule();

    // Generate References and Citations
    await generateReferences();

    console.log('Physical AI & Humanoid Robotics book generated successfully!');
  } catch (error) {
    console.error('Error generating Physical AI book:', error);
  }
}

// Generate Module 1: Robotic Nervous System (ROS 2)
async function generateROS2Module() {
  console.log('Generating ROS 2 module...');

  // Create module index
  await mcpIntegration.generateDocumentationContent(
    'Module 1: Robotic Nervous System (ROS 2)',
    `# Module 1: Robotic Nervous System (ROS 2)

This module covers the foundational concepts of ROS 2 and how they apply to humanoid robotics, including nodes, topics, services, URDF, and Python integration.

## Learning Objectives
- Understand the core concepts of ROS 2
- Learn to create ROS 2 nodes and handle topics/services
- Master URDF robot modeling
- Implement Python integration with rclpy

## Table of Contents
1. [Introduction to ROS 2 and Physical AI](./introduction)
2. [Nodes, Topics, and Services](./nodes-topics-services)
3. [URDF Robot Modeling](./urdf-modeling)
4. [Python Integration with rclpy](./python-integration)
5. [Practical ROS 2 Examples](./practical-examples)

## Prerequisites
- Basic Python programming knowledge
- Understanding of robotics fundamentals`,
    'ros2',
    1
  );

  // Create individual sections for ROS 2 module
  await mcpIntegration.generateDocumentationContent(
    'Introduction to ROS 2 and Physical AI',
    `# Introduction to ROS 2 and Physical AI

## Overview
ROS 2 (Robot Operating System 2) serves as the nervous system for humanoid robots, providing the communication backbone that enables different components to work together seamlessly.

## Key Concepts
- **Nodes**: Independent processes that perform specific functions
- **Topics**: Unidirectional communication channels
- **Services**: Bidirectional request/response communication
- **Actions**: Asynchronous communication for long-running tasks

## Why ROS 2 for Physical AI?
ROS 2 provides the infrastructure needed to connect sensors, actuators, and AI systems in a humanoid robot. It handles the complex task of inter-process communication, allowing developers to focus on creating intelligent behaviors rather than managing communication protocols.

## ROS 2 Distributions
For this book, we'll be using ROS 2 Humble Hawksbill, which is an LTS (Long Term Support) version with extensive documentation and community support.

## Getting Started
In the next section, we'll explore the fundamental concepts of nodes, topics, and services that form the basis of ROS 2 communication.`,
    'ros2',
    2
  );

  await mcpIntegration.generateDocumentationContent(
    'Nodes, Topics, and Services',
    `# Nodes, Topics, and Services

## Nodes
Nodes are the fundamental building blocks of ROS 2. Each node performs a specific function and communicates with other nodes through topics, services, and actions.

### Creating a Simple Node
\`\`\`python
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
\`\`\`

## Topics
Topics enable unidirectional, asynchronous communication between nodes using a publish/subscribe pattern.

### Publisher-Subscriber Pattern
- Publishers send messages to topics
- Subscribers receive messages from topics
- Multiple publishers and subscribers can use the same topic

## Services
Services provide bidirectional, synchronous communication using a request/response pattern.

### Service Server and Client
- Service servers provide specific functionality
- Service clients request that functionality
- Communication is synchronous (client waits for response)

## Practical Exercise
Create a simple publisher and subscriber pair that communicates robot sensor data.`,
    'ros2',
    3
  );

  await mcpIntegration.generateDocumentationContent(
    'URDF Robot Modeling',
    `# URDF Robot Modeling

## What is URDF?
URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including links, joints, and materials.

## Basic URDF Structure
\`\`\`xml
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
</robot>
\`\`\`

## Links and Joints
- **Links**: Rigid bodies with visual, collision, and inertial properties
- **Joints**: Connections between links that allow relative motion
- **Joint Types**: Fixed, revolute, continuous, prismatic, floating, planar

## Kinematic Chains
URDF models can represent complex kinematic chains for robotic arms, legs, or full humanoid bodies.

## Best Practices
- Use consistent naming conventions
- Include proper inertial properties for simulation
- Validate URDF files before use
- Separate complex models into multiple files`,
    'ros2',
    4
  );

  await mcpIntegration.generateDocumentationContent(
    'Python Integration with rclpy',
    `# Python Integration with rclpy

## rclpy Overview
rclpy is the Python client library for ROS 2. It provides a Python API for creating ROS 2 nodes, publishers, subscribers, services, and actions.

## Key Components
- **Node**: The basic execution unit
- **Publisher**: Sends messages to topics
- **Subscriber**: Receives messages from topics
- **Service Server**: Provides service functionality
- **Service Client**: Requests service functionality

## Creating a Publisher Node
\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
\`\`\`

## Creating a Subscriber Node
\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
\`\`\`

## Parameters
Nodes can use parameters to configure behavior at runtime.

## Launch Files
Launch files allow you to start multiple nodes simultaneously with specific configurations.`,
    'ros2',
    5
  );

  await mcpIntegration.generateDocumentationContent(
    'Practical ROS 2 Examples',
    `# Practical ROS 2 Examples

## Example 1: Simple Publisher-Subscriber
This example demonstrates the basic publisher-subscriber pattern for robot communication.

## Example 2: Service-Based Robot Control
Using services to control robot actions with request/response communication.

## Example 3: Action-Based Navigation
Implementing complex navigation tasks using ROS 2 actions for long-running operations.

## Example 4: Parameter-Based Configuration
Using parameters to configure robot behavior at runtime.

## Example 5: URDF Integration
Combining URDF robot models with ROS 2 nodes for complete robot simulation.

## Running the Examples
\`\`\`bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select robot_examples

# Source the workspace
source install/setup.bash

# Run a publisher node
ros2 run robot_examples simple_publisher

# In another terminal, run a subscriber node
ros2 run robot_examples simple_subscriber
\`\`\`

## Troubleshooting Common Issues
- Environment setup problems
- Node communication issues
- Package build errors
- Topic/service connection problems`,
    'ros2',
    6
  );

  console.log('ROS 2 module generated');
}

// Generate Module 2: Digital Twin (Gazebo & Unity)
async function generateDigitalTwinModule() {
  console.log('Generating Digital Twin module...');

  await mcpIntegration.generateDocumentationContent(
    'Module 2: Digital Twin (Gazebo & Unity)',
    `# Module 2: Digital Twin (Gazebo & Unity)

This module covers the implementation of digital twins using Gazebo and Unity for humanoid robot simulation, including physics simulation, sensors, and environment modeling.

## Learning Objectives
- Understand digital twin concepts in robotics
- Learn Gazebo physics simulation
- Explore Unity environment modeling
- Implement sensor integration in simulation
- Create practical simulation examples

## Table of Contents
1. [Introduction to Digital Twins](./introduction)
2. [Gazebo Physics Simulation](./gazebo-basics)
3. [Unity Environment Modeling](./unity-integration)
4. [Sensor Integration in Simulation](./sensor-integration)
5. [Practical Simulation Examples](./practical-examples)

## Prerequisites
- Basic understanding of ROS 2 concepts
- Familiarity with 3D modeling concepts`,
    'simulation',
    1
  );

  // Add more content for the simulation module...
  await mcpIntegration.generateDocumentationContent(
    'Introduction to Digital Twins',
    `# Introduction to Digital Twins

## What is a Digital Twin?
A digital twin is a virtual representation of a physical system that simulates its behavior and characteristics in real-time. In robotics, digital twins enable:

- Safe testing of algorithms without physical hardware
- Prototyping and validation of robot behaviors
- Training of AI systems with synthetic data
- Optimization of robot performance

## Digital Twins in Robotics
Digital twins are crucial for robotics development because they allow:

- **Risk Reduction**: Test dangerous or complex behaviors safely
- **Cost Savings**: Reduce need for physical prototypes
- **Faster Iteration**: Quick testing and refinement of algorithms
- **Data Generation**: Create large datasets for AI training

## Simulation vs Reality
While digital twins aim to accurately represent physical systems, there's always a "reality gap" that must be considered. Techniques like domain randomization help bridge this gap.

## Simulation Platforms for Robotics
- **Gazebo**: Physics-accurate simulation with extensive robotics tools
- **Unity**: High-quality graphics and user interaction capabilities
- **PyBullet**: Fast physics simulation with Python API
- **Webots**: Complete robot simulation environment

## The Simulation Pipeline
1. Robot modeling (URDF/SDF)
2. Physics engine configuration
3. Sensor modeling
4. Environment creation
5. Control algorithm implementation
6. Data collection and analysis`,
    'simulation',
    2
  );

  await mcpIntegration.generateDocumentationContent(
    'Gazebo Physics Simulation',
    `# Gazebo Physics Simulation

## Gazebo Overview
Gazebo is a 3D simulation environment that enables accurate and efficient testing of robotics algorithms. It provides:

- High-fidelity physics simulation
- Quality graphics rendering
- Multiple physics engines
- Sensor simulation
- Realistic environments

## Gazebo Architecture
- **Server**: Handles simulation and physics calculations
- **Client**: Provides visualization interface
- **Plugins**: Extend functionality for custom needs

## Physics Engines
Gazebo supports multiple physics engines:
- **ODE**: Open Dynamics Engine (default)
- **Bullet**: Fast and robust
- **SimBody**: Biomechanics-focused
- **DART**: Dynamic Animation and Robotics Toolkit

## Creating Gazebo Worlds
\`\`\`xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.6 0.4 -0.8</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
\`\`\`

## Integrating with ROS 2
Gazebo integrates seamlessly with ROS 2 through:
- **ros_gz**: ROS 2 to Gazebo bridge
- **Gazebo plugins**: Custom ROS 2 interfaces
- **URDF/SDF conversion**: Robot model compatibility`,
    'simulation',
    3
  );

  console.log('Digital Twin module generated (partial)');
}

// Generate Module 3: AI-Robot Brain (NVIDIA Isaac)
async function generateAIRobotBrainModule() {
  console.log('Generating AI-Robot Brain module...');

  await mcpIntegration.generateDocumentationContent(
    'Module 3: AI-Robot Brain (NVIDIA Isaac)',
    `# Module 3: AI-Robot Brain (NVIDIA Isaac)

This module covers AI integration in robotics using NVIDIA Isaac, including perception, VSLAM, Nav2 path planning, and practical AI integration examples.

## Learning Objectives
- Understand perception systems in robotics
- Learn VSLAM (Visual Simultaneous Localization and Mapping)
- Implement Nav2 path planning
- Create practical AI integration examples

## Table of Contents
1. [Introduction to AI in Robotics](./introduction)
2. [Perception Systems](./perception)
3. [VSLAM (Visual SLAM)](./vslam)
4. [Nav2 Path Planning](./nav2-planning)
5. [Practical AI Integration Examples](./practical-examples)

## Prerequisites
- Basic understanding of machine learning concepts
- ROS 2 and simulation knowledge`,
    'ai-robot-brain',
    1
  );

  console.log('AI-Robot Brain module generated (partial)');
}

// Generate Module 4: Vision-Language-Action (VLA)
async function generateVLAModule() {
  console.log('Generating VLA module...');

  await mcpIntegration.generateDocumentationContent(
    'Module 4: Vision-Language-Action (VLA)',
    `# Module 4: Vision-Language-Action (VLA)

This module covers Vision-Language-Action systems for robotics, including voice command processing, cognitive planning, LLM integration, and practical VLA examples.

## Learning Objectives
- Understand VLA architecture components
- Implement voice command processing
- Create cognitive planning systems
- Integrate LLMs with robotics

## Table of Contents
1. [Introduction to VLA Systems](./introduction)
2. [Voice Command Processing](./voice-processing)
3. [Cognitive Planning](./cognitive-planning)
4. [LLM Integration](./llm-integration)
5. [Practical VLA Examples](./practical-examples)

## Prerequisites
- Understanding of natural language processing
- AI and robotics integration knowledge`,
    'vla',
    1
  );

  console.log('VLA module generated (partial)');
}

// Generate Capstone Project
async function generateCapstoneModule() {
  console.log('Generating Capstone module...');

  await mcpIntegration.generateDocumentationContent(
    'Capstone: Autonomous Humanoid Robot',
    `# Capstone: Autonomous Humanoid Robot

This capstone project integrates all modules to create an autonomous humanoid robot capable of receiving voice commands, perceiving objects, planning actions, navigating, and manipulating objects.

## Learning Objectives
- Integrate all previous modules into a complete system
- Implement a voice-to-action pipeline
- Combine perception, navigation, and manipulation
- Create a complete humanoid robot system

## Project Overview
The capstone project involves creating a complete humanoid robot system that can:
1. Receive and interpret voice commands
2. Perceive and identify objects in the environment
3. Plan navigation and manipulation actions
4. Execute complex multi-step tasks

## Architecture
The system integrates:
- **ROS 2**: Communication backbone
- **Simulation**: Gazebo for safe testing
- **AI Perception**: Object detection and recognition
- **Navigation**: Path planning and obstacle avoidance
- **VLA**: Voice command processing

## Implementation Steps
1. System architecture design
2. Component integration
3. Testing and validation
4. Performance optimization

## Assessment Criteria
- Successful completion of voice command execution
- Accurate object perception and identification
- Safe and efficient navigation
- Successful manipulation of objects`,
    'capstone',
    1
  );

  console.log('Capstone module generated');
}

// Generate References
async function generateReferences() {
  console.log('Generating References...');

  await mcpIntegration.generateDocumentationContent(
    'References and Citations',
    `# References and Citations

## Academic Sources
1. Quigley, M., et al. (2009). ROS: an open-source Robot Operating System. ICRA Workshop on Open Source Software, 3(3), 5.

2. Cole, D., et al. (2018). Gazebo: A 3D Multi-Robot Simulator. Gazebo Documentation. http://gazebosim.org/tutorials

3. NVIDIA Isaac. (2023). NVIDIA Isaac Sim Documentation. https://nvidia-isaac-sim.github.io/

4. ROS 2 Documentation. (2023). ROS 2 Humble Hawksbill Documentation. https://docs.ros.org/en/humble/

5. Fox, D., Burgard, W., & Thrun, S. (1997). The dynamic window approach to collision avoidance. IEEE Robotics & Automation Magazine, 4(1), 23-33.

## Technical Documentation
- ROS 2 Concepts: https://docs.ros.org/en/humble/Concepts.html
- Gazebo Tutorials: http://gazebosim.org/tutorials
- URDF/XML Format: http://wiki.ros.org/urdf/XML
- Nav2 Navigation System: https://navigation.ros.org/

## Additional Resources
- IEEE Xplore Digital Library: https://ieeexplore.ieee.org/
- ACM Digital Library: https://dl.acm.org/
- arXiv Robotics Papers: https://arxiv.org/list/cs.RO/recent`,
    'references',
    1
  );

  console.log('References generated');
}

// Run the book generation
generatePhysicalAIBook();

module.exports = {
  generatePhysicalAIBook,
  generateROS2Module,
  generateDigitalTwinModule,
  generateAIRobotBrainModule,
  generateVLAModule,
  generateCapstoneModule,
  generateReferences
};