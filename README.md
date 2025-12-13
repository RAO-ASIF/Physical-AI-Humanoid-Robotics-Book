# Physical AI & Humanoid Robotics System

This project implements a comprehensive Physical AI & Humanoid Robotics system with simulation-first approach, extensible to real hardware. The system integrates ROS 2, perception, navigation, and Vision-Language-Action (VLA) capabilities to create an intelligent humanoid robot that can understand voice commands, navigate environments, and perform tasks.

## Features

- Full ROS 2 integration for robot communication and control
- Perception system for object detection and environment understanding
- Navigation system with path planning and obstacle avoidance
- Vision-Language-Action (VLA) pipeline for voice command processing
- Simulation-ready architecture using Gazebo/Unity
- Capstone project integrating all modules into a complete humanoid robot system

## Architecture

The system consists of:

- `src/capstone-project/complete_implementation/integrated_robot_system.py` - Main integrated robot system
- `src/ros2-examples/` - ROS 2 communication examples and nodes
- `src/simulation-scenes/` - Simulation environments and robot models
- `src/ai-pipelines/` - AI integration modules (perception, navigation, voice control)
- `docs/` - Comprehensive documentation for all modules

## Prerequisites

- Python 3.10+ with ROS 2 Humble/Iron
- Robot Operating System (ROS 2) installed
- rclpy Python package
- Additional dependencies for simulation (Gazebo/Unity) and AI processing

## Setup

1. Install ROS 2 (Humble Hawksbill or Iron Irwini)
2. Set up Python environment with required packages:
   ```bash
   pip install rclpy numpy
   ```

3. For development documentation (optional):
   ```bash
   npm install
   ```

## Usage

### Launch the Integrated Robot System

```bash
npm start
```

Or directly run the Python script:
```bash
python src/capstone-project/complete_implementation/integrated_robot_system.py
```

### Core Modules

The system includes four core modules:

1. **Robotic Nervous System (ROS 2)** - Communication backbone
2. **Digital Twin (Gazebo & Unity)** - Simulation environments
3. **AI-Robot Brain (NVIDIA Isaac)** - Perception and planning
4. **Vision-Language-Action (VLA)** - Voice command processing

### Example Voice Commands

- "Go to kitchen" - Navigate to a specific location
- "Find person" - Search for specific objects/people
- "Help" - Request assistance
- "Stop" - Emergency stop

## Scripts

- `npm start` - Launch the integrated humanoid robot system
- `npm run docs:build` - Generate documentation (developer tool)
- `npm run docs:generate` - Generate specific documentation content (developer tool)

## System Components

### Integrated Robot System
- State management (IDLE, LISTENING, PROCESSING, NAVIGATING, etc.)
- Voice command processing
- Perception pipeline integration
- Navigation and path planning
- Safety and obstacle detection

### ROS 2 Integration
- Publisher/subscriber communication patterns
- Service-based interactions
- Robot URDF models
- Sensor data processing

### AI Pipelines
- Perception system for object detection
- Navigation pipeline with path planning
- Voice control pipeline for VLA processing
- AI model integration examples

## File Structure

The system follows a modular architecture:

```
src/                           # Source code
├── ros2-examples/            # ROS 2 communication examples
├── simulation-scenes/        # Simulation environments
│   ├── gazebo_worlds/        # Gazebo simulation files
│   └── unity_scenes/         # Unity simulation files
├── ai-pipelines/             # AI integration modules
│   ├── perception/           # Object detection and perception
│   ├── navigation/           # Path planning and navigation
│   └── voice_control/        # Voice command processing
└── capstone-project/         # Integrated system implementation
    └── complete_implementation/
        └── integrated_robot_system.py
docs/                         # Educational documentation
├── physical-ai/              # Physical AI fundamentals
├── ros2/                     # ROS 2 module documentation
├── simulation/               # Simulation environment docs
├── ai-robot-brain/           # AI integration documentation
├── vla/                      # VLA system documentation
└── capstone/                 # Capstone project documentation
static/                       # Static assets and models
```

## Safety Features

- Emergency stop functionality
- Obstacle detection and avoidance
- State-based safety checks
- Distance thresholds for navigation

## Development

This project was developed following Spec-Driven Development principles with comprehensive documentation covering all aspects of humanoid robotics with AI integration. The system is designed for university students, researchers, and educators in AI and robotics.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes to the Python robotics system
4. Test in simulation environment
5. Submit a pull request

## License

MIT License - see the LICENSE file for details.