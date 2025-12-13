// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Physical AI Fundamentals',
      link: {
        type: 'doc',
        id: 'physical-ai/index',
      },
      items: [
        'physical-ai/fundamentals',
        'physical-ai/applications'
      ],
    },
    {
      type: 'category',
      label: 'ROS 2 - Robotic Nervous System',
      link: {
        type: 'doc',
        id: 'ros2/index',
      },
      items: [
        'ros2/introduction',
        'ros2/nodes-topics-services',
        'ros2/urdf-modeling',
        'ros2/python-integration',
        'ros2/practical-examples',
        'ros2/setup-guide'
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin - Simulation',
      link: {
        type: 'doc',
        id: 'simulation/index',
      },
      items: [
        'simulation/introduction',
        'simulation/gazebo-basics',
        'simulation/unity-integration',
        'simulation/sensor-integration',
        'simulation/practical-examples',
        'simulation/setup-guide'
      ],
    },
    {
      type: 'category',
      label: 'AI-Robot Brain',
      link: {
        type: 'doc',
        id: 'ai-robot-brain/index',
      },
      items: [
        'ai-robot-brain/introduction',
        'ai-robot-brain/perception',
        'ai-robot-brain/vslam',
        'ai-robot-brain/nav2-planning',
        'ai-robot-brain/practical-examples'
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA)',
      link: {
        type: 'doc',
        id: 'vla/index',
      },
      items: [
        'vla/voice-processing',
        'vla/cognitive-planning',
        'vla/llm-integration',
        'vla/practical-examples'
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      link: {
        type: 'doc',
        id: 'capstone/index',
      },
      items: [
        'capstone/project-outline',
        'capstone/implementation',
        'capstone/testing-procedures',
        'capstone/assessment-criteria'
      ],
    },
    {
      type: 'category',
      label: 'References',
      link: {
        type: 'doc',
        id: 'references/citations',
      },
      items: [
        'references/citations',
        'references/apa-citations-verification'
      ],
    }
  ],
};

export default sidebars;