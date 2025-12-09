// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {
        type: 'generated-index',
        title: 'Module 1: The Robotic Nervous System (ROS 2)',
        description: 'Learn about ROS 2 fundamentals, Python agents with rclpy, and URDF for humanoid robots.',
        slug: '/module-1-ros2-nervous-system',
      },
      items: [
        'module-1-ros2-nervous-system/ros2-as-nervous-system',
        'module-1-ros2-nervous-system/python-agents-rclpy',
        'module-1-ros2-nervous-system/urdf-humanoid-robots',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      link: {
        type: 'generated-index',
        title: 'Module 2: The Digital Twin (Gazebo & Unity)',
        description: 'Learn about physics simulation in Gazebo and high-fidelity digital twins in Unity.',
        slug: '/module-2-digital-twin',
      },
      items: [
        'module-2-digital-twin/physics-simulation-gazebo',
        'module-2-digital-twin/high-fidelity-digital-twins-unity',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      link: {
        type: 'generated-index',
        title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
        description: 'Learn about NVIDIA Isaac Sim for perception and navigation with Isaac ROS + Nav2.',
        slug: '/module-3-ai-brain',
      },
      items: [
        'module-3-ai-brain/nvidia-isaac-sim-perception',
        'module-3-ai-brain/navigation-isaac-ros-nav2',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      link: {
        type: 'generated-index',
        title: 'Module 4: Vision-Language-Action (VLA)',
        description: 'Learn about voice-to-action pipelines and capstone autonomous humanoid project.',
        slug: '/module-4-vla',
      },
      items: [
        'module-4-vla/voice-to-action-pipelines',
        'module-4-vla/capstone-autonomous-humanoid',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/glossary',
        'appendices/references',
        'appendices/setup-guide',
      ],
      collapsed: true,
    },
  ],
};

export default sidebars;