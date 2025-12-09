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
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;