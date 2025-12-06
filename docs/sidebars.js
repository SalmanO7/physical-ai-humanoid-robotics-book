/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics',
      collapsed: false,
      items: [
        'index',
        'specification',
        'implementation-plan',
        'data-model',
        'ros2-interfaces',
        'research-findings',
        'quickstart-guide',
        'feature-tasks'
      ],
    },
  ],
};

module.exports = sidebars;