/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'index',
    {
      type: 'category',
      label: 'Project Overview',
      items: ['specification', 'implementation-plan', 'data-model'],
    },
    {
      type: 'category',
      label: 'Technical Details',
      items: ['ros2-interfaces', 'research-findings', 'quickstart-guide'],
    },
    {
      type: 'category',
      label: 'Implementation',
      items: ['feature-tasks'],
    },
  ],
};

module.exports = sidebars;