/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
export default {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Course Overview',
      items: [
        'course-overview',
        'learning-objectives',
        'syllabus',
        'assessments',
        'hardware-requirements'
      ],
    },
    {
      type: 'category',
      label: 'Module 1: Foundations of Physical AI',
      collapsible: true,
      collapsed: false,
      items: [
        'module-1/index',
        {
          type: 'category',
          label: 'Week 1: Introduction to Physical AI',
          items: [
            'module-1/week-1/index'
          ]
        },
        {
          type: 'category',
          label: 'Week 2: Sensorimotor Learning',
          items: [
            'module-1/week-2/index'
          ]
        },
        {
          type: 'category',
          label: 'Week 3: Embodied Cognition',
          items: [
            'module-1/week-3/index'
          ]
        }
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Robot Kinematics and Dynamics',
      collapsible: true,
      collapsed: false,
      items: [
        'module-2/index',
        {
          type: 'category',
          label: 'Week 4: Forward and Inverse Kinematics',
          items: [
            'module-2/week-1/index'
          ]
        },
        {
          type: 'category',
          label: 'Week 5: Robot Dynamics and Motion Planning',
          items: [
            'module-2/week-2/index'
          ]
        },
        {
          type: 'category',
          label: 'Week 6: Degrees of Freedom and Stability',
          items: [
            'module-2/week-3/index'
          ]
        }
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Control Systems and Locomotion',
      collapsible: true,
      collapsed: false,
      items: [
        'module-3/index',
        {
          type: 'category',
          label: 'Week 7: Feedback Control Systems',
          items: [
            'module-3/week-1/index'
          ]
        }
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Advanced AI for Humanoid Systems',
      collapsible: true,
      collapsed: false,
      items: [
        'module-4/index'
      ],
    },
    {
      type: 'category',
      label: 'Tutorial',
      items: ['hello'],
    },
  ],
};