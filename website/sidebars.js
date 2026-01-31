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
            'module-1/week-1/index',
            'module-1/week-1/foundations',
            'module-1/week-1/exercises',
            'module-1/week-1/quiz'
          ]
        },
        {
          type: 'category',
          label: 'Week 2: ROS2 Introduction and Core Concepts',
          items: [
            'module-1/week-2/index',
            'module-1/week-2/ros2-intro',
            'module-1/week-2/ros2-core-concepts',
            'module-1/week-2/exercises',
            'module-1/week-2/quiz'
          ]
        },
        {
          type: 'category',
          label: 'Week 3: Embodied Cognition and Environmental Interaction',
          items: [
            'module-1/week-3/index',
            'module-1/week-3/embodied-cognition',
            'module-1/week-3/environmental-interaction',
            'module-1/week-3/exercises',
            'module-1/week-3/quiz'
          ]
        },
        'module-1/capstone-project'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Robot Simulation with Gazebo',
      collapsible: true,
      collapsed: false,
      items: [
        'module-2/index',
        {
          type: 'category',
          label: 'Week 1: Gazebo Simulation Fundamentals',
          items: [
            'module-2/week-1/index',
            'module-2/week-1/gazebo-fundamentals',
            'module-2/week-1/exercises'
          ]
        },
        {
          type: 'category',
          label: 'Week 2: Robot Model Integration',
          items: [
            'module-2/week-2/index',
            'module-2/week-2/robot-model-integration',
            'module-2/week-2/exercises'
          ]
        },
        {
          type: 'category',
          label: 'Week 3: Physics and Sensor Simulation',
          items: [
            'module-2/week-3/index',
            'module-2/week-3/physics-sensor-simulation',
            'module-2/week-3/exercises'
          ]
        },
        {
          type: 'category',
          label: 'Week 4: ROS2 Integration',
          items: [
            'module-2/week-4/index',
            'module-2/week-4/gazebo-ros2-integration',
            'module-2/week-4/exercises'
          ]
        },
        'module-2/quiz',
        'module-2/capstone-project'
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
            'module-3/week-7/index'
          ]
        },
        {
          type: 'category',
          label: 'Week 8: PID Controllers and Balance Control',
          items: [
            'module-3/week-8/index'
          ]
        },
        {
          type: 'category',
          label: 'Week 9: Walking Algorithms and Gait Generation',
          items: [
            'module-3/week-9/index'
          ]
        },
        {
          type: 'category',
          label: 'Week 10: Locomotion and Motion Planning',
          items: [
            'module-3/week-10/index'
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
        'module-4/index',
        {
          type: 'category',
          label: 'Week 11: Machine Learning for Humanoid AI',
          items: [
            'module-4/week-11/index'
          ]
        },
        {
          type: 'category',
          label: 'Week 12: Perception and Multimodal Integration',
          items: [
            'module-4/week-12/index'
          ]
        },
        {
          type: 'category',
          label: 'Week 13: Real World Applications and Adaptation',
          items: [
            'module-4/week-13/index'
          ]
        }
      ],
    },
    {
      type: 'category',
      label: 'Tutorial',
      items: ['hello'],
    },
  ],
};