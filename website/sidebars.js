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
      label: 'Module 3: NVIDIA Isaac Sim / Isaac ROS Lessons',
      collapsible: true,
      collapsed: false,
      items: [
        'module-3/index',
        {
          type: 'category',
          label: 'Week 1: Introduction to Isaac Sim Environment',
          items: [
            'module-3/week-1/introduction-to-isaac-sim',
            'module-3/week-1/isaac-sim-setup',
            'module-3/week-1/lab-1-isaac-sim-basics'
          ]
        },
        {
          type: 'category',
          label: 'Week 2: Perception Pipeline Implementation',
          items: [
            'module-3/week-2/perception-pipelines-overview',
            'module-3/week-2/camera-lidar-processing',
            'module-3/week-2/lab-2-perception-pipeline'
          ]
        },
        {
          type: 'category',
          label: 'Week 3: Navigation Pipeline Development',
          items: [
            'module-3/week-3/navigation-pipeline-basics',
            'module-3/week-3/path-planning-obstacle-avoidance',
            'module-3/week-3/lab-3-navigation-implementation'
          ]
        },
        {
          type: 'category',
          label: 'Week 4: Sim-to-Real Transfer Concepts',
          items: [
            'module-3/week-4/sim-to-real-concepts',
            'module-3/week-4/domain-randomization',
            'module-3/week-4/lab-4-sim-to-real-transfer'
          ]
        },
        {
          type: 'category',
          label: 'Week 5: Integrated Project and Assessment',
          items: [
            'module-3/week-5/integrated-project',
            'module-3/week-5/module-3-assessment',
            'module-3/week-5/troubleshooting-guide'
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