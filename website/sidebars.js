/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
export default {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
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
      label: 'Module 1 — Foundations of Physical AI',
      collapsible: true,
      collapsed: false,
      items: [
        'module-1/index',
        {
          type: 'category',
          label: '1.1 Physical AI Foundations',
          items: [
            'module-1/1.1-introduction-to-physical-ai/index',
            'module-1/1.1-introduction-to-physical-ai/physical-ai-foundations',
            'module-1/1.1-introduction-to-physical-ai/physical-ai-lab-exercises',
            'module-1/1.1-introduction-to-physical-ai/quiz'
          ]
        },
        {
          type: 'category',
          label: '1.2 ROS 2 Architecture & Communication',
          items: [
            'module-1/1.2-ros2-introduction-and-core-concepts/index',
            'module-1/1.2-ros2-introduction-and-core-concepts/ros2-overview-foundations',
            'module-1/1.2-ros2-introduction-and-core-concepts/ros2-architecture-communication-patterns',
            'module-1/1.2-ros2-introduction-and-core-concepts/ros2-lab-exercises',
            'module-1/1.2-ros2-introduction-and-core-concepts/quiz'
          ]
        },
        {
          type: 'category',
          label: '1.3 Embodied Cognition & Interaction',
          items: [
            'module-1/1.3-embodied-cognition-and-environmental-interaction/index',
            'module-1/1.3-embodied-cognition-and-environmental-interaction/embodied-cognition-theory',
            'module-1/1.3-embodied-cognition-and-environmental-interaction/environmental-interaction-principles',
            'module-1/1.3-embodied-cognition-and-environmental-interaction/embodied-robotics-lab-exercises',
            'module-1/1.3-embodied-cognition-and-environmental-interaction/quiz'
          ]
        },
        'module-1/capstone-project'
      ],
    },
    {
      type: 'category',
      label: 'Module 2 — Robot Simulation & Gazebo',
      collapsible: true,
      collapsed: false,
      items: [
        'module-2/index',
        {
          type: 'category',
          label: '2.1 Gazebo Simulation Fundamentals',
          items: [
            'module-2/2.1-gazebo-simulation-fundamentals/index',
            'module-2/2.1-gazebo-simulation-fundamentals/gazebo-simulation-basics',
            'module-2/2.1-gazebo-simulation-fundamentals/simulation-lab-exercises'
          ]
        },
        {
          type: 'category',
          label: '2.2 Robot Modeling & Integration',
          items: [
            'module-2/2.2-robot-model-integration/index',
            'module-2/2.2-robot-model-integration/robot-model-design-integration',
            'module-2/2.2-robot-model-integration/model-integration-lab-exercises'
          ]
        },
        {
          type: 'category',
          label: '2.3 Physics Engines & Sensors',
          items: [
            'module-2/2.3-physics-and-sensor-simulation/index',
            'module-2/2.3-physics-and-sensor-simulation/physics-engine-sensor-modeling',
            'module-2/2.3-physics-and-sensor-simulation/sensor-simulation-lab-exercises'
          ]
        },
        {
          type: 'category',
          label: '2.4 ROS 2–Gazebo Integration',
          items: [
            'module-2/2.4-gazebo-ros2-integration/index',
            'module-2/2.4-gazebo-ros2-integration/ros2-gazebo-integration-framework',
            'module-2/2.4-gazebo-ros2-integration/integration-testing-lab-exercises'
          ]
        },
        'module-2/quiz',
        'module-2/capstone-project'
      ],
    },
    {
      type: 'category',
      label: 'Module 3 — Isaac Sim & Perception',
      collapsible: true,
      collapsed: false,
      items: [
        'module-3/index',
        {
          type: 'category',
          label: '3.1 Isaac Sim Fundamentals',
          items: [
            'module-3/3.1-introduction-to-isaac-sim/isaac-sim-overview-foundations',
            'module-3/3.1-introduction-to-isaac-sim/isaac-sim-environment-setup',
            'module-3/3.1-introduction-to-isaac-sim/lab-1-isaac-sim-fundamentals'
          ]
        },
        {
          type: 'category',
          label: '3.2 Perception & Sensor Fusion',
          items: [
            'module-3/3.2-perception-pipelines/perception-systems-overview',
            'module-3/3.2-perception-pipelines/multi-sensor-data-processing',
            'module-3/3.2-perception-pipelines/lab-2-perception-workflow-implementation'
          ]
        },
        {
          type: 'category',
          label: '3.3 Navigation & Path Planning',
          items: [
            'module-3/3.3-navigation-pipelines/navigation-systems-fundamentals',
            'module-3/3.3-navigation-pipelines/trajectory-planning-obstacle-navigation',
            'module-3/3.3-navigation-pipelines/lab-3-navigation-system-implementation'
          ]
        },
        {
          type: 'category',
          label: '3.4 Sim-to-Real Transfer',
          items: [
            'module-3/3.4-sim-to-real-concepts/sim-to-real-transfer-methodologies',
            'module-3/3.4-sim-to-real-concepts/domain-adaptation-techniques',
            'module-3/3.4-sim-to-real-concepts/lab-4-sim-to-real-implementation'
          ]
        },
        {
          type: 'category',
          label: '3.5 Capstone & Assessment',
          items: [
            'module-3/3.5-integrated-project-and-assessment/capstone-project-implementation',
            'module-3/3.5-integrated-project-and-assessment/module-assessment-evaluation',
            'module-3/3.5-integrated-project-and-assessment/troubleshooting-guide'
          ]
        }
      ],
    },
    {
      type: 'category',
      label: 'Module 4 — Humanoid Robotics Systems',
      collapsible: true,
      collapsed: false,
      items: [
        'module-4/intro',
        {
          type: 'category',
          label: '4.1 Humanoid Kinematics',
          items: [
            'module-4/4.1-humanoid-fundamentals/index',
            'module-4/4.1-humanoid-fundamentals/forward-kinematics',
            'module-4/4.1-humanoid-fundamentals/inverse-kinematics',
            'module-4/4.1-humanoid-fundamentals/coordinate-systems',
            'module-4/4.1-humanoid-fundamentals/kinematics-lab'
          ]
        },
        {
          type: 'category',
          label: '4.2 Locomotion & Manipulation',
          items: [
            'module-4/4.2-locomotion-manipulation/index',
            'module-4/4.2-locomotion-manipulation/bipedal-walking',
            'module-4/4.2-locomotion-manipulation/gait-patterns',
            'module-4/4.2-locomotion-manipulation/grasping-principles',
            'module-4/4.2-locomotion-manipulation/dexterous-manipulation',
            'module-4/4.2-locomotion-manipulation/locomotion-manipulation-lab'
          ]
        },
        {
          type: 'category',
          label: '4.3 Vision-Language-Action Models',
          items: [
            'module-4/4.3-vla-concepts/index',
            'module-4/4.3-vla-concepts/vision-language-action-models',
            'module-4/4.3-vla-concepts/embodied-ai-integration',
            'module-4/4.3-vla-concepts/practical-applications',
            'module-4/4.3-vla-concepts/vla-mini-experiments'
          ]
        },
        {
          type: 'category',
          label: '4.4 Human-Robot Interaction',
          items: [
            'module-4/4.4-conversational-robotics/index',
            'module-4/4.4-conversational-robotics/dialogue-systems',
            'module-4/4.4-conversational-robotics/social-interaction',
            'module-4/4.4-conversational-robotics/human-robot-communication',
            'module-4/4.4-conversational-robotics/conversational-robotics-lab'
          ]
        },
        {
          type: 'category',
          label: '4.5 Safety & Ethics',
          items: [
            'module-4/4.5-safety-considerations/index',
            'module-4/4.5-safety-considerations/physical-safety',
            'module-4/4.5-safety-considerations/ethical-considerations'
          ]
        },
        {
          type: 'category',
          label: 'Capstone — Integrated Humanoid System',
          items: [
            'module-4/capstone-project/index',
            'module-4/capstone-project/project-overview',
            'module-4/capstone-project/implementation-guidelines',
            'module-4/capstone-project/evaluation-criteria'
          ]
        },
        'module-4/module-summary'
      ],
    },
    {
      type: 'category',
      label: 'Tutorial',
      items: ['hello'],
    },
  ],
};
