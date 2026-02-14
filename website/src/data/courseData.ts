/**
 * Centralized course data configuration for all components
 * Single source of truth for modules, statistics, and course information
 */

export const MODULES = [
  {
    id: 1,
    title: 'Module 1: Foundations of Physical AI & Robotics',
    description: 'Introduction to Physical AI, embodied intelligence, sensorimotor learning, and environmental interaction',
    link: '/docs/module-1/',
    color: 'blue',
    icon: 'brain',
    duration: '3 weeks',
    lessons: 3,
    labs: 1,
    topics: [
      'Physical AI Foundations & Concepts',
      'ROS2 Architecture & Communication Patterns',
      'Embodied Cognition & Environmental Interaction',
    ],
    capstone: true,
  },
  {
    id: 2,
    title: 'Module 2: Advanced Robot Simulation & Gazebo',
    description: 'Master Gazebo simulation, URDF/SDF integration, physics engines, sensors, and ROS2 integration',
    link: '/docs/module-2/',
    color: 'purple',
    icon: 'cog',
    duration: '4 weeks',
    lessons: 4,
    labs: 3,
    topics: [
      'Gazebo Simulation & Environment Setup',
      'Robot Model Design & Integration',
      'Physics Engines & Sensor Modeling',
      'ROS2-Gazebo Integration Framework',
    ],
    capstone: true,
  },
  {
    id: 3,
    title: 'Module 3: Advanced Isaac Sim & Perception',
    description: 'NVIDIA Isaac Sim, perception pipelines, navigation systems, sim-to-real transfer, and integrated projects',
    link: '/docs/module-3/',
    color: 'cyan',
    icon: 'circuit',
    duration: '4 weeks',
    lessons: 5,
    labs: 4,
    topics: [
      'Isaac Sim Fundamentals & Environment Setup',
      'Perception Systems & Multi-Sensor Processing',
      'Navigation Systems & Trajectory Planning',
      'Sim-to-Real Transfer & Domain Adaptation',
    ],
    capstone: true,
  },
  {
    id: 4,
    title: 'Module 4: Advanced AI for Humanoid Systems',
    description: 'Humanoid fundamentals, locomotion, manipulation, VLA concepts, conversational robotics, and safety',
    link: '/docs/module-4/',
    color: 'indigo',
    icon: 'eye',
    duration: '3 weeks',
    lessons: 5,
    labs: 2,
    topics: [
      'Humanoid Kinematics & Mathematical Foundations',
      'Locomotion & Dexterous Manipulation',
      'Vision-Language-Action (VLA) AI Systems',
      'Conversational Robotics & Human-Robot Interaction',
      'Safety, Ethics & Responsible AI in Robotics',
    ],
    capstone: true,
  },
];

export const STATS = [
  {
    value: '4',
    label: 'Core Modules',
    description: 'From Physical AI foundations to Humanoid Systems',
    icon: 'book',
  },
  {
    value: '17',
    label: 'Lessons',
    description: 'Comprehensive curriculum across all modules',
    icon: 'graduation',
  },
  {
    value: '10+',
    label: 'Hands-on Labs',
    description: 'Practical exercises with simulations and projects',
    icon: 'flask',
  },
  {
    value: '13',
    label: 'Weeks',
    description: 'Complete program from start to finish',
    icon: 'calendar',
  },
];

export const HERO_COUNTERS = [
  { key: 'modules', target: 4, label: 'Modules', suffix: '' },
  { key: 'lessons', target: 17, label: 'Lessons', suffix: '' },
  { key: 'labs', target: 10, label: 'Labs', suffix: '+' },
  { key: 'weeks', target: 13, label: 'Weeks', suffix: '' },
];

export const RESOURCES = [
  {
    title: 'Course Syllabus',
    description: '13-week program with detailed weekly breakdown',
    to: '/docs/syllabus',
    icon: 'calendar',
    color: 'blue',
  },
  {
    title: 'Learning Objectives',
    description: 'Specific skills and knowledge outcomes for each module',
    to: '/docs/learning-objectives',
    icon: 'target',
    color: 'purple',
  },
  {
    title: 'Assessment Methods',
    description: 'Grading criteria and evaluation approaches',
    to: '/docs/assessments',
    icon: 'checkmark',
    color: 'cyan',
  },
  {
    title: 'Hardware Requirements',
    description: 'Computing and equipment needs for course completion',
    to: '/docs/hardware-requirements',
    icon: 'harddrive',
    color: 'indigo',
  },
];

export const VALUE_PROPOSITIONS = [
  {
    title: 'Interactive Learning',
    description: 'Engage with hands-on simulations in Gazebo and Isaac Sim, interactive diagrams, and practical exercises that bring robotics concepts to life.',
    icon: 'pointer',
    accent: 'blue',
  },
  {
    title: 'Open Source & Community Driven',
    description: 'The entire curriculum is open source. Contribute chapters, fix issues, submit labs — help shape the future of robotics education.',
    icon: 'code',
    accent: 'purple',
  },
  {
    title: 'Practical Application',
    description: 'Learn with industry-standard tools like ROS2, Gazebo, and Isaac Sim. Apply concepts to real simulation scenarios and sim-to-real transfer.',
    icon: 'book-check',
    accent: 'cyan',
  },
  {
    title: 'From Theory to Practice',
    description: 'Progress from Physical AI foundations through perception, control, and humanoid robotics with hands-on labs and capstone projects.',
    icon: 'flask',
    accent: 'indigo',
  },
];

export const COURSE_OVERVIEW = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Master the intersection of AI and physical systems',
  description: 'An interactive, open-source textbook that bridges the gap between theory and practice. Learn to build intelligent robots that perceive, reason, and act in the physical world.',
  totalLessons: 17,
  totalLabs: 10,
  estimatedDuration: '13 weeks',
  targetAudience: 'Beginners to advanced learners interested in robotics, AI, and embodied intelligence',
};
