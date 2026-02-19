# Course Structure Examples

Real-world examples of course outlines converted to Docusaurus structures.

## Example 1: Physical AI & Humanoid Robotics

### Input: Course Outline

```
Physical AI & Humanoid Robotics

Module 1: The Robotic Nervous System (ROS 2)
  Weeks 1-2: Introduction to Physical AI
    - Foundations of Physical AI and embodied intelligence
    - From digital AI to robots that understand physical laws
    - Overview of humanoid robotics landscape
    - Sensor systems: LIDAR, cameras, IMUs, force/torque sensors
  
  Weeks 3-5: ROS 2 Fundamentals
    - ROS 2 architecture and core concepts
    - Nodes, topics, services, and actions
    - Building ROS 2 packages with Python
    - Launch files and parameter management

Module 2: The Digital Twin (Gazebo & Unity)
  Weeks 6-7: Robot Simulation with Gazebo
    - Gazebo simulation environment setup
    - URDF and SDF robot description formats
    - Physics simulation and sensor simulation
    - Introduction to Unity for robot visualization
```

### Output: Directory Structure

```
docs/
├── index.mdx
├── introduction.mdx
├── module-1-robotic-nervous-system/
│   ├── _category_.json
│   ├── week-1-2/
│   │   ├── _category_.json
│   │   ├── 1-foundations-physical-ai.mdx
│   │   ├── 2-digital-ai-to-robots.mdx
│   │   ├── 3-humanoid-robotics-landscape.mdx
│   │   └── 4-sensor-systems.mdx
│   └── week-3-5/
│       ├── _category_.json
│       ├── 1-ros-2-architecture.mdx
│       ├── 2-nodes-topics-services.mdx
│       ├── 3-building-packages.mdx
│       └── 4-launch-files.mdx
└── module-2-digital-twin/
    └── week-6-7/
        ├── _category_.json
        ├── 1-gazebo-setup.mdx
        ├── 2-urdf-sdf-formats.mdx
        ├── 3-physics-simulation.mdx
        └── 4-unity-visualization.mdx
```

### Output: sidebars.ts

```typescript
import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'index',
    'introduction',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      link: {
        type: 'generated-index',
        description: 'Learn about ROS 2 and robot control middleware.',
      },
      items: [
        {
          type: 'category',
          label: 'Weeks 1-2: Introduction to Physical AI',
          items: [
            'module-1-robotic-nervous-system/week-1-2/1-foundations-physical-ai',
            'module-1-robotic-nervous-system/week-1-2/2-digital-ai-to-robots',
            'module-1-robotic-nervous-system/week-1-2/3-humanoid-robotics-landscape',
            'module-1-robotic-nervous-system/week-1-2/4-sensor-systems',
          ],
        },
        {
          type: 'category',
          label: 'Weeks 3-5: ROS 2 Fundamentals',
          items: [
            'module-1-robotic-nervous-system/week-3-5/1-ros-2-architecture',
            'module-1-robotic-nervous-system/week-3-5/2-nodes-topics-services',
            'module-1-robotic-nervous-system/week-3-5/3-building-packages',
            'module-1-robotic-nervous-system/week-3-5/4-launch-files',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      link: {
        type: 'generated-index',
        description: 'Master Gazebo and Unity for robot simulation.',
      },
      items: [
        {
          type: 'category',
          label: 'Weeks 6-7: Robot Simulation with Gazebo',
          items: [
            'module-2-digital-twin/week-6-7/1-gazebo-setup',
            'module-2-digital-twin/week-6-7/2-urdf-sdf-formats',
            'module-2-digital-twin/week-6-7/3-physics-simulation',
            'module-2-digital-twin/week-6-7/4-unity-visualization',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
```

## Example 2: Web Development Course

### Input: Course Outline

```
Modern Web Development

Module 1: Frontend Foundations
  Week 1: HTML & CSS Basics
    - HTML document structure
    - CSS selectors and styling
    - Responsive design principles
  
  Week 2: JavaScript Fundamentals
    - Variables and data types
    - Functions and scope
    - DOM manipulation

Module 2: React Development
  Week 3: React Basics
    - Components and JSX
    - Props and state
    - Event handling
  
  Week 4: Advanced React
    - Hooks and lifecycle
    - Context API
    - Performance optimization
```

### Output: Directory Structure

```
docs/
├── index.mdx
├── module-1-frontend-foundations/
│   ├── _category_.json
│   ├── week-1/
│   │   ├── _category_.json
│   │   ├── 1-html-structure.mdx
│   │   ├── 2-css-selectors.mdx
│   │   └── 3-responsive-design.mdx
│   └── week-2/
│       ├── _category_.json
│       ├── 1-variables-data-types.mdx
│       ├── 2-functions-scope.mdx
│       └── 3-dom-manipulation.mdx
└── module-2-react-development/
    ├── week-3/
    │   ├── _category_.json
    │   ├── 1-components-jsx.mdx
    │   ├── 2-props-state.mdx
    │   └── 3-event-handling.mdx
    └── week-4/
        ├── _category_.json
        ├── 1-hooks-lifecycle.mdx
        ├── 2-context-api.mdx
        └── 3-performance-optimization.mdx
```

## Example 3: Data Science Course

### Input: Course Outline

```
Data Science Fundamentals

Module 1: Python for Data Science
  Weeks 1-3: Python Essentials
    - Python basics and syntax
    - Data structures (lists, dicts, sets)
    - NumPy arrays and operations
    - Pandas DataFrames

Module 2: Data Analysis
  Weeks 4-6: Statistical Analysis
    - Descriptive statistics
    - Probability distributions
    - Hypothesis testing
    - Correlation and regression

Module 3: Machine Learning
  Weeks 7-9: Supervised Learning
    - Linear regression
    - Logistic regression
    - Decision trees
    - Model evaluation
```

### Generated JSON Structure

```json
{
  "title": "Data Science Fundamentals",
  "modules": [
    {
      "number": 1,
      "title": "Python for Data Science",
      "slug": "python-data-science",
      "weeks": [
        {
          "start": 1,
          "end": 3,
          "topic": "Python Essentials",
          "slug": "week-1-3",
          "chapters": [
            {
              "title": "Python basics and syntax",
              "slug": "python-basics-syntax"
            },
            {
              "title": "Data structures (lists, dicts, sets)",
              "slug": "data-structures-lists-dicts-sets"
            },
            {
              "title": "NumPy arrays and operations",
              "slug": "numpy-arrays-operations"
            },
            {
              "title": "Pandas DataFrames",
              "slug": "pandas-dataframes"
            }
          ]
        }
      ]
    }
  ]
}
```

## Example _category_.json Files

### Module Level

```json
{
  "label": "Module 1: The Robotic Nervous System",
  "position": 1,
  "link": {
    "type": "generated-index",
    "description": "Learn about ROS 2 and robot control middleware."
  }
}
```

### Week Level

```json
{
  "label": "Weeks 1-2: Introduction to Physical AI",
  "position": 1,
  "link": {
    "type": "generated-index",
    "description": "Foundations of Physical AI and embodied intelligence."
  }
}
```

## Example MDX Frontmatter

### Chapter File

```mdx
---
title: Foundations of Physical AI and Embodied Intelligence
sidebar_position: 1
description: Introduction to the core concepts of Physical AI
keywords:
  - physical ai
  - embodied intelligence
  - robotics
  - ai systems
---

# Foundations of Physical AI and Embodied Intelligence

[Content here...]
```

### Index File

```mdx
---
title: Physical AI & Humanoid Robotics
sidebar_position: 1
slug: /
---

# Physical AI & Humanoid Robotics

Welcome to the comprehensive course on Physical AI and Humanoid Robotics!

## What You'll Learn

This course covers...

## Course Structure

The course is organized into modules...
```

## Command Workflow Examples

### Starting Fresh

```bash
# 1. Parse the course outline
python3 scripts/parse_outline.py course_description.txt

# 2. Generate everything
python3 scripts/full_regenerate.py course_description.json docs/ --validate
```

### Adding New Content

```bash
# 1. Edit outline JSON manually to add new chapters
# 2. Regenerate stubs only (won't overwrite existing)
python3 scripts/generate_stubs.py outline.json docs/

# 3. Update sidebar
python3 scripts/generate_sidebar.py outline.json --output sidebars.ts

# 4. Validate
python3 scripts/validate_structure.py docs/ sidebars.ts
```

### Restructuring Course

```bash
# 1. Update outline JSON with new structure
# 2. Full regeneration
python3 scripts/full_regenerate.py outline.json docs/ --validate

# 3. Manually migrate content from old files to new files
# 4. Validate again
python3 scripts/validate_structure.py docs/ sidebars.ts
```