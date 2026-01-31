# Quickstart Guide: Module 2 (Gazebo/Simulation) Content Production

## Overview
This guide explains how to create and maintain Module 2 content (Gazebo simulation fundamentals, robot model integration, physics/sensor simulation, and ROS2 integration) following established patterns and standards.

## Prerequisites
- Node.js 18+ installed
- Docusaurus knowledge
- Understanding of Gazebo simulation concepts
- ROS2 knowledge for integration sections
- Basic understanding of URDF/SDF formats

## Creating a New Lesson
1. **Choose the appropriate week directory**:
   - Week 1: Gazebo Simulation Fundamentals
   - Week 2: Robot Model Integration in Gazebo
   - Week 3: Physics and Sensor Simulation
   - Week 4: Gazebo-ROS2 Integration

2. **Follow the lesson template**:
   ```md
   ---
   title: "[LESSON TITLE]"
   description: "[Brief description of lesson content]"
   tags: [gazebo, simulation, robotics, tag3]
   learning-objectives:
     - "Understand [concept 1]"
     - "Apply [concept 2]"
     - "Evaluate [concept 3]"
   ---

   # [Lesson Title]

   ## Learning Objectives
   <!-- Objectives listed above -->

   ## Introduction
   <!-- Brief introduction to the topic -->

   ## [Main Content Sections]
   <!-- Detailed content with examples -->

   ## Key Terms
   <!-- Important terminology -->

   ## Exercises
   <!-- Hands-on activities -->

   ## Summary
   <!-- Recap of key concepts -->

   ## Quiz Questions
   <!-- Assessment questions -->
   ```

## Lab Exercise Structure
Each lab exercise should follow this pattern:
```
## Setup
- Prerequisites and environment requirements
- Installation steps if needed

## Steps
1. Step-by-step instructions with exact commands
2. Expected outputs after each step
3. Verification checkpoints to confirm progress

## Expected Output
- What students should see after each step
- Screenshots or text output examples

## Verification
- How to verify the exercise was successful
- Troubleshooting tips for common issues

## Troubleshooting
- Common issues and solutions

## Challenge Extension (Optional)
- Advanced exercises for interested students
```

## Asset Management
Place simulation assets in the appropriate directories:
- World files: `static/assets/module-2/worlds/`
- URDF models: `static/assets/module-2/models/`
- Launch files: `static/assets/module-2/launch/`
- Configuration files: `static/assets/module-2/config/`
- Images: `static/assets/module-2/images/`

## Internal Linking
Use Docusaurus markdown links:
- `[Link text](/docs/path/to/target)`
- Relative links within Module 2: `[Related concept](./week-2/lesson-title)`
- Link to assets: `[Download asset](/assets/module-2/path/to/asset)`

## Content QA Process
1. Technical review by domain expert
2. Lab reproducibility test by following exact steps
3. Terminology consistency check
4. Peer review
5. Student feedback integration