# Data Model: Module 3 - NVIDIA Isaac Sim / Isaac ROS Lessons

## Overview
This document defines the key entities and data structures for Module 3 educational content covering Isaac Sim and Isaac ROS lessons.

## Key Entities

### 1. Module Lesson
**Description**: A single lesson within the module
**Fields**:
- id: Unique identifier for the lesson
- title: Display title of the lesson
- slug: URL-friendly identifier
- docId: Docusaurus document ID
- week: Week number in the module sequence
- position: Position within the week
- learningObjectives: Array of learning objectives
- prerequisites: Array of prerequisite concepts
- duration: Estimated completion time in minutes
- tags: Array of topic tags
- relatedLessons: Array of related lesson IDs
- difficulty: Beginner/Intermediate/Advanced

**Validation Rules**:
- id must be unique across all lessons
- title must be 10-100 characters
- slug must follow kebab-case format
- learningObjectives must have 1-5 items
- duration must be between 15-120 minutes

### 2. Lab Exercise
**Description**: Hands-on lab exercise with runnable steps
**Fields**:
- id: Unique identifier for the lab
- title: Display title of the lab
- slug: URL-friendly identifier
- docId: Docusaurus document ID
- week: Week number in the module sequence
- lessonId: Associated lesson ID
- objectives: Array of lab objectives
- prerequisites: Array of prerequisite concepts/tools
- estimatedTime: Time to complete in minutes
- steps: Array of lab steps
- expectedOutputs: Array of expected results
- verificationSteps: Array of verification checkpoints
- troubleshootingTips: Array of common issues and solutions
- difficulty: Beginner/Intermediate/Advanced

**Validation Rules**:
- id must be unique across all labs
- steps must have at least 3 items
- expectedOutputs must match step count
- estimatedTime must be between 30-240 minutes

### 3. Lab Step
**Description**: Individual step within a lab exercise
**Fields**:
- stepNumber: Sequential number of the step
- description: Explanation of the step
- command: Terminal/command to execute (optional)
- expectedOutput: What the user should see
- verificationPoint: Boolean indicating if this is a verification checkpoint
- hints: Array of helpful hints
- commonErrors: Array of common mistakes and solutions

**Validation Rules**:
- stepNumber must be sequential starting from 1
- description must be 10-200 characters
- verificationPoint must be boolean

### 4. Configuration Snippet
**Description**: Reusable configuration file for Isaac Sim/ROS
**Fields**:
- id: Unique identifier for the snippet
- name: Display name of the configuration
- filePath: Relative path from static directory
- fileName: Name of the configuration file
- description: Purpose and usage of the configuration
- category: Isaac-Sim/Isaac-ROS/Perception/Navigation/Sim-to-Real
- version: Isaac Sim/ROS version this config is compatible with
- dependencies: Other configurations or packages required
- usageExample: Code snippet showing how to use this configuration

**Validation Rules**:
- id must be unique across all snippets
- filePath must start with /static/
- version must follow semantic versioning

### 5. Code Example
**Description**: Reusable code example for Isaac ROS nodes
**Fields**:
- id: Unique identifier for the example
- name: Display name of the example
- filePath: Relative path from static directory
- fileName: Name of the example file
- description: Purpose and usage of the code
- language: Programming language (Python, C++, etc.)
- category: Perception/Navigation/Sim-to-Real
- dependencies: Packages or libraries required
- usageExample: Code snippet showing how to use this example

**Validation Rules**:
- id must be unique across all examples
- filePath must start with /static/
- language must be a recognized programming language

### 6. Assessment Question
**Description**: Question for module assessment
**Fields**:
- id: Unique identifier for the question
- questionText: The actual question
- questionType: MultipleChoice/ShortAnswer/Practical/Scenario
- difficulty: Beginner/Intermediate/Advanced
- category: Isaac-Sim/Perception/Navigation/Sim-to-Real
- options: Array of possible answers (for multiple choice)
- correctAnswer: The correct answer
- explanation: Explanation of the correct answer
- relatedLessonIds: Array of related lesson IDs

**Validation Rules**:
- questionText must be 10-500 characters
- for MultipleChoice, options must have 2-6 items
- correctAnswer must match one of the options (for MC questions)

### 7. Cross-Module Reference
**Description**: Links between different modules in the curriculum
**Fields**:
- id: Unique identifier for the reference
- sourceModule: Module where the reference originates
- sourceLessonId: Lesson containing the reference
- targetModule: Module being referenced
- targetLessonId: Lesson being referenced
- relationship: Prerequisite/Related/Advanced/Extension
- description: Explanation of why this reference exists

**Validation Rules**:
- sourceModule and targetModule must be valid module identifiers
- relationship must be one of the predefined types

## State Transitions

### Lab Exercise States
- Draft → Review → Published → Updated → Archived
- Only published labs are available to students
- Updates trigger version increment

### Assessment States
- Draft → Review → Published → Active → Inactive
- Active assessments are available for student use
- Inactive assessments are archived but still accessible for reference