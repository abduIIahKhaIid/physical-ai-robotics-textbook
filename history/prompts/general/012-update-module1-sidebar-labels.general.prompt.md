---
id: 012
title: Update Module 1 Sidebar Labels
stage: general
date: 2026-02-28
surface: agent
model: claude-sonnet-4-6
feature: none
branch: main
user: user
command: edit
labels: ["sidebar", "frontmatter", "module-1", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - website/docs/module-1/index.md
  - website/docs/module-1/1.1-introduction-to-physical-ai/index.md
  - website/docs/module-1/1.1-introduction-to-physical-ai/physical-ai-foundations.md
  - website/docs/module-1/1.1-introduction-to-physical-ai/physical-ai-lab-exercises.md
  - website/docs/module-1/1.1-introduction-to-physical-ai/quiz.md
  - website/docs/module-1/1.2-ros2-introduction-and-core-concepts/index.md
  - website/docs/module-1/1.2-ros2-introduction-and-core-concepts/ros2-overview-foundations.md
  - website/docs/module-1/1.2-ros2-introduction-and-core-concepts/ros2-architecture-communication-patterns.md
  - website/docs/module-1/1.2-ros2-introduction-and-core-concepts/ros2-lab-exercises.md
  - website/docs/module-1/1.2-ros2-introduction-and-core-concepts/quiz.md
  - website/docs/module-1/1.3-embodied-cognition-and-environmental-interaction/index.md
  - website/docs/module-1/1.3-embodied-cognition-and-environmental-interaction/embodied-cognition-theory.md
  - website/docs/module-1/1.3-embodied-cognition-and-environmental-interaction/environmental-interaction-principles.md
  - website/docs/module-1/1.3-embodied-cognition-and-environmental-interaction/embodied-robotics-lab-exercises.md
  - website/docs/module-1/1.3-embodied-cognition-and-environmental-interaction/quiz.md
  - website/docs/module-1/capstone-project.md
tests:
  - none
---

## Prompt

Update the sidebar_label in these files' YAML frontmatter. Use the Edit tool to change ONLY the sidebar_label (or add one if missing). Do NOT change the title.

1. website/docs/module-1/index.md — change sidebar_label to "Module Overview"
2. website/docs/module-1/1.1-introduction-to-physical-ai/index.md — change sidebar_label to "1.1 Overview"
3. website/docs/module-1/1.1-introduction-to-physical-ai/physical-ai-foundations.md — add sidebar_label: "1.1.1 Core Concepts"
4. website/docs/module-1/1.1-introduction-to-physical-ai/physical-ai-lab-exercises.md — add sidebar_label: "1.1.2 Lab Exercises"
5. website/docs/module-1/1.1-introduction-to-physical-ai/quiz.md — add sidebar_label: "1.1.3 Quiz"
6. website/docs/module-1/1.2-ros2-introduction-and-core-concepts/index.md — change sidebar_label to "1.2 Overview"
7. website/docs/module-1/1.2-ros2-introduction-and-core-concepts/ros2-overview-foundations.md — add sidebar_label: "1.2.1 ROS 2 Foundations"
8. website/docs/module-1/1.2-ros2-introduction-and-core-concepts/ros2-architecture-communication-patterns.md — add sidebar_label: "1.2.2 Communication Patterns"
9. website/docs/module-1/1.2-ros2-introduction-and-core-concepts/ros2-lab-exercises.md — add sidebar_label: "1.2.3 Lab Exercises"
10. website/docs/module-1/1.2-ros2-introduction-and-core-concepts/quiz.md — add sidebar_label: "1.2.4 Quiz"
11. website/docs/module-1/1.3-embodied-cognition-and-environmental-interaction/index.md — change sidebar_label to "1.3 Overview"
12. website/docs/module-1/1.3-embodied-cognition-and-environmental-interaction/embodied-cognition-theory.md — add sidebar_label: "1.3.1 Cognition Theory"
13. website/docs/module-1/1.3-embodied-cognition-and-environmental-interaction/environmental-interaction-principles.md — add sidebar_label: "1.3.2 Interaction Principles"
14. website/docs/module-1/1.3-embodied-cognition-and-environmental-interaction/embodied-robotics-lab-exercises.md — add sidebar_label: "1.3.3 Lab Exercises"
15. website/docs/module-1/1.3-embodied-cognition-and-environmental-interaction/quiz.md — add sidebar_label: "1.3.4 Quiz"
16. website/docs/module-1/capstone-project.md — add sidebar_label: "Capstone Project"

## Response snapshot

Updated sidebar_label in all 16 Module 1 files. For 3 files that already had sidebar_label (module-1/index.md, 1.1 index, 1.2 index, 1.3 index), replaced the existing value. For the remaining 12 files, added sidebar_label immediately after the title line in frontmatter.

## Outcome

- Impact: Sidebar navigation for Module 1 now shows concise, numbered labels instead of full titles.
- Tests: none
- Files: 16 files modified
- Next prompts: Verify sidebar rendering in Docusaurus dev server; repeat for Module 2 and Module 3.
- Reflection: Straightforward batch frontmatter edit. All edits were independent and parallelizable.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
