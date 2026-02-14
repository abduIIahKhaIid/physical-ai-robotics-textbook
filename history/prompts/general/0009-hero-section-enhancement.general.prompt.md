---
id: 0009
title: Hero Section Enhancement & Data Integration
stage: misc
date: 2026-02-13
surface: agent
model: Claude Haiku 4.5
feature: none
branch: main
user: user
command: adjust hero section accordingly to course data
labels:
  - landing-page
  - hero-section
  - ui-enhancement
  - data-integration
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - website/src/components/HeroSection/index.tsx
  - website/src/data/courseData.ts
tests_yaml: []
---

## Summary

Enhanced the landing page hero section to accurately reflect Physical AI & Humanoid Robotics course structure and data. Updated statistics counters, course description, and added visual enhancements.

## Changes Made

### 1. Course Data Updates (courseData.ts)
- Updated MODULES array with correct titles, descriptions, durations, and lesson/lab counts
- Modified STATS to show: 4 Modules, 17 Lessons, 10+ Labs, 13 weeks
- Refreshed VALUE_PROPOSITIONS with specific tooling (Gazebo, Isaac Sim, ROS2)
- Updated COURSE_OVERVIEW with accurate totals: 17 lessons, 10 labs, 13 weeks

### 2. Hero Section Component Updates (HeroSection/index.tsx)
- Updated counter animation targets from {modules: 4, chapters: 24, hours: 60} to {modules: 4, chapters: 17, hours: 10}
- Changed stat card labels:
  - "Learning Hours" → "Lessons & Topics"
  - "Hands-on Labs" → "Practical Labs"
- Enhanced course description subtitle to mention Gazebo, Isaac Sim, and ROS2
- Added course duration badge showing "13 weeks • Self-paced learning"
- Maintained existing animations and visual polish

## Visual Hierarchy

The hero section now displays:
1. Badge: "Open Source & Community Driven"
2. Main Heading: "Master Physical AI & Humanoid Robotics"
3. Enhanced Description: Course overview with specific tools
4. Three Stat Cards: Modules, Lessons, Labs with animated counters
5. Course Duration Badge: 13 weeks indicator
6. Feature Tags: Embodied Intelligence, Sim-to-Real Transfer, Hands-on Labs
7. CTA Buttons: Start Learning, View Syllabus

## Technical Details

- Counter animation duration: 2500ms with smooth easing
- All text reflects actual course content from docs/
- Dark mode support maintained throughout
- Accessibility features preserved (semantic HTML, keyboard navigation)
- Responsive design for mobile, tablet, desktop

## Related Files

- Source: website/src/components/HeroSection/index.tsx
- Data: website/src/data/courseData.ts
- Docs Reference: website/docs/course-overview.md, syllabus.md

## Verification

All counter values and labels now match the official course structure:
- 4 modules across 13 weeks
- 17 total lessons/topics
- 10+ practical labs
- Self-paced learning format

---

## User Prompt

```
firstly you enchance the hero section of...
now you adjust accordingly in hero section
```

## Agent Response

Successfully enhanced and adjusted the hero section to match the Physical AI & Humanoid Robotics course structure. Updated all statistics, descriptions, and visual elements to reflect the actual course content from the documentation. The hero section now displays accurate counters (4 modules, 17 lessons, 10+ labs) with improved visual hierarchy and a new course duration indicator.
