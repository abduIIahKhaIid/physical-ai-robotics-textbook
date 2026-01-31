# Implementation Tasks: Homepage Landing Page for Physical AI & Humanoid Robotics Textbook

**Feature**: Homepage Landing Page Redesign
**Branch**: `001-homepage-redesign`
**Date**: 2026-01-30
**Spec**: [specs/001-homepage-redesign/spec.md](./spec.md)

## Overview

This document contains the atomic tasks checklist for implementing the homepage redesign. The tasks are organized by user story priority and include all necessary implementation steps, testing requirements, and validation criteria.

## Dependencies

- User Story 3 (Mobile Learner) must be completed before User Story 1 (First-Time Visitor) can be fully validated
- Foundational tasks must be completed before any user story tasks
- Responsive design considerations affect all user stories

## Parallel Execution Opportunities

- Module cards for different modules can be developed in parallel ([P] tasks)
- Styling and component implementation can be worked on simultaneously ([P] tasks)
- Course meta quick-links can be added in parallel with other sections ([P] tasks)

## Implementation Strategy

**MVP Scope**: Focus on User Story 1 (First-Time Visitor) with basic hero section, module cards for Modules 1-4, and primary CTA. Add minimal styling and ensure mobile responsiveness.

**Incremental Delivery**:
- Phase 1-2: Foundation and basic homepage structure
- Phase 3: User Story 1 (core functionality)
- Phase 4: User Story 2 (enhanced features)
- Phase 5: User Story 3 (responsive optimization)
- Final Phase: Polish and validation

---

## Phase 1: Setup

Goal: Initialize project environment and ensure all prerequisites are met

- [X] T001 Set up development environment with Node.js v18+ and npm
- [X] T002 Verify website directory exists and contains Docusaurus installation
- [X] T003 Create backup of current homepage files (index.js and index.module.css)

## Phase 2: Foundational Tasks

Goal: Prepare foundational components and structure needed for all user stories

- [X] T004 Create new ModuleCard component at `website/src/components/ModuleCard/index.js`
- [X] T005 Create styling for ModuleCard component at `website/src/components/ModuleCard/styles.module.css`
- [X] T006 Create CourseMetaSection component at `website/src/components/CourseMetaSection/index.js`
- [X] T007 Create styling for CourseMetaSection component at `website/src/components/CourseMetaSection/styles.module.css`
- [X] T008 Create ChatbotTeaser component at `website/src/components/ChatbotTeaser/index.js`
- [X] T009 Create styling for ChatbotTeaser component at `website/src/components/ChatbotTeaser/styles.module.css`

## Phase 3: User Story 1 - First-Time Visitor (Priority: P1)

Goal: Enable first-time visitors to quickly understand the course, see module structure, and find a clear starting point

**Independent Test Criteria**: The homepage clearly communicates the course value proposition, displays organized module information in card format, and provides a prominent "Start Here" button that leads directly to Module 1.

- [X] T010 [US1] Update hero section in `website/src/pages/index.js` with compelling course summary
- [X] T011 [US1] Implement primary "Start Here" CTA button in hero section pointing to Module 1
- [X] T012 [P] [US1] Create ModuleCard for Module 1 at `website/src/pages/index.js` with title and description
- [X] T013 [P] [US1] Create ModuleCard for Module 2 at `website/src/pages/index.js` with title and description
- [X] T014 [P] [US1] Create ModuleCard for Module 3 at `website/src/pages/index.js` with title and description
- [X] T015 [P] [US1] Create ModuleCard for Module 4 at `website/src/pages/index.js` with title and description
- [X] T016 [US1] Ensure all module cards link to their respective module index pages
- [X] T017 [US1] Add CourseMetaSection with quick-links to overview/outcomes/weekly/assessments/hardware
- [X] T018 [US1] Add ChatbotTeaser section noting "coming in Spec 011"
- [X] T019 [US1] Apply minimal styling to homepage layout in `website/src/pages/index.module.css`
- [X] T020 [US1] Test that clicking "Start Here" CTA navigates to Module 1 index page

## Phase 4: User Story 2 - Returning Learner (Priority: P2)

Goal: Enable returning learners to quickly access their progress and jump to different modules

**Independent Test Criteria**: The homepage provides clear navigation options for both continuing current progress and jumping to specific modules.

- [X] T021 [US2] Add "Continue Learning" section to homepage in `website/src/pages/index.js` (SKIPPED - out of scope for static site)
- [X] T022 [US2] Implement logic to show last visited location for returning users (SKIPPED - requires backend functionality)
- [X] T023 [US2] Add visual indicators to module cards showing completion/progress status (SKIPPED - out of scope for static site)
- [X] T024 [US2] Enhance module cards with progress tracking features (SKIPPED - requires backend functionality)
- [X] T025 [US2] Update CourseMetaSection with additional navigation options for returning learners (SKIPPED - out of scope for static site)

**Note**: Returning learner functionality has been deferred as it requires backend capabilities for user authentication and progress tracking, which is outside the scope of this static homepage redesign.

## Phase 5: User Story 3 - Mobile Learner (Priority: P3)

Goal: Ensure homepage is fully responsive and usable on both mobile and desktop screens

**Independent Test Criteria**: The homepage layout adapts appropriately to different screen sizes while maintaining all functionality and readability.

- [X] T026 [US3] Implement responsive grid layout for module cards in `website/src/pages/index.module.css`
- [X] T027 [US3] Add mobile-first CSS breakpoints for different screen sizes
- [X] T028 [US3] Optimize touch targets for mobile accessibility (minimum 44px)
- [X] T029 [US3] Test layout on mobile screen sizes (320px, 375px, 768px)
- [X] T030 [US3] Test layout on desktop screen sizes (1024px, 1366px, 1920px)
- [X] T031 [US3] Verify all interactive elements maintain accessibility on mobile
- [X] T032 [US3] Optimize typography scaling across different screen sizes

## Final Phase: Polish & Validation

Goal: Complete final styling, accessibility improvements, and validation

- [X] T033 Apply consistent spacing and typography across all homepage sections
- [X] T034 Ensure all components meet WCAG 2.1 AA contrast ratio requirements
- [X] T035 Add proper ARIA attributes for screen reader compatibility
- [X] T036 Optimize images and assets for fast loading
- [X] T037 Test keyboard navigation throughout the homepage
- [X] T038 Run accessibility audit and address any issues
- [X] T039 Validate all internal links work correctly with GitHub Pages base URL
- [X] T040 Run `npm run build` to ensure build process completes successfully
- [X] T041 Test homepage functionality in development server
- [X] T042 Verify all CTAs and navigation links route correctly
- [X] T043 Perform final mobile responsiveness check on multiple devices
- [X] T044 Validate that no broken links exist in the redesigned homepage
- [X] T045 Update any documentation reflecting the homepage changes

---

## Validation Checklist

### Mobile View
- [X] Layout is optimized for mobile with appropriate spacing
- [X] Touch targets are minimum 44px in size
- [X] Content is readable without horizontal scrolling
- [X] Navigation works properly on small screens

### Link Targets
- [X] "Start Here" CTA navigates to Module 1 correctly
- [X] All module cards link to their respective index pages
- [X] Course meta quick-links navigate to correct pages
- [X] All links respect the GitHub Pages base URL

### Build Validation
- [X] `npm run build` completes without errors (Note: Build has warnings about other pages but homepage builds correctly)
- [X] Production build loads correctly
- [X] All assets are properly included in the build
- [X] No broken links in the production build (Note: Homepage has no broken links, other pages have unrelated broken links)

### Accessibility
- [X] All interactive elements have sufficient color contrast (4.5:1 minimum)
- [X] Keyboard navigation works for all interactive elements
- [X] Proper semantic HTML structure with heading hierarchy
- [X] ARIA attributes properly implemented for screen readers