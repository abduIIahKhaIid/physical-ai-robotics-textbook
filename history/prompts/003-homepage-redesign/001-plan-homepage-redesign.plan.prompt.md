---
id: "001"
title: "plan-homepage-redesign"
stage: "plan"
date_iso: "2026-01-30"
surface: "agent"
model: "Sonnet 4.5"
feature: "001-homepage-redesign"
branch: "001-homepage-redesign"
user: "codespace"
command: "sp.plan"
labels:
  - "docusaurus"
  - "homepage"
  - "ui-ux"
  - "redesign"
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files_yaml:
  - "specs/001-homepage-redesign/plan.md"
  - "specs/001-homepage-redesign/research.md"
  - "specs/001-homepage-redesign/data-model.md"
  - "specs/001-homepage-redesign/quickstart.md"
  - "specs/001-homepage-redesign/contracts/homepage-component.yaml"
tests_yaml: []
---

# Plan: Homepage Redesign Implementation

## Technical Context

Language/Version: JavaScript/TypeScript with Node.js v18+ (based on Docusaurus v3.x requirements)
Primary Dependencies: Docusaurus v3.x, React 18+, @docusaurus/core, @docusaurus/preset-classic
Storage: Static files for content, no dynamic storage needed for homepage
Testing: Jest for unit tests, Cypress for end-to-end tests, Docusaurus built-in validation
Target Platform: Web browser (mobile and desktop), GitHub Pages deployment
Project Type: Static web documentation site
Performance Goals: Sub-3 second load time, responsive mobile experience, accessible navigation
Constraints: Must work with GitHub Pages baseUrl '/physical-ai-robotics-textbook/', responsive design for mobile-first approach, WCAG 2.1 AA compliance
Scale/Scope: Educational textbook site, expected low-medium traffic, mobile-responsive design for various screen sizes

## Summary

Redesign the homepage landing page for the Physical AI & Humanoid Robotics textbook to provide a modern, mobile-responsive experience with clear navigation to course modules. The implementation will replace the current simple hero + features layout with a more structured design featuring a course summary section, prominent module cards for Modules 1-4, and enhanced "Start Here" CTA. The redesign maintains Docusaurus best practices while improving user onboarding and navigation to course content, with particular attention to mobile responsiveness and accessibility requirements.

## Constitution Check

### Compliance Verification

**I. Documentation-First Approach** ✅
- Homepage redesign documented in feature specification before implementation
- All changes will be reflected in Docusaurus documentation structure
- User experience and design decisions documented in this plan

**II. Selected Text Only Answering Mode** ❌/N/A
- Not applicable to homepage redesign feature (this is for RAG chatbot functionality)

**III. Test-First (NON-NEGOTIABLE)** ⚠️ PARTIAL
- Unit tests needed for new React components (to be created)
- Integration tests needed for homepage functionality
- Manual testing plan included in validation strategy
- Automated tests to be created during implementation phase

**IV. Secure Architecture** ✅
- Homepage is static content, no security vulnerabilities introduced
- No credentials or sensitive data stored in implementation
- All configuration follows Docusaurus security patterns

**V. Scalable Cloud Infrastructure** ❌/N/A
- Not applicable to static homepage design (this is for backend infrastructure)

**VI. Modular Component Design** ✅
- Using Docusaurus component architecture patterns
- Component-based design with reusable elements
- Clear separation between homepage sections

### Gate Status: APPROVED WITH CONDITIONS
- Test-first requirement needs to be addressed during implementation (Phase 2 tasks)
- All other constitution requirements satisfied
- Proceeding with implementation while noting test creation as mandatory during task execution

## Prompt Text

```
Plan the homepage redesign implementation for Docusaurus: specify the exact file(s) to edit/create (e.g., src/pages/index.* and related CSS), define the homepage section layout and component structure, define the content sources (reuse existing meta page titles/links, module index links), define accessibility and responsiveness requirements (keyboard navigation, semantic headings, mobile breakpoints), and include a validation plan (local dev preview + npm run build + link checks). Acceptance: the plan is actionable at file level, produces a professional mobile-friendly landing page, and ensures every homepage CTA/link routes correctly under GitHub Pages baseUrl.
```

## Response Text

Completed comprehensive planning for the homepage redesign including:
- Technical context with JavaScript/TypeScript, Docusaurus v3.x, and React 18+
- Detailed project structure mapping the website directory
- Research document analyzing current homepage structure and requirements
- Data model defining ModuleCard, HomepageHero, and other key entities
- Quickstart guide for implementation
- Component contract for homepage functionality
- Constitution compliance verification