---
id: 0007
title: Header Auth Buttons, Search, Footer Responsiveness
stage: green
date: 2026-02-20
surface: agent
model: claude-sonnet-4-6
feature: 013-authentication-and-onboarding
branch: 013-authentication-and-onboarding
user: human
command: implement
labels: ["navbar", "auth", "search", "footer", "ui"]
links:
  spec: specs/013-authentication-and-onboarding/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - .env.example
  - website/src/theme/Navbar/index.js
  - website/docusaurus.config.js
  - website/src/css/custom.css
  - website/package.json
tests:
  - npm run build (pass)
---

## Prompt

Implement the following plan:
1. Add Sign In / Get Started buttons to the header navbar
2. Install @easyops-cn/docusaurus-search-local for search in header
3. Improve footer responsiveness and update links to project-relevant content
4. Fix .env.example real secret with placeholder

## Response snapshot

Implemented all four changes:
1. **Navbar auth buttons**: Updated `website/src/theme/Navbar/index.js` to render Sign In / Get Started buttons (unauthenticated) or user avatar + name + dropdown (authenticated) using existing `useAuth()` hook, `LoginForm`, and `RegisterForm`. Dropdown forms appear on click, close on outside click or Escape.
2. **Local search**: Installed `@easyops-cn/docusaurus-search-local` and added it to `themes` array in `docusaurus.config.js` with `hashed: true` and `indexBlog: false`.
3. **Footer**: Replaced generic Docusaurus community links with project-relevant links (ROS 2 docs, NVIDIA Isaac Sim, Report an Issue). Added responsive CSS for mobile stacking.
4. **Secret fix**: Replaced real `BETTER_AUTH_SECRET` value in `.env.example` with placeholder.

Build passes successfully.

## Outcome

- Impact: UI improvements — auth accessible from navbar, search enabled, footer cleaned up
- Tests: npm run build passes
- Files: 5 files modified (Navbar/index.js, docusaurus.config.js, custom.css, package.json, .env.example)
- Next prompts: Test auth flow end-to-end with running auth-service
- Reflection: Auth buttons positioned absolutely within navbar wrapper to avoid modifying Docusaurus internals

## Evaluation notes (flywheel)

- Failure modes observed: None — build succeeded on first attempt
- Graders run and results (PASS/FAIL): Build PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Add mobile-friendly auth (sidebar integration)
