---
id: "0002"
title: "Plan ChatKit Widget Implementation"
stage: plan
date: 2026-02-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: 011-embed-chatkit-ui
branch: 011-embed-chatkit-ui
user: architect
command: /sp.plan
labels: ["chatkit", "widget", "docusaurus", "architecture", "sse", "react"]
links:
  spec: specs/011-embed-chatkit-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/011-embed-chatkit-ui/plan.md
  - specs/011-embed-chatkit-ui/research.md
  - specs/011-embed-chatkit-ui/data-model.md
  - specs/011-embed-chatkit-ui/quickstart.md
  - specs/011-embed-chatkit-ui/contracts/chat-client-api.yaml
tests:
  - none
---

## Prompt

Plan the ChatKit embed implementation: define the Docusaurus integration approach (which files/components to modify, e.g., theme layout, page wrappers, or a custom React component), define the UI/UX flow for normal chat and selection-only chat (selection capture → preview → submit → server response → render), define the exact payload contract and error handling between UI and FastAPI (including selected_text and used_mode), define session management (session_id storage, token flow if used), define styling/responsiveness approach (CSS modules/global styles, placement, z-index, mobile behavior), define accessibility requirements (ARIA, focus management, keyboard selection trigger), and define validation steps (local dev, build, deployed GitHub Pages verification). Acceptance: the plan is actionable at file level, respects Spec 010 endpoints, ensures selection-only mode is correctly triggered client-side, and prevents routing/baseUrl issues on GitHub Pages. use context7

## Response snapshot

Created implementation plan for the ChatKit UI widget embed. Key architecture decisions: (1) Use `src/theme/Root.js` swizzle for persistent global widget injection (never unmounts across navigations). (2) Custom fetch-based SSE client for POST requests (native EventSource only supports GET). (3) React Context in Root for chat state that survives SPA navigation. (4) CSS custom properties (`data-theme`) for theme integration instead of `useColorMode` hook (Root renders above Layout). (5) Text selection via `mouseup` + `window.getSelection()` with tooltip positioned at bounding rect. (6) No new npm dependencies — all built with existing React 18, Tailwind CSS, and browser APIs. (7) Client payload maps to existing backend ChatRequest model: `page_url` → `source_doc_path`, `page_title` → `source_section`. Generated 5 artifacts: plan.md, research.md (7 decisions), data-model.md (client state model with 7 UI states), contracts/chat-client-api.yaml, quickstart.md.

## Outcome

- Impact: Complete architecture plan for the ChatKit widget — 8 React components, 2 utility modules, 1 CSS file, 1 config modification.
- Tests: No code yet — plan defines validation steps (build check, SSR safety, responsive, keyboard, error states).
- Files: plan.md, research.md, data-model.md, quickstart.md, contracts/chat-client-api.yaml
- Next prompts: /sp.tasks to generate implementation tasks, then /sp.implement
- Reflection: Backend API analysis revealed exact field mapping (source_doc_path/source_section vs page_url/page_title) — avoided a misalignment. Root.js vs Layout swizzle was clarified via Context7 docs showing Root never unmounts, making it ideal for persistent state.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): Constitution check PASS (all 8 gates)
- Prompt variant (if applicable): standard /sp.plan flow
- Next experiment: none
