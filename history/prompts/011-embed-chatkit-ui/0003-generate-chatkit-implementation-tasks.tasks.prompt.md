---
id: "0003"
title: "Generate ChatKit Implementation Tasks"
stage: tasks
date: 2026-02-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: 011-embed-chatkit-ui
branch: 011-embed-chatkit-ui
user: architect
command: /sp.tasks
labels: ["chatkit", "tasks", "implementation", "checklist"]
links:
  spec: specs/011-embed-chatkit-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/011-embed-chatkit-ui/tasks.md
tests:
  - none
---

## Prompt

Create an atomic tasks checklist to implement Spec 011: add tasks to implement the widget container component and embed it in the chosen Docusaurus location, implement environment-safe configuration for backend URL (dev/prod) compatible with GitHub Pages, implement session management (create/reuse session_id, optional token fetch), implement selection capture UX (highlight -> capture text -> show selection banner/preview -> enforce max length -> clear selection), implement chat request submission with the agreed payload (message + optional selected_text + session_id), implement UI states (loading/streaming, error, selection-mode active, selection missing/too long), implement responsive styling and accessibility (keyboard/focus), and add validation steps (npm run build, run site locally, confirm widget works on a built preview, and a post-deploy checklist to verify on GitHub Pages). Acceptance: tasks list exact files to create/edit and end with a clear verification checklist proving both normal chat and selection-only flows work. use context7

## Response snapshot

Generated 32 implementation tasks (T001-T032) organized across 8 phases plus a 20-item verification checklist. Phase 1 (Setup): 2 tasks for directory structure and Docusaurus config. Phase 2 (Foundational): 5 tasks for config reader, SSE client, base CSS, ChatProvider context, and Root.js swizzle. Phase 3 (US1 MVP): 6 tasks for ChatButton, MessageBubble, WelcomeScreen, ChatPanel, barrel export, auto-scroll. Phase 4 (US2 Selection): 4 tasks for SelectionTooltip, SelectedTextChip, integration, mode verification. Phase 5 (US3 Responsive): 3 tasks for responsive CSS, viewport detection, mobile button. Phase 6 (US4 Sessions): 2 tasks for persistence and stale recovery. Phase 7 (US5 States): 5 tasks for typing indicator, error, rate-limit, offline, scroll refinement. Phase 8 (Polish): 5 tasks for ARIA/keyboard, dark mode, build validation, serve test, deploy checklist. Dependency graph shows US1 is MVP; US2-US5 are independent after US1.

## Outcome

- Impact: Complete actionable task list for ChatKit UI widget — 32 tasks with exact file paths, every task executable by an LLM without additional context.
- Tests: No automated tests — validation via build checks (T030), serve test (T031), and 20-item verification checklist (V01-V20).
- Files: specs/011-embed-chatkit-ui/tasks.md
- Next prompts: /sp.analyze to cross-validate, then /sp.implement to begin coding
- Reflection: Mapped all 5 user stories from spec to independent phases. Each phase has a checkpoint. Dependency graph makes parallel opportunities explicit. The verification checklist covers both normal and selection-only flows.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): Format validation PASS — all 32 tasks follow checkbox+ID+label+path format
- Prompt variant (if applicable): standard /sp.tasks flow
- Next experiment: none
