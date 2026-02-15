---
id: "0001"
title: "Specify ChatKit Widget Embed"
stage: spec
date: 2026-02-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: 011-embed-chatkit-ui
branch: 011-embed-chatkit-ui
user: architect
command: /sp.specify
labels: ["chatkit", "widget", "docusaurus", "text-selection", "sse", "ui"]
links:
  spec: specs/011-embed-chatkit-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/011-embed-chatkit-ui/spec.md
  - specs/011-embed-chatkit-ui/checklists/requirements.md
tests:
  - none
---

## Prompt

Embed ChatKit UI widget into Docusaurus site, including UX for selecting text and asking questions constrained to selection. Define client-server payload, UI states, and responsive behavior. use context7

## Response snapshot

Created feature specification for embedding a ChatKit UI chat widget into the Docusaurus textbook site. The spec defines 5 user stories covering: (1) floating chat button on all pages with SSE streaming, (2) text-selection-to-chat flow with contextual tooltip and selected_text_only mode, (3) responsive behavior (desktop panel vs. mobile full-screen drawer), (4) session continuity across SPA navigation, and (5) loading/error/empty UI states. Includes a client-server payload contract with request/response SSE event schemas, 14 functional requirements, 7 measurable success criteria, 7 edge cases, and dependencies on specs 009 (RAG) and 010 (FastAPI backend). Quality checklist passed all items.

## Outcome

- Impact: New feature spec created for the client-side chat widget — the primary user interaction surface for the RoboTutor chatbot.
- Tests: No code tests — spec-only artifact.
- Files: spec.md, checklists/requirements.md
- Next prompts: /sp.clarify or /sp.plan to proceed to architecture planning
- Reflection: Used Context7 for Docusaurus swizzling patterns (Root.js wrapper, BrowserOnly, useIsBrowser) which informed assumptions. Referenced existing spec 010 payload contract to ensure alignment.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): Checklist validation PASS (all items checked)
- Prompt variant (if applicable): standard /sp.specify flow
- Next experiment: none
