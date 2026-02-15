---
id: "0005"
title: "Implement ChatKit UI Widget"
stage: green
date: 2026-02-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: 011-embed-chatkit-ui
branch: 011-embed-chatkit-ui
user: architect
command: /sp.implement
labels: ["implementation", "chatkit", "widget", "docusaurus", "sse", "selection-mode"]
links:
  spec: specs/011-embed-chatkit-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - website/src/components/ChatWidget/ChatProvider.tsx
  - website/src/components/ChatWidget/ChatButton.tsx
  - website/src/components/ChatWidget/ChatPanel.tsx
  - website/src/components/ChatWidget/MessageBubble.tsx
  - website/src/components/ChatWidget/WelcomeScreen.tsx
  - website/src/components/ChatWidget/SelectionTooltip.tsx
  - website/src/components/ChatWidget/SelectedTextChip.tsx
  - website/src/components/ChatWidget/index.tsx
  - website/src/components/ChatWidget/ChatWidget.css
  - website/src/utils/sseClient.ts
  - website/src/utils/chatConfig.ts
  - website/src/theme/Root.js
  - website/docusaurus.config.js
tests:
  - npm run build (Docusaurus build validation - PASS)
---

## Prompt

Implement Spec 011 end-to-end: embed the ChatKit UI widget into the Docusaurus site at the agreed location, implement a complete UX for normal chat and for "selected text only mode" including reliable selection capture (highlight -> capture/trim/limit -> user-triggered ask), implement the client-server payload contract aligned to Spec 010 (message, session_id, optional selected_text/mode, error handling), implement session persistence (localStorage or server-issued token), implement required UI states (idle, typing/streaming, error, selection active, selection missing/too long) with responsive and accessible behavior, and validate locally and via build. Acceptance: widget loads on mobile and desktop, works under GitHub Pages routing, normal chat works, selection-only chat sends selected_text correctly and triggers server selection-only mode, no broken links/build errors are introduced, and `npm run build` passes. use context7

## Response snapshot

Implemented all 32 tasks (T001-T032) across 8 phases. Created 12 files: ChatProvider.tsx (React Context with full state management, SSE streaming, session persistence, offline/rate-limit handling, stale session recovery), ChatButton.tsx (floating FAB with ARIA), ChatPanel.tsx (message list with all 7 UI states, responsive mobile/desktop, Escape key, auto-scroll with near-bottom check), MessageBubble.tsx (user/assistant variants with citations), WelcomeScreen.tsx (suggestion chips), SelectionTooltip.tsx (window.getSelection + bounding rect positioning), SelectedTextChip.tsx (quoted preview with dismiss), index.tsx (barrel export), ChatWidget.css (complete styles with dark mode, responsive, animations), sseClient.ts (fetch-based SSE parser for POST), chatConfig.ts (config reader with constants), Root.js (Docusaurus swizzle with BrowserOnly guard). Build passes with zero SSR errors.

## Outcome

- Impact: Full ChatKit UI widget embedded in Docusaurus textbook site. All 5 user stories implemented: normal chat (US1), selected-text chat (US2), responsive layout (US3), session continuity (US4), and all UI states (US5). Widget persists across SPA navigation via Root.js swizzle.
- Tests: `npm run build` PASS (zero errors, zero broken links, zero SSR violations)
- Files: 12 files created/modified (9 components, 2 utils, 1 theme swizzle, 1 config update)
- Next prompts: Run `npm run serve` for manual QA, verify with backend running, deploy to GitHub Pages
- Reflection: Building all components with their full feature set upfront (rather than stubbing and iterating) was efficient since the spec, plan, and data model were comprehensive. The BrowserOnly guard in Root.js + lazy require pattern prevented all SSR issues cleanly.

## Evaluation notes (flywheel)

- Failure modes observed: None - build passed on first attempt after all components created
- Graders run and results (PASS/FAIL): Docusaurus build PASS, all 32 tasks completed
- Prompt variant (if applicable): standard /sp.implement flow
- Next experiment: none
