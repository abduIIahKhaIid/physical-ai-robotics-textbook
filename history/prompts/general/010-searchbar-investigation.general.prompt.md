---
id: 010
title: Search Bar Investigation
stage: general
date: 2026-02-27
surface: agent
model: claude-sonnet-4-6
feature: none
branch: main
user: user
command: general
labels: ["search","docusaurus","debug"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - website/docusaurus.config.js
tests:
  - none
---

## Prompt

why the searchbar can't work properly fix it and work properly

## Response snapshot

Investigated the search bar. The plugin `@easyops-cn/docusaurus-search-local` is correctly installed and configured. The `search-index.json` is generated during `npm run build`. The root cause is that this plugin does NOT work in dev mode (`npm start`) — only after a production build. No code changes needed; this is expected plugin behavior. Users should run `npm run build && npm run serve` to test search locally.

## Outcome

- ✅ Impact: No code changes needed; clarified expected behavior of search plugin
- 🧪 Tests: Build succeeds, search-index.json generated
- 📁 Files: No files modified
- 🔁 Next prompts: If search fails on deployed site, investigate further
- 🧠 Reflection: Search only works post-build, not in dev mode

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
