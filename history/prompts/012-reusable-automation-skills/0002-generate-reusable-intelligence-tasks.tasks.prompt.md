---
id: "0002"
title: "Generate Reusable Intelligence Tasks"
stage: tasks
date: 2026-02-17
surface: agent
model: claude-sonnet-4-6
feature: 012-reusable-automation-skills
branch: 012-reusable-automation-skills
user: architect
command: /sp.tasks
labels: ["skills", "agents", "tasks", "checklist"]
links:
  spec: specs/012-reusable-automation-skills/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/012-reusable-automation-skills/tasks.md
tests:
  - null
---

## Prompt

Create an atomic tasks checklist to build the Reusable Intelligence pack: add tasks to create each required skill folder and SKILL.md with consistent format and practical examples, add tasks to create the six project-level subagents using Claude Code /agents (with names, descriptions, tool permissions, and system prompts), add tasks to write a routing policy doc (when to invoke which agent/skill across specs), add tasks to add at least one template file where useful (chapter template, sidebar skeleton, migration template), add tasks to document how to invoke skills and agents in Claude Code CLI, and add validation tasks (verify discovery via /skills and /agents, run a short demo scenario that uses at least 3 skills and 2 agents). Acceptance: tasks list exact file paths and produce a complete, documented RI pack that is usable by a beginner without guesswork.

## Response snapshot

Generated 51 tasks across 8 phases organized by 5 user stories. Phase 1: Fix trailing-space directory names (11 tasks). Phase 2: Create canonical templates (2 tasks). Phase 3-5: Standardize skills (US1-US2, 12 tasks) and agents (US3, 7 tasks) in parallel. Phase 6: Create routing guide at .claude/ROUTING.md (6 tasks). Phase 7: Verify template assets (5 tasks). Phase 8: Security scan, consistency audit, discoverability validation, and demo run (8 tasks). 67% of tasks are parallelizable. MVP = Phases 1-3 (15 tasks).

## Outcome

- Impact: 51 atomic tasks covering all 10 skills, 6 agents, routing guide, template assets, and validation
- Tests: Validation tasks T044-T051 serve as acceptance tests (secrets scan, structure audit, discoverability, demo)
- Files: tasks.md created at specs/012-reusable-automation-skills/tasks.md
- Next prompts: /sp.implement to begin executing tasks
- Reflection: Tasks are standardization/documentation work, not new code — low risk, high parallelism

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (task generation phase)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run /sp.analyze before /sp.implement to verify cross-artifact consistency
