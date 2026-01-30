---
id: 1
title: Build Hackathon Constitution
stage: constitution
date: 2026-01-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: claude
command: 'Build a constitution for a hackathon project: Docusaurus textbook deployed on GitHub Pages + embedded RAG chatbot. Required stack: FastAPI backend, Neon Postgres, Qdrant Cloud, OpenAI Agents SDK + ChatKit UI embed. Hard requirement: "selected text only" answering mode. Define: repo structure, definition of done, testing/build rules, branching (one spec per branch), documentation style, security basics (no secrets in repo)'
labels: [constitution, hackathon, ai, textbook, rag]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```text
Build a constitution for a hackathon project: Docusaurus textbook deployed on GitHub Pages + embedded RAG chatbot. Required stack: FastAPI backend, Neon Postgres, Qdrant Cloud, OpenAI Agents SDK + ChatKit UI embed. Hard requirement: "selected text only" answering mode. Define: repo structure, definition of done, testing/build rules, branching (one spec per branch), documentation style, security basics (no secrets in repo)
```

## Response snapshot

Created a comprehensive constitution for the AI Robotics Textbook hackathon project that includes:

- Core principles emphasizing documentation-first approach, selected text only answering mode, test-first methodology, secure architecture, scalable infrastructure, and modular design
- Technical stack requirements covering all specified technologies (Docusaurus, GitHub Pages, FastAPI, Neon Postgres, Qdrant Cloud, OpenAI Agents SDK, ChatKit)
- Performance standards for page load times and response times
- Development workflow with branching strategy (one spec per branch), code review requirements, quality gates, and definition of done
- Governance rules to ensure compliance with the constitution

The constitution is versioned as 1.0.0 and follows all security best practices including no secrets in the repository.

## Outcome

- ✅ Impact: Created initial project constitution with comprehensive guidelines for the hackathon project
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: Continue with spec, plan, and task creation for the project
- 🧠 Reflection: The constitution covers all requirements specified by the user and establishes a solid foundation for the hackathon project

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating the feature specification based on the constitution