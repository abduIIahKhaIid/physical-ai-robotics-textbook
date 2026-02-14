---
id: 0005
title: Footer light mode text visibility fix
stage: green
date: 2026-02-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-homepage-redesign
branch: main
user: abduIIahKhaIid
command: direct-prompt
labels: ["footer", "light-mode", "css", "accessibility"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - website/src/css/custom.css
tests:
  - Docusaurus build passes successfully
---

## Prompt

in homepage why the footer in light mode text are not show enchance the footer accordingly

## Response snapshot

Root cause: The footer in `docusaurus.config.js` has `style: 'dark'` which renders all footer text in white/light colors by default. But `custom.css` overrides the footer background to a light gradient (`#f8fafc` to `#f1f5f9`) in light mode, resulting in white text on a white background (invisible).

Fix applied in `website/src/css/custom.css`:
1. Added light mode text colors for `.footer` (`#1e293b`), `.footer__title` (`#0f172a`), `.footer__link-item` (`#475569`), `.footer__link-item:hover` (`#2563eb`), `.footer__copyright` (`#64748b`)
2. Fixed GitHub SVG icon to use explicit fill colors per theme (`#475569` light, `#cbd5e1` dark) instead of `currentColor`
3. Added dark mode `.footer__copyright` color (`#94a3b8`) for consistency

## Outcome

- Impact: Footer text now visible and properly styled in both light and dark modes
- Tests: Docusaurus build passes successfully
- Files: website/src/css/custom.css
- Next prompts: Review other components for similar light/dark mode contrast issues
- Reflection: When overriding Docusaurus footer background but keeping `style: 'dark'`, text colors must also be explicitly overridden for light mode

## Evaluation notes (flywheel)

- Failure modes observed: White text on white background due to conflicting footer style and CSS background override
- Graders run and results (PASS/FAIL): Build PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
