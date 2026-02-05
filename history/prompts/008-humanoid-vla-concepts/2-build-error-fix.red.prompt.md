---
id: 2
title: Fix Build Error for Physical AI & Humanoid Robotics Website
stage: red
date: 2026-02-05
surface: agent
model: Claude Sonnet 4.5
feature: build-fix
branch: 008-humanoid-vla-concepts
user: codespace
command: Fix build error task
labels: ["build", "error", "tailwind", "docusaurus", "configuration"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - /website/docusaurus.config.js
  - /website/postcss.config.js
  - /website/tailwind.config.js
  - package.json
tests:
  - Build completes successfully
  - No Tailwind CSS errors occur
  - Site renders correctly
---

# Fix Build Error for Physical AI & Humanoid Robotics Website

## Summary
Fixed the build error that occurred when running `npm run build` by addressing Tailwind CSS v4 compatibility issues with Docusaurus.

## Problem
The build was failing with the error "Cannot find module '@tailwindcss/postcss'" and "Cannot apply unknown utility class `from-blue-600`" when using Tailwind CSS v4.

## Solution
- Downgraded from Tailwind CSS v4 to v3 for better Docusaurus compatibility
- Updated Docusaurus config to use `require('tailwindcss')` instead of `require('@tailwindcss/postcss')`
- Updated PostCSS config file to reference `tailwindcss` instead of `@tailwindcss/postcss`
- Cleared Docusaurus cache to ensure changes took effect
- Fixed content patterns in Tailwind config to prevent node_modules scanning warnings

## Outcome
The build now completes successfully without errors. The website builds properly with Tailwind CSS v3 integrated correctly with Docusaurus.