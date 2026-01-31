# Documentation Inventory Report

**Date**: 2026-01-30
**Feature**: 001-sidebar-ia-redesign
**Inventory Tool**: Manual review of `website/docs/` directory

## Directory Structure

```
website/docs/
├── intro.md
├── course-overview/
│   ├── index.md
│   ├── learning-objectives.md
│   ├── syllabus.md
│   ├── assessments.md
│   └── hardware-requirements.md
├── module-1/
│   ├── index.md
│   ├── week-1/
│   │   └── index.md
│   ├── week-2/
│   │   └── index.md
│   └── week-3/
│       └── index.md
├── module-2/
│   ├── index.md
│   ├── week-4/
│   │   └── index.md
│   ├── week-5/
│   │   └── index.md
│   └── week-6/
│       └── index.md
├── module-3/
│   ├── index.md
│   ├── week-7/
│   │   └── index.md
│   ├── week-8/
│   │   └── index.md
│   ├── week-9/
│   │   └── index.md
│   └── week-10/
│       └── index.md
├── module-4/
│   ├── index.md
│   ├── week-11/
│   │   └── index.md
│   ├── week-12/
│   │   └── index.md
│   └── week-13/
│       └── index.md
└── hello.md
```

## Current Sidebar Doc ID Mapping

From `website/sidebars.js`:

### Top Level Items
- `'intro'` → `docs/intro.md` ✓ EXISTS

### Course Overview Category
- `'course-overview'` → `docs/course-overview/index.md` ✓ EXISTS
- `'learning-objectives'` → `docs/course-overview/learning-objectives.md` ✓ EXISTS
- `'syllabus'` → `docs/course-overview/syllabus.md` ✓ EXISTS
- `'assessments'` → `docs/course-overview/assessments.md` ✓ EXISTS
- `'hardware-requirements'` → `docs/course-overview/hardware-requirements.md` ✓ EXISTS

### Module 1 Category
- `'module-1/index'` → `docs/module-1/index.md` ✓ EXISTS
- `'module-1/week-1/index'` → `docs/module-1/week-1/index.md` ✓ EXISTS
- `'module-1/week-2/index'` → `docs/module-1/week-2/index.md` ✓ EXISTS
- `'module-1/week-3/index'` → `docs/module-1/week-3/index.md` ✓ EXISTS

### Module 2 Category
- `'module-2/index'` → `docs/module-2/index.md` ✓ EXISTS
- `'module-2/week-4/index'` → `docs/module-2/week-4/index.md` ✓ EXISTS
- `'module-2/week-5/index'` → `docs/module-2/week-5/index.md` ✓ EXISTS
- `'module-2/week-6/index'` → `docs/module-2/week-6/index.md` ✓ EXISTS

### Module 3 Category
- `'module-3/index'` → `docs/module-3/index.md` ✓ EXISTS
- `'module-3/week-7/index'` → `docs/module-3/week-7/index.md` ✓ EXISTS
- `'module-3/week-8/index'` → `docs/module-3/week-8/index.md` ✓ EXISTS
- `'module-3/week-9/index'` → `docs/module-3/week-9/index.md` ✓ EXISTS
- `'module-3/week-10/index'` → `docs/module-3/week-10/index.md` ✓ EXISTS

### Module 4 Category
- `'module-4/index'` → `docs/module-4/index.md` ✓ EXISTS
- `'module-4/week-11/index'` → `docs/module-4/week-11/index.md` ✓ EXISTS
- `'module-4/week-12/index'` → `docs/module-4/week-12/index.md` ✓ EXISTS
- `'module-4/week-13/index'` → `docs/module-4/week-13/index.md` ✓ EXISTS

### Tutorial Category
- `'hello'` → `docs/hello.md` ✓ EXISTS

## Missing Pages Analysis

Based on the planned sidebar structure, all required documentation already exists. No additional placeholder pages need to be created for the basic structure since the current documentation already includes:

- Course overview pages (overview, learning objectives, syllabus, assessments, hardware requirements)
- Module index pages for all 4 modules
- Week index pages for all weeks (1-13)

## Conclusion

The existing documentation structure is complete and matches the planned sidebar hierarchy. No additional placeholder pages need to be created for basic functionality, as all doc IDs referenced in the planned sidebar structure have corresponding files in the `website/docs/` directory.