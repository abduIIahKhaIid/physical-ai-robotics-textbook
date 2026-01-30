---
name: course-outline-to-sidebar
description: Convert course outlines (modules → weeks → chapters/lessons) into Docusaurus sidebar configuration and file/folder structure. Use when starting a textbook project, updating course structure, synchronizing sidebar with content changes, adding/removing chapters, or validating navigation consistency. Triggers on tasks like "create sidebar from course outline", "set up docs structure", "update course navigation", or "sync sidebar with chapters".
---

# Course Outline to Sidebar

Transforms hierarchical course outlines into production-ready Docusaurus documentation structure with automated sidebar generation, consistent naming conventions, and proper file organization.

## Workflow Overview

1. **Parse course outline** - Extract modules, weeks, and chapters
2. **Generate file structure** - Create organized docs directories
3. **Create sidebar config** - Build sidebars.ts with proper navigation
4. **Generate stub files** - Create placeholder MDX files with frontmatter
5. **Validate structure** - Ensure no dead links and consistent naming

## Step 1: Parse Course Outline

Extract the hierarchical structure from the course description:

```
Course Name
├── Module 1: Title
│   ├── Week 1-2: Topic
│   │   ├── Chapter: Specific Topic
│   │   └── Chapter: Another Topic
│   └── Week 3-5: Topic
│       └── Chapter: Topic
└── Module 2: Title
    └── Week 6-7: Topic
        └── Chapter: Topic
```

Use the outline parser script to extract this structure:

```bash
python3 scripts/parse_outline.py course_description.txt
```

This outputs a JSON structure suitable for sidebar generation.

## Step 2: Generate File Structure

Create the directory hierarchy following Docusaurus conventions:

**Naming Rules:**
- Use kebab-case for all file and folder names
- Module folders: `module-{n}-{slug}` (e.g., `module-1-robotic-nervous-system`)
- Week folders: `week-{n}` or `week-{n}-{m}` (e.g., `week-1-2` for multi-week)
- Chapter files: `{n}-{slug}.mdx` where n is sequential within the week

**Structure Pattern:**
```
docs/
├── index.mdx
├── introduction.mdx
├── module-1-topic/
│   ├── _category_.json
│   ├── week-1-2/
│   │   ├── _category_.json
│   │   ├── 1-first-chapter.mdx
│   │   └── 2-second-chapter.mdx
│   └── week-3-5/
│       ├── _category_.json
│       └── 1-chapter.mdx
└── module-2-topic/
    └── week-6-7/
        └── 1-chapter.mdx
```

Run the structure generator:

```bash
python3 scripts/generate_structure.py outline.json docs/
```

## Step 3: Create Sidebar Configuration

Generate `sidebars.ts` with proper TypeScript types:

```typescript
import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'index',
    'introduction',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      link: {
        type: 'generated-index',
        description: 'Learn about ROS 2 and robot control middleware.',
      },
      items: [
        {
          type: 'category',
          label: 'Weeks 1-2: Introduction',
          items: [
            'module-1-robotic-nervous-system/week-1-2/1-foundations',
            'module-1-robotic-nervous-system/week-1-2/2-sensor-systems',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
```

**Key Requirements:**
- Use `type: 'category'` for groupings
- Include `link.type: 'generated-index'` for module overview pages
- Use document IDs without `.mdx` extension
- Document IDs match file paths relative to `docs/` directory

Generate with:

```bash
python3 scripts/generate_sidebar.py outline.json > sidebars.ts
```

## Step 4: Generate Stub MDX Files

Create placeholder files with proper frontmatter:

```mdx
---
title: Chapter Title
sidebar_position: 1
---

# Chapter Title

[Content to be written]

## Learning Objectives

- Objective 1
- Objective 2

## Key Concepts

## Hands-on Activities

## Summary

## References
```

Use the stub generator:

```bash
python3 scripts/generate_stubs.py outline.json docs/
```

**Frontmatter Requirements:**
- `title` - Chapter display name
- `sidebar_position` - Sequential number within parent
- Optional: `sidebar_label`, `description`, `keywords`

## Step 5: Validate Structure

Run validation checks:

```bash
python3 scripts/validate_structure.py docs/ sidebars.ts
```

**Validation Checks:**
1. All sidebar references point to existing files
2. All MDX files have proper frontmatter
3. No orphaned files (files not in sidebar)
4. Consistent naming conventions throughout
5. Proper `_category_.json` files in each folder
6. Sequential `sidebar_position` values

## Category Metadata

Each folder needs `_category_.json`:

```json
{
  "label": "Week 1-2: Introduction",
  "position": 1,
  "link": {
    "type": "generated-index",
    "description": "Introduction to Physical AI concepts."
  }
}
```

Generate with:

```bash
python3 scripts/generate_categories.py outline.json docs/
```

## Common Operations

### Adding a New Chapter

1. Create the MDX file in the appropriate week folder
2. Update `sidebars.ts` to include the new file path
3. Update `sidebar_position` values if needed
4. Run validation

### Removing a Chapter

1. Delete the MDX file
2. Remove the entry from `sidebars.ts`
3. Adjust `sidebar_position` values
4. Run validation

### Restructuring Modules

1. Update outline JSON
2. Run full regeneration:
   ```bash
   python3 scripts/full_regenerate.py outline.json docs/
   ```
3. Review changes and validate

## Slug Generation Rules

Consistent slug generation from titles:

- Convert to lowercase
- Replace spaces with hyphens
- Remove special characters except hyphens
- Remove leading/trailing hyphens
- Collapse multiple consecutive hyphens

Example: "The Robotic Nervous System (ROS 2)" → `robotic-nervous-system-ros-2`

## Resources

### scripts/
Python utilities for automated sidebar and structure generation:

- `parse_outline.py` - Extract course structure from text
- `generate_structure.py` - Create directory hierarchy
- `generate_sidebar.py` - Build sidebars.ts config
- `generate_stubs.py` - Create placeholder MDX files
- `generate_categories.py` - Create _category_.json files
- `validate_structure.py` - Check for errors and inconsistencies
- `full_regenerate.py` - Complete regeneration workflow

### references/
- `docusaurus-sidebar-spec.md` - Complete Docusaurus sidebar API reference
- `naming-conventions.md` - Detailed naming and slug rules
- `examples.md` - Example course structures and configurations

### assets/
- `outline-template.txt` - Template for course outline format
- `frontmatter-template.md` - MDX frontmatter template