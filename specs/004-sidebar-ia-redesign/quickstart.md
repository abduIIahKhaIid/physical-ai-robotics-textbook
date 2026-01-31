# Quickstart Guide: Sidebar Information Architecture

## Overview
This guide provides the essential steps to implement the redesigned sidebar information architecture for the Physical AI & Humanoid Robotics textbook.

## Prerequisites
- Node.js v18+ installed
- Docusaurus project set up
- Access to the `website/sidebars.js` file
- Understanding of Docusaurus sidebar configuration format

## Step 1: Examine Current Sidebar Structure
First, review the existing `website/sidebars.js` file to understand the current configuration:

```javascript
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
export default {
  tutorialSidebar: [
    // Current structure here
  ],
};
```

## Step 2: Plan the New Structure
The new sidebar will have the following hierarchy:
1. Course Overview (with utility links)
2. Module 1-4 (each with week breakdowns)
3. Tutorial section (if applicable)

## Step 3: Update sidebars.js File
Replace the content of `website/sidebars.js` with the new structure:

```javascript
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
export default {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Course Overview',
      items: [
        'course-overview',
        'learning-objectives',
        'syllabus',
        'assessments',
        'hardware-requirements'
      ],
    },
    {
      type: 'category',
      label: 'Module 1: Foundations of Physical AI',
      collapsible: true,
      collapsed: false,
      items: [
        'module-1/index',
        {
          type: 'category',
          label: 'Week 1: Introduction to Physical AI',
          items: [
            'module-1/week-1/index'
          ]
        },
        {
          type: 'category',
          label: 'Week 2: Sensorimotor Learning',
          items: [
            'module-1/week-2/index'
          ]
        },
        {
          type: 'category',
          label: 'Week 3: Embodied Cognition',
          items: [
            'module-1/week-3/index'
          ]
        }
      ],
    },
    // Continue with Modules 2-4 in the same pattern...
  ],
};
```

## Step 4: Verify Doc IDs
Ensure all doc IDs referenced in the sidebar exist in the `website/docs/` directory:
- Check that `docs/intro.md` exists
- Check that `docs/course-overview.md` exists
- Check that `docs/module-1/index.md` exists
- And so on for all referenced files

## Step 5: Create Missing Files (If Needed)
If any doc IDs don't have corresponding files, create them:
```bash
# Example: Create missing module file
touch website/docs/module-1/week-1/index.md
```

## Step 6: Test the Changes
Start the development server to test the new sidebar:
```bash
cd website
npm start
```

## Step 7: Build and Validate
Build the site to ensure no errors:
```bash
npm run build
```

## Validation Checklist
- [ ] All sidebar links navigate to correct pages
- [ ] No broken links or 404 errors
- [ ] Hierarchy is readable and intuitive
- [ ] Naming conventions are consistent
- [ ] Week items are properly nested under modules
- [ ] Utility links are accessible
- [ ] Mobile responsiveness is maintained

## Troubleshooting
- If a link gives a 404 error, verify the doc ID matches an existing file
- If the sidebar doesn't render properly, check the JavaScript syntax
- If categories don't collapse/expand, ensure `collapsible: true` is set