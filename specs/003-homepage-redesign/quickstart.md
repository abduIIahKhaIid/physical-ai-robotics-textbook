# Quickstart Guide: Homepage Redesign Implementation

## Overview
This guide provides step-by-step instructions to implement the homepage redesign for the Physical AI & Humanoid Robotics textbook website.

## Prerequisites
- Node.js v18+ installed
- npm or yarn package manager
- Git for version control
- Access to the project repository

## Setup Environment
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd physical-ai-robotics-textbook
   ```

2. Navigate to the website directory:
   ```bash
   cd website
   ```

3. Install dependencies:
   ```bash
   npm install
   ```

## Implementation Steps

### Step 1: Backup Current Homepage
Before making changes, create a backup of the current homepage:
```bash
cp src/pages/index.js src/pages/index.js.backup
cp src/pages/index.module.css src/pages/index.module.css.backup
```

### Step 2: Update Homepage Component
Replace the content of `src/pages/index.js` with the new implementation that includes:
- Updated hero section with course summary
- Module cards grid for Modules 1-4
- Enhanced "Start Here" CTA pointing to Module 1

### Step 3: Update Styles
Modify `src/pages/index.module.css` to include:
- Responsive grid layout for module cards
- Mobile-first CSS with appropriate breakpoints
- Enhanced styling for CTAs and sections

### Step 4: Create Module Cards Component (Optional)
If needed, create a new component at `src/components/ModuleCards/index.js` to handle the module card display logic separately from the main homepage.

### Step 5: Test Responsive Design
Verify the design works across different screen sizes:
- Desktop: 1920px, 1366px
- Tablet: 1024px, 768px
- Mobile: 375px, 320px

### Step 6: Accessibility Testing
Ensure the homepage meets accessibility requirements:
- All interactive elements have proper ARIA labels
- Sufficient color contrast ratios
- Keyboard navigation works properly
- Screen reader compatibility

## Running the Development Server
To preview changes during development:
```bash
npm run start
```

This will start the Docusaurus development server at http://localhost:3000

## Build and Validation
To build the static site and validate the changes:
```bash
npm run build
```

This creates a production-ready build in the `build/` directory.

## Link Validation
To verify all internal links work correctly:
```bash
npm run serve
```

Then visit http://localhost:3000 to verify all navigation works correctly with the GitHub Pages base URL.

## Common Issues and Solutions

### Issue: Base URL Problems
Solution: Ensure all internal links use Docusaurus' Link component which automatically handles the base URL correctly.

### Issue: Responsive Design Breaking
Solution: Check that CSS media queries are properly defined and module cards collapse appropriately on smaller screens.

### Issue: Missing Module Content
Solution: Verify that module index pages exist and the navigation paths are correct.

## Next Steps
After successful implementation:
1. Run all tests to ensure nothing is broken
2. Verify mobile responsiveness
3. Test with accessibility tools
4. Get design review approval
5. Prepare for deployment