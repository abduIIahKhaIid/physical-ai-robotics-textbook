# Research: Homepage Redesign for Physical AI & Humanoid Robotics Textbook

## Findings Summary

### 1. Current Homepage Structure
- **File**: `website/src/pages/index.js`
- **Components**: Uses Layout wrapper, HomepageHeader, and HomepageFeatures component
- **Styling**: Uses CSS modules (`index.module.css`)
- **Navigation**: Single "Start Learning" button linking to `/docs/course-overview`
- **Features**: Three feature cards showing Physical AI, Humanoid Robotics, and Advanced AI Integration

### 2. Module Structure Analysis
- **Modules**: 4 main modules (module-1 through module-4)
- **Organization**: Each module has an index page and 2-4 weeks of content
- **Entry Points**: Module index pages serve as starting points for each module
- **URL Pattern**: `/docs/module-{number}/` for each module

### 3. Docusaurus Configuration
- **Base URL**: `/physical-ai-robotics-textbook/` for GitHub Pages deployment
- **Navigation**: Sidebar-based with textbook structure
- **Responsive**: Built-in mobile-responsive design
- **Accessibility**: Docusaurus follows accessibility best practices

### 4. Requirements Analysis

#### A. Course Summary Section
- Need to create a compelling course summary highlighting Physical AI & Humanoid Robotics
- Should explain the relevance and learning objectives
- Must be visually engaging and informative

#### B. Module Cards Implementation
- Need to create 4 module cards for Modules 1-4
- Each card should show title, brief description, and progress indicators
- Should link directly to respective module index pages
- Responsive grid layout for different screen sizes

#### C. "Start Here" CTA
- Prominent call-to-action button
- Should navigate to Module 1 (most logical starting point)
- Path: `/docs/module-1/` instead of `/docs/course-overview/`

#### D. Mobile Responsiveness
- Docusaurus provides responsive foundation
- Need to ensure module cards stack properly on mobile
- Touch targets should be appropriately sized
- Navigation should remain intuitive on smaller screens

### 5. Technical Implementation Approach

#### A. Files to Modify/Create
- `website/src/pages/index.js` - Main homepage component
- `website/src/pages/index.module.css` - Homepage styles
- Potentially new components for module cards

#### B. Component Structure
- Hero section with course summary
- Module cards grid section
- Improved CTAs with module-specific navigation

#### C. Accessibility Requirements
- Semantic HTML headings (H1, H2, H3)
- Proper ARIA labels for interactive elements
- Keyboard navigation support
- Sufficient color contrast ratios (WCAG 2.1 AA)
- Screen reader compatibility

### 6. Content Sources
- Module titles and descriptions from existing module index files
- Course overview content from `/docs/course-overview.md`
- Learning objectives from `/docs/learning-objectives.md`

### 7. Link Routing Considerations
- All internal links must respect the GitHub Pages baseUrl
- Module links should follow pattern: `/physical-ai-robotics-textbook/docs/module-{number}/`
- Docusaurus Link component handles base URL automatically

### 8. Responsive Breakpoints
- Desktop: 1024px and above (4-column grid for modules)
- Tablet: 768px - 1023px (2-column grid for modules)
- Mobile: Below 768px (1-column stacked layout)

## Decision Log

### Decision: Module Card Content Source
**What was chosen**: Extract titles and descriptions from existing module index files
**Rationale**: Reuses existing well-written content, maintains consistency with course structure
**Alternatives considered**: Writing new copy specifically for homepage, pulling from sidebar config

### Decision: Navigation Starting Point
**What was chosen**: Direct navigation to Module 1 instead of course overview
**Rationale**: Aligns with "Start Here" concept, provides immediate entry to core content
**Alternatives considered**: Keeping course overview as starting point

### Decision: Component Structure
**What was chosen**: Extend existing component structure rather than rewrite completely
**Rationale**: Leverages existing Docusaurus patterns and styling, reduces risk of breaking existing functionality
**Alternatives considered**: Complete rebuild with custom layout system