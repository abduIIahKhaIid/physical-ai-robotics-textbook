# ADR 001: Homepage Redesign Architecture for Physical AI & Humanoid Robotics Textbook

## Status
Accepted | Superseded by [ADR-NNN] | Deprecated | Draft

## Context
We need to redesign the homepage for the Physical AI & Humanoid Robotics textbook to improve user experience and provide better navigation to course modules. The current homepage is a simple hero section with generic features, lacking clear pathways to the 4 course modules and important course meta information.

## Decision
We will implement a structured homepage with the following components:
1. Enhanced hero section with compelling course summary
2. Module cards grid for Modules 1-4 with clear titles and descriptions
3. Primary "Start Here" CTA button linking to Module 1
4. Course meta quick-links section with access to overview, outcomes, weekly content, assessments, and hardware requirements
5. Chatbot teaser section indicating upcoming functionality
6. Mobile-responsive design following Docusaurus best practices
7. Accessibility compliance with WCAG 2.1 AA standards

## Alternatives Considered
1. **Minimal approach**: Only update the CTA text without structural changes - rejected as it doesn't address the core navigation issues
2. **Complete overhaul**: Redesign the entire site navigation structure - rejected as too broad for this focused improvement
3. **Carousel approach**: Use a rotating banner for module highlights - rejected due to accessibility concerns and reduced information density

## Consequences
### Positive
- Improved user onboarding for first-time visitors
- Clear navigation paths to all course modules
- Better organization of course meta information
- Mobile-responsive design enhances accessibility
- Consistent with Docusaurus framework patterns

### Negative
- Requires modification of existing homepage component
- May need additional CSS for responsive behavior
- Increases initial page load slightly due to additional components

## Implementation Notes
- Will modify `website/src/pages/index.js` and `website/src/pages/index.module.css`
- Will create new component files for ModuleCard, CourseMetaSection, and ChatbotTeaser
- Will maintain compatibility with GitHub Pages deployment and base URL
- Will follow Docusaurus component architecture patterns

## Decision Drivers
- User Story 1 priority (first-time visitor experience)
- Mobile responsiveness requirements
- Accessibility compliance needs
- Consistency with existing Docusaurus patterns