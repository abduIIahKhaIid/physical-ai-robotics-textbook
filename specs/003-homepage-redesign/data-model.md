# Data Model: Homepage Redesign

## Key Entities

### 1. ModuleCard
Represents a single module card displayed on the homepage

**Fields:**
- id: string (unique identifier for the module)
- title: string (display title of the module)
- description: string (brief description of the module content)
- path: string (navigation path to the module index page)
- icon: string (icon identifier or SVG component name)
- position: number (display order on the homepage)
- isCompleted: boolean (indicates if user has completed this module)
- isNew: boolean (indicates if this is new content for the user)

**Relationships:**
- Belongs to Homepage
- Links to Module content in docs/

### 2. HomepageHero
Represents the hero section of the homepage containing the main headline and CTA

**Fields:**
- title: string (main headline text)
- subtitle: string (supporting tagline)
- ctaText: string (call-to-action button text)
- ctaPath: string (navigation path for the CTA)
- backgroundColor: string (CSS color value for the hero background)
- backgroundImage: string (optional background image path)

### 3. HomepageLayout
Defines the overall structure and responsive behavior of the homepage

**Fields:**
- sections: Array<Section> (ordered list of sections)
- mobileBreakpoint: string (CSS media query breakpoint)
- desktopBreakpoint: string (CSS media query breakpoint)
- accessibilityFeatures: Array<string> (accessibility features enabled)

### 4. Section
Represents a distinct section within the homepage

**Fields:**
- id: string (unique identifier)
- type: string ('hero' | 'features' | 'modules' | 'cta' | 'about')
- title: string (section heading)
- content: string | Array<Object> (section content)
- layout: string ('grid' | 'list' | 'columns')
- isVisible: boolean (whether section is displayed)

## Validation Rules

### ModuleCard Validation
- title: Required, max 100 characters
- description: Required, max 300 characters
- path: Required, must be a valid internal route
- position: Required, must be a positive integer
- id: Required, unique across all ModuleCards

### HomepageHero Validation
- title: Required, max 80 characters
- ctaText: Required, max 30 characters
- ctaPath: Required, must be a valid internal route

### Responsive Behavior
- Grid layouts must collapse to single column on mobile
- Font sizes must scale appropriately across screen sizes
- Touch targets must be minimum 44px for mobile accessibility

## State Transitions

### ModuleCard States
- Default: Normal display state
- Hover: Enhanced visual state for interactivity
- Active: When currently selected
- Completed: Visual indication of completion status

### Homepage States
- Loading: Initial render state
- Loaded: Full content displayed
- Mobile: Responsive mobile layout activated
- Desktop: Full desktop layout activated