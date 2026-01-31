# Data Model: Sidebar Information Architecture

## Entities

### Module Group
- **Entity name**: Module Group
- **Description**: Hierarchical container representing a major course division (Module 1-4) with associated metadata and child content items
- **Fields**:
  - id: string (unique identifier following format "module-X")
  - label: string (display name following format "Module X: [Title]")
  - type: string (constant value "category")
  - collapsible: boolean (always true for modules)
  - collapsed: boolean (false to show expanded by default)
  - items: Array<ModuleItem> (child content items including week categories and module index)

### Weekly Content Item
- **Entity name**: Weekly Content Item
- **Description**: Individual lessons or content pieces organized chronologically within a module
- **Fields**:
  - id: string (unique identifier following format "module-X/week-Y")
  - label: string (display name following format "Week Y: [Title]")
  - type: string (constant value "category")
  - items: Array<string> (references to doc files)
  - collapsible: boolean (always true for weeks)

### Utility Link
- **Entity name**: Utility Link
- **Description**: Special category of navigation items providing access to supplementary resources (setup, hardware, assessments)
- **Fields**:
  - id: string (unique identifier for the doc file)
  - label: string (descriptive display name)
  - type: string (constant value "doc")

### Course Meta Section
- **Entity name**: Course Meta Section
- **Description**: Container for introductory and administrative content
- **Fields**:
  - id: string (unique identifier)
  - label: string (display name like "Course Overview")
  - type: string (constant value "category")
  - items: Array<string> (references to meta content files)

## Relationships

### Module Group contains Weekly Content Items
- One Module Group entity contains multiple Weekly Content Item entities
- Relationship: One-to-Many (1:M)

### Module Group contains Module Index
- One Module Group entity contains one module index document
- Relationship: One-to-One (1:1)

### Course Meta Section contains Utility Links
- One Course Meta Section entity contains multiple Utility Link entities
- Relationship: One-to-Many (1:M)

## Validation Rules

### From Requirements
- FR-001: Module Group entities must be numbered sequentially (Module 1-4)
- FR-002: Weekly Content Items must be ordered chronologically within each module
- FR-003: Utility Link entities must be grouped in Course Meta Section
- FR-004: All entity IDs must correspond to existing doc files to prevent dead links
- FR-005: Hierarchy depth must not exceed 3 levels for readability
- FR-006: Label formats must follow consistent naming conventions
- FR-007: Ordering must be maintained alphabetically or chronologically

## State Transitions

### Collapsible Categories
- Initial State: Collapsed (collapsed: true) or Expanded (collapsed: false)
- Transition: User interaction toggles collapsed state
- Final State: Remains in user-selected state during session