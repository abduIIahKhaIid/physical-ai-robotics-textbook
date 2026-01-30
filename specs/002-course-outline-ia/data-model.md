# Data Model: Docusaurus Course/Module Framework

## Key Entities

### 1. Course Module
**Entity**: Course Module
- **Fields**:
  - `id`: String - Unique identifier for the module (e.g., "module-1", "introduction-to-robotics")
  - `title`: String - Display title of the module
  - `description`: String - Brief description of the module content
  - `duration`: String - Estimated time to complete (e.g., "2 weeks", "10 hours")
  - `prerequisites`: Array<String> - Other modules or knowledge required
  - `learning_outcomes`: Array<String> - IDs of learning outcomes covered
  - `weeks_count`: Number - Number of weeks in the module
  - `order`: Number - Position in the course sequence

**Validation Rules**:
- `id` is required and must be URL-friendly format
- `title` is required and must be 1-100 characters
- `order` must be unique across all modules
- `weeks_count` must be between 1 and 10

### 2. Learning Outcome
**Entity**: Learning Outcome
- **Fields**:
  - `id`: String - Unique identifier for the outcome
  - `title`: String - Brief title of the outcome
  - `description`: String - Detailed description of what student will achieve
  - `measurable_criteria`: Array<String> - Specific, measurable criteria
  - `assessment_methods`: Array<String> - How the outcome will be assessed
  - `related_modules`: Array<String> - IDs of related modules
  - `order`: Number - Position in the outcomes sequence

**Validation Rules**:
- `id` is required and must be URL-friendly format
- `title` is required and must be 1-100 characters
- `description` is required and must be 10-500 characters
- `order` must be unique across all outcomes

### 3. Weekly Unit
**Entity**: Weekly Unit
- **Fields**:
  - `id`: String - Unique identifier for the week (e.g., "week-1-module-1")
  - `module_id`: String - Reference to the parent module
  - `week_number`: Number - Week number within the module
  - `title`: String - Display title for the week
  - `objectives`: Array<String> - Learning objectives for the week
  - `activities`: Array<String> - Planned activities and assignments
  - `resources`: Array<String> - Links to additional resources
  - `duration`: String - Estimated time commitment

**Validation Rules**:
- `id` is required and must be URL-friendly format
- `module_id` must reference an existing module
- `week_number` must be unique within the module
- `title` is required and must be 1-100 characters

### 4. Assessment
**Entity**: Assessment
- **Fields**:
  - `id`: String - Unique identifier for the assessment
  - `title`: String - Display title of the assessment
  - `type`: String - Type of assessment (quiz, project, peer-review, etc.)
  - `weight`: Number - Percentage weight in overall course grade (0-100)
  - `due_date`: String - Due date or time frame
  - `criteria`: Array<String> - Grading criteria
  - `related_outcomes`: Array<String> - Learning outcomes being assessed
  - `submission_method`: String - How to submit (upload, link, etc.)

**Validation Rules**:
- `id` is required and must be URL-friendly format
- `type` must be one of ['quiz', 'project', 'assignment', 'peer-review', 'exam', 'presentation']
- `weight` must be between 0 and 100
- `title` is required and must be 1-100 characters

### 5. Hardware Requirement
**Entity**: Hardware Requirement
- **Fields**:
  - `id`: String - Unique identifier for the requirement
  - `category`: String - Category (computer, sensor, actuator, etc.)
  - `name`: String - Product/service name
  - `specifications`: Object - Technical specifications
  - `alternatives`: Array<Object> - Alternative options with similar specs
  - `cost_range`: String - Approximate cost range
  - `availability`: String - Where to obtain
  - `required_for`: Array<String> - Which modules or weeks require this

**Validation Rules**:
- `id` is required and must be URL-friendly format
- `category` is required and must be 1-50 characters
- `name` is required and must be 1-100 characters
- `specifications` must be a valid object with at least one property

### 6. Meta Page
**Entity**: Meta Page
- **Fields**:
  - `id`: String - Unique identifier for the page
  - `title`: String - Display title of the page
  - `type`: String - Type of meta page (overview, outcomes, weekly-breakdown, assessments, hardware-requirements, modules)
  - `description`: String - Brief description of the page content
  - `content_structure`: Object - Template structure with required sections
  - `navigation_position`: String - Position in the navigation hierarchy
  - `linked_pages`: Array<String> - IDs of related pages for cross-linking

**Validation Rules**:
- `id` is required and must be URL-friendly format
- `type` must be one of ['overview', 'outcomes', 'weekly-breakdown', 'assessments', 'hardware-requirements', 'modules', 'start-here']
- `title` is required and must be 1-100 characters
- `navigation_position` must follow the specified pathway order

### 7. Module Index Page
**Entity**: Module Index Page
- **Fields**:
  - `id`: String - Unique identifier for the module index page
  - `module_id`: String - Reference to the parent module
  - `title`: String - Display title of the module index
  - `sections`: Array<Object> - Standardized sections for the template
  - `navigation_links`: Array<String> - Links to child pages (weeks, resources)
  - `cross_references`: Array<String> - Links to related content elsewhere

**Validation Rules**:
- `id` is required and must be URL-friendly format
- `module_id` must reference an existing module
- `title` is required and must be 1-100 characters
- `sections` must contain the standard template sections

### 8. Course Navigation Pathway
**Entity**: Course Navigation Pathway
- **Fields**:
  - `sequence`: Array<String> - Ordered list of page types in the pathway
  - `entry_point`: String - Starting page for the pathway
  - `optional_jumps`: Array<Object> - Allowed navigation jumps between sections
  - `progress_indicators`: Array<String> - Pages that track user progress

**Validation Rules**:
- `sequence` must contain all required pathway elements in correct order
- `entry_point` must reference an existing page
- `sequence` must include ['start-here', 'overview', 'outcomes', 'weekly-breakdown', 'assessments', 'hardware-requirements', 'modules']