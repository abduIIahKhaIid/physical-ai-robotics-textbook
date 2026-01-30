# Data Model: Docusaurus Book Site for Physical AI & Humanoid Robotics

## Key Entities

### 1. Documentation Content
**Entity**: Documentation Content
- **Fields**:
  - `id`: String - Unique identifier for the document (auto-generated from filename/path)
  - `title`: String - Display title of the document
  - `slug`: String - URL-friendly slug for the document
  - `sidebar_label`: String - Label to display in sidebar navigation
  - `description`: String - Brief description of the document content
  - `tags`: Array<String> - Keywords/tags associated with the document
  - `authors`: Array<String> - Authors/contributors to the document
  - `date`: Date - Creation/modification date
  - `content`: String - Markdown content of the document
  - `hierarchy`: String - Hierarchical path (e.g., "module1/week2/lesson3")

**Validation Rules**:
- `title` is required and must be 1-100 characters
- `slug` is required and must follow URL-friendly format
- `content` is required and must be valid Markdown
- `hierarchy` must follow the format "module/week/lesson" pattern

### 2. Navigation Structure
**Entity**: Navigation Structure
- **Fields**:
  - `id`: String - Unique identifier for navigation item
  - `type`: String - Type of navigation item ('category', 'doc', 'link')
  - `label`: String - Display label for navigation item
  - `to`: String - Destination URL (for links)
  - `items`: Array<NavigationItem> - Child navigation items (for categories)
  - `collapsible`: Boolean - Whether category can be collapsed
  - `collapsed`: Boolean - Initial collapsed state

**Validation Rules**:
- `type` must be one of ['category', 'doc', 'link']
- For 'doc' type, `id` must correspond to an existing document
- For 'link' type, `to` must be a valid URL

### 3. Site Configuration
**Entity**: Site Configuration
- **Fields**:
  - `siteTitle`: String - Main title of the website
  - `tagline`: String - Subtitle/description of the website
  - `url`: String - Base URL of the website
  - `baseUrl`: String - Base URL path (for GitHub Pages subdirectory)
  - `organizationName`: String - GitHub organization/username
  - `projectName`: String - GitHub repository name
  - `favicon`: String - Path to favicon file
  - `trailingSlash`: Boolean - Whether to append trailing slashes to URLs

**Validation Rules**:
- All fields are required
- `url` must be a valid URL format
- `baseUrl` must start with '/' and end with '/'
- `trailingSlash` must be boolean

### 4. Theme Configuration
**Entity**: Theme Configuration
- **Fields**:
  - `navbar.title`: String - Title displayed in navigation bar
  - `navbar.logo.src`: String - Path to logo image
  - `navbar.items`: Array<Object> - Navigation items in header
  - `footer.style`: String - Style of footer ('dark', 'light', etc.)
  - `footer.links`: Array<Object> - Links to display in footer
  - `footer.copyright`: String - Copyright text to display

**Validation Rules**:
- `navbar.title` is required
- `footer.style` must be one of supported styles
- `footer.links` items must have valid URLs