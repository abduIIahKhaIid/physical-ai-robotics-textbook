# Research Findings: Docusaurus Book Site Setup

## Decision: Docusaurus Classic Preset with GitHub Pages Deployment
**Rationale**: The classic preset is the most appropriate for a book-style documentation site, and GitHub Pages deployment via GitHub Actions is the simplest and most reliable approach for static site hosting.

## Architecture Choices Made:

### 1. Docusaurus Preset
- **Choice**: Classic preset (`@docusaurus/preset-classic`) configured for docs-first approach
- **Rationale**: Best suited for educational content with clear navigation structure
- **Alternatives considered**:
  - Custom preset (more complex, unnecessary for basic book site)
  - Blog-focused preset (not appropriate for textbook content)

### 2. Package Manager
- **Choice**: npm (as specified in requirements)
- **Rationale**: Consistent with project constraints, simpler dependency management
- **Alternatives considered**:
  - Yarn (adds additional dependency, not required by constraints)

### 3. GitHub Pages Deployment Strategy
- **Choice**: GitHub Actions with `actions/deploy-pages@v4` (modern approach)
- **Rationale**: Official GitHub recommendation, cleaner workflow, no need for SSH keys
- **Alternatives considered**:
  - `peaceiris/actions-gh-pages` (older approach, requires token management)
  - Manual deployment (doesn't meet automation requirement)

### 4. Base URL Configuration
- **Choice**: `baseUrl: '/<REPO_NAME>/'` for repository-based sites, `baseUrl: '/'` for user/org sites
- **Rationale**: Required for proper routing on GitHub Pages when site is hosted in subdirectory
- **Details**: If repo is `<ORG>.github.io`, use `/`; if repo is `<ORG>/<REPO>`, use `/<REPO>/`

## Technical Implementation Details:

### GitHub Actions Workflow
The workflow will include:
1. `build` job that checks out code, sets up Node.js, installs dependencies with `npm ci`, builds the site with `npm run build`, and uploads the build artifact
2. `deploy` job that deploys the artifact to GitHub Pages using `actions/deploy-pages@v4`

### Docusaurus Configuration
Key configuration elements:
- `url`: GitHub Pages URL (e.g., `https://<org>.github.io`)
- `baseUrl`: Depends on repository name structure
- `projectName`: Repository name
- `organizationName`: Organization/username
- `trailingSlash`: Explicitly set to `false` or `true` for consistent routing
- `favicon`: Appropriate for educational content
- `themes`: Classic preset with documentation plugin

### Navigation Configuration
- Navbar with site title and documentation links
- Footer with copyright and additional links
- Sidebar for documentation organization (will be auto-generated from docs structure)

## Risks and Mitigations:

### 1. Base URL Routing Issues
- **Risk**: Incorrect baseUrl configuration causes broken links and 404s
- **Mitigation**: Explicitly test locally with `baseUrl` set to production value; use `npm run serve` after build

### 2. Trailing Slash Behavior
- **Risk**: Inconsistent trailing slash handling causes duplicate content/index.html issues
- **Mitigation**: Explicitly set `trailingSlash` to consistent value (recommended: `false`)

### 3. GitHub Actions Permissions
- **Risk**: Insufficient permissions prevent deployment
- **Mitigation**: Use required permissions (`pages: write`, `id-token: write`) and proper environment configuration

### 4. Build Failures
- **Risk**: Dependencies or configuration changes break the build process
- **Mitigation**: Use frozen lockfile (`npm ci`) and test build locally before pushing

## Key Files to Create/Edit:
1. `package.json` - Docusaurus dependencies and build scripts
2. `docusaurus.config.js` - Site configuration for GitHub Pages
3. `.github/workflows/deploy.yml` - GitHub Actions deployment workflow
4. `docs/` directory - Initial documentation structure
5. `sidebars.js` - Navigation sidebar configuration
6. `static/` directory - Static assets like favicon