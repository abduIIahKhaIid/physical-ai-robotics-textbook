# GitHub Actions Workflows

This directory contains the GitHub Actions workflows for the Physical AI & Humanoid Robotics textbook project. The workflows are designed to support the Spec-Driven Development (SDD) methodology and ensure high-quality documentation.

## Workflow Overview

### 1. Deploy to GitHub Pages (`.github/workflows/deploy.yml`)

This workflow handles the deployment of the Docusaurus-based textbook to GitHub Pages. It runs on pushes to the main branch and includes:

- Building the Docusaurus site
- Running linting checks
- Performing link validation
- Deploying to GitHub Pages

**Triggers:**
- Push to `main` and `007-author-module-3` branches
- Changes to website files

### 2. PR Validation (`.github/workflows/pr-validation.yml`)

This workflow validates pull requests to ensure code quality and consistency. It includes:

- Documentation structure validation
- Spell checking
- Broken link detection
- Security scanning
- Spec completeness validation

**Triggers:**
- Pull requests to `main` branch
- Changes to documentation, specs, or history files

### 3. SDD Process Validation (`.github/workflows/sdd-validation.yml`)

This workflow validates the Spec-Driven Development process adherence. It checks:

- Spec file completeness and structure
- Plan/spec consistency
- Task coverage and completeness
- Prompt History Record (PHR) structure
- Constitution compliance

**Triggers:**
- Pushes to `main` and `007-author-module-3` branches
- Changes to specs, .specify, or history directories

### 4. Documentation Quality Assurance (`.github/workflows/documentation-quality.yml`)

This workflow ensures documentation quality and consistency. It validates:

- Module structure and organization
- Frontmatter consistency
- Link integrity
- Build success
- Sidebar configuration
- Content quality metrics

**Triggers:**
- Pushes to `main` and `007-author-module-3` branches
- Changes to documentation files

### 5. Module 3 Content Validation (`.github/workflows/module-3-validation.yml`)

This workflow specifically validates the Module 3 content (NVIDIA Isaac Sim / Isaac ROS). It checks:

- Module 3 directory structure
- Content completeness and relevance
- Spec compliance
- Build integration
- Cross-module linking

**Triggers:**
- Pushes to `007-author-module-3` and `main` branches
- Changes to Module 3 documentation or specs

## Best Practices

1. **Branch Strategy**: Use feature branches with the naming convention `00X-feature-name` following the SDD approach
2. **Pull Requests**: All changes should go through pull requests with proper validation
3. **Documentation First**: Ensure all changes maintain documentation quality standards
4. **Spec Compliance**: Follow the SDD methodology with proper specs, plans, and tasks
5. **Module Organization**: Maintain consistent module and week structure

## Performance Considerations

- Workflows use concurrency groups to prevent conflicts
- Caching is enabled for faster builds
- Large operations run in parallel where possible
- Validation steps have appropriate timeouts

## Security

- CodeQL analysis for security scanning
- Secrets detection in documentation
- Permission minimization for workflow jobs