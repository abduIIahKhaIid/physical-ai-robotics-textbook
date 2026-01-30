<!-- Sync Impact Report:
Version change: N/A → 1.0.0
Added sections: All sections (initial constitution)
Modified principles: None
Removed sections: None
Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# AI Robotics Textbook Constitution

## Core Principles

### I. Documentation-First Approach
Every feature and functionality must be documented before implementation. All textbook content must be authored in Docusaurus format with clear, accessible explanations. Documentation serves as the primary contract for user experience and system behavior.

### II. Selected Text Only Answering Mode
The RAG chatbot must only answer questions based on selected text from the textbook. No hallucinations or external knowledge injection allowed. Answers must be grounded in the specific content the user has highlighted or referenced.

### III. Test-First (NON-NEGOTIABLE)
All features must have tests written before implementation. This includes backend API tests, frontend integration tests, and end-to-end user flows. TDD cycle strictly enforced: Red-Green-Refactor.

### IV. Secure Architecture
No secrets, credentials, or sensitive data stored in the repository. All configuration managed through environment variables. Client-side code must not expose backend connection details. Security scanning required for all PRs.

### V. Scalable Cloud Infrastructure
Backend services designed for horizontal scaling. Database connections optimized for concurrent users. Vector storage optimized for fast retrieval. Deployment pipeline supports automated scaling.

### VI. Modular Component Design
Frontend components must be reusable and independently testable. Backend services follow microservice principles with clear API boundaries. Clear separation between textbook content, RAG functionality, and chat interface.

## Technical Stack Requirements

### Required Technologies
- **Documentation Platform**: Docusaurus for textbook content
- **Deployment**: GitHub Pages for static content hosting
- **Backend API**: FastAPI for RAG functionality
- **Database**: Neon Postgres for metadata and user data
- **Vector Storage**: Qdrant Cloud for embeddings and document retrieval
- **AI Services**: OpenAI Agents SDK for natural language processing
- **UI Components**: ChatKit for chat interface embedding
- **Authentication**: Secure session management (if required)

### Performance Standards
- Page load time under 3 seconds for textbook content
- Chat response time under 2 seconds for selected text queries
- 99% uptime during hackathon judging period
- Support for concurrent users during demo

## Development Workflow

### Branching Strategy
- One feature branch per specification document
- Master branch protected with required reviews
- Feature branches named as `feature/[spec-name]`
- Pull requests must reference specific specification document

### Code Review Requirements
- At least one technical review for all changes
- Security review for any credential or API key changes
- Design review for UI/UX changes to maintain textbook quality
- Performance impact assessment for backend changes

### Quality Gates
- All tests must pass before merge
- Documentation updates required for new features
- Code coverage minimum 80% for new code
- Security scan passes with no critical vulnerabilities

### Definition of Done
- Feature implemented according to specification
- All tests passing (unit, integration, e2e)
- Documentation updated
- Security review completed
- Deployed to staging environment
- Manual QA performed
- Performance benchmarks met
- Code reviewed and approved

## Governance

All development must comply with this constitution. Changes to core principles require explicit approval from project leadership. New features must align with the selected text only answering mode requirement. This constitution supersedes all other practices and guidelines.

**Version**: 1.0.0 | **Ratified**: 2026-01-29 | **Last Amended**: 2026-01-29