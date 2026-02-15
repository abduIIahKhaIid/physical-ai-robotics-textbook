# Feature Specification: FastAPI Chat Backend

**Feature Branch**: `010-fastapi-chat-backend`
**Created**: 2026-02-14
**Status**: Draft
**Input**: User description: "Design FastAPI backend for chatbot: endpoints, streaming strategy, Neon Postgres schema for users/sessions/messages, Qdrant query flow. Include security basics and rate limits. Must support selected-text-only mode."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Asks a Question via Chat (Priority: P1)

A student reading the Physical AI & Humanoid Robotics textbook opens the RoboTutor chatbot widget embedded in the Docusaurus site. They type a question (e.g., "What is sim-to-real transfer?") and submit it. The backend receives the question, retrieves relevant textbook passages from the vector store, generates a grounded answer using a language model, and streams the response back to the chat widget in real time, token by token. The answer includes inline citations linking back to the source chapters.

**Why this priority**: This is the core user-facing interaction. Without a working chat endpoint that retrieves, generates, and streams answers, the chatbot delivers zero value. Everything else builds on this flow.

**Independent Test**: Can be fully tested by sending a chat message to the endpoint and verifying that a streamed response is returned with relevant content and citations.

**Acceptance Scenarios**:

1. **Given** the backend is running and the vector store contains textbook content, **When** a student sends the message "What are the principles of Physical AI?", **Then** the backend streams a response grounded in Module 1 content with citations, and the full response completes within 10 seconds.
2. **Given** the backend is running, **When** a student sends a message, **Then** they see tokens appearing progressively (streaming), not a single delayed response.
3. **Given** the backend is running, **When** a student sends a question that is completely outside the textbook scope (e.g., "What is the GDP of Brazil?"), **Then** the backend responds explaining the question is outside the textbook's coverage and suggests related topics.
4. **Given** a student is in an active chat session with prior messages, **When** they send a follow-up question, **Then** the backend considers the conversation history (within the session) to maintain context continuity.

---

### User Story 2 - Selected-Text-Only Chat (Priority: P1)

A student highlights a specific passage in a chapter and sends a question about it. The backend receives both the question and the selected text, and responds using *only* the selected text as context — without retrieving additional content from the vector store. The response is still streamed in real time.

**Why this priority**: Co-equal with full-book chat because selected-text-only mode is a core differentiation feature. Students need targeted clarification on specific passages. This mode is architecturally distinct (bypasses vector search) and must be validated independently.

**Independent Test**: Can be fully tested by sending a message with selected text and verifying the response references only the selected content, never pulling in external information.

**Acceptance Scenarios**:

1. **Given** the backend is running, **When** a student sends a question with `selected_text` containing a paragraph about bipedal gait patterns and `mode` set to "selected_text_only", **Then** the response is derived exclusively from that paragraph.
2. **Given** a student sends a question with selected text, **When** the question is unrelated to the selection (e.g., selected text about kinematics, question about sensors), **Then** the response explains the question doesn't relate to the selected text and offers to switch to full-book mode.
3. **Given** a student sends a request in selected-text-only mode, **When** the selected text is extremely short (fewer than 10 words), **Then** the backend still answers from the selection but notes that a longer selection would enable more detail.

---

### User Story 3 - Session and Message Persistence (Priority: P2)

When a student starts a conversation, a session is created and their messages and the bot's responses are stored persistently. If the student returns later (same browser/device), their previous conversation history is available. Conversations are organized into sessions, each containing an ordered sequence of messages.

**Why this priority**: Persistence enables conversation continuity, usage analytics, and future features like conversation export. It's secondary to the core chat flow but essential for a production-quality experience.

**Independent Test**: Can be tested by creating a session, sending messages, then querying the session endpoint to verify all messages are returned in order.

**Acceptance Scenarios**:

1. **Given** a new user starts a chat, **When** they send their first message, **Then** a new session is created and both the user message and bot response are persisted to the database.
2. **Given** a user has an active session with 5 messages, **When** they request their session history, **Then** all 5 messages are returned in chronological order with correct roles (user/assistant), timestamps, and content.
3. **Given** a user has multiple sessions, **When** they list their sessions, **Then** sessions are returned ordered by most recent activity, each with a title and message count summary.
4. **Given** a session has not been accessed for more than 30 days, **When** the retention policy runs, **Then** the session and its messages are soft-deleted (marked inactive, not physically removed).

---

### User Story 4 - Rate Limiting and Abuse Prevention (Priority: P2)

The system protects itself from abuse by limiting the number of requests a user can make within a time window. Users who exceed the limit receive a clear message explaining the limit and when they can retry. Different rate limits apply to anonymous vs. identified users.

**Why this priority**: Without rate limiting, the system is vulnerable to abuse that could exhaust API quotas (embedding + LLM calls) and degrade service for legitimate users. Critical for any public-facing API.

**Independent Test**: Can be tested by sending requests above the configured limit and verifying the system returns appropriate rate-limit responses.

**Acceptance Scenarios**:

1. **Given** a rate limit of 20 messages per minute per user, **When** a user sends a 21st message within one minute, **Then** the backend returns a rate-limit error with a Retry-After header indicating when they can send again.
2. **Given** an anonymous user (no token), **When** they send messages, **Then** rate limits are applied based on their IP address with a lower limit (10 messages/minute).
3. **Given** a user hits the rate limit, **When** they wait for the rate-limit window to reset, **Then** they can resume sending messages normally.

---

### User Story 5 - Health Check and Operational Monitoring (Priority: P3)

Operators and monitoring systems can check whether the backend is healthy by calling a health endpoint. The health check verifies connectivity to dependent services (database, vector store) and reports status.

**Why this priority**: Essential for production readiness but not part of the user-facing experience.

**Independent Test**: Can be tested by calling the health endpoint and verifying it returns status of all dependencies.

**Acceptance Scenarios**:

1. **Given** the backend is running and all dependencies are healthy, **When** the health endpoint is called, **Then** it returns a success status with individual component statuses (database: ok, vector store: ok).
2. **Given** the database is unreachable, **When** the health endpoint is called, **Then** it returns a degraded status indicating the database is down, while still reporting other component statuses.

---

### Edge Cases

- What happens when the database is unreachable during a chat request? The system MUST still attempt to answer the question (retrieval + generation) and return the response, but MUST log the persistence failure and include a warning that the conversation may not be saved.
- What happens when the vector store is unreachable during a normal-mode query? The system MUST return a clear error message to the user explaining that search is temporarily unavailable, with an appropriate error status.
- What happens when streaming is interrupted mid-response (client disconnects)? The system MUST gracefully handle the disconnection, stop generation, and still persist any partial response that was generated.
- What happens when a session contains more messages than can fit in the LLM context window? The system MUST apply a truncation strategy (most recent N messages, or summarization) to fit within the context budget while preserving the latest conversation context.
- What happens when a user sends an empty message or whitespace-only message? The system MUST reject the message with a validation error before any processing occurs.
- What happens when the LLM provider is rate-limited or returns errors? The system MUST return a user-friendly error explaining that the service is temporarily busy and they should retry shortly.

## Requirements *(mandatory)*

### Functional Requirements

#### Chat Endpoints

- **FR-001**: System MUST provide a chat endpoint that accepts a user's message, an optional session identifier, and optional retrieval parameters, and returns a generated response grounded in the retrieved textbook content.
- **FR-002**: System MUST support streaming responses using Server-Sent Events (SSE), delivering response tokens progressively as they are generated rather than waiting for the full response.
- **FR-003**: System MUST support a "selected_text_only" mode where the user provides highlighted text along with their question, and the system responds using ONLY the provided text — bypassing vector store retrieval entirely.
- **FR-004**: System MUST support a "normal" mode where the user's question is embedded and searched against the full vector store, retrieving the top-K most relevant chunks as context for generation.
- **FR-005**: System MUST include conversation history from the current session as context when generating responses, enabling multi-turn conversations where follow-up questions reference prior exchanges.
- **FR-006**: System MUST apply grounding policies to all responses: in normal mode, answers cite textbook sources; in selected-text mode, answers reference only the provided text; questions outside scope are refused with helpful guidance.

#### Session and Message Management

- **FR-007**: System MUST create a new session when a user starts a conversation without providing an existing session identifier.
- **FR-008**: System MUST persist every user message and assistant response to the database, including timestamps, role (user/assistant), message content, and associated session identifier.
- **FR-009**: System MUST provide an endpoint to retrieve all messages within a given session, returned in chronological order.
- **FR-010**: System MUST provide an endpoint to list a user's sessions, ordered by most recent activity, with summary metadata (title, message count, last activity timestamp).
- **FR-011**: System MUST store additional metadata for each message when applicable: retrieval mode used (normal/selected_text_only), number of chunks retrieved, and selected text content (if provided).

#### Security and Rate Limiting

- **FR-012**: System MUST enforce rate limits on the chat endpoint, configurable per user tier (anonymous: 10 messages/minute, identified: 20 messages/minute).
- **FR-013**: System MUST identify users via a session token or API key passed in the request header. Anonymous users (no token) MUST still be rate-limited by IP address.
- **FR-014**: System MUST validate and sanitize all user inputs (message text, selected text, session IDs) to prevent injection attacks.
- **FR-015**: System MUST enforce CORS policies allowing requests only from the configured Docusaurus site origin.
- **FR-016**: System MUST NOT expose internal error details (stack traces, database errors) to users. All errors MUST be returned in a consistent format with a user-friendly message and an error code.

#### Operational

- **FR-017**: System MUST provide a health check endpoint that reports the status of the backend and its dependencies (database, vector store).
- **FR-018**: System MUST log all chat requests (excluding message content for privacy) with metadata sufficient for debugging: timestamp, session ID, retrieval mode, response latency, error codes.

### Key Entities

- **User**: A person interacting with the chatbot. Identified by a session token (if authenticated) or IP address (if anonymous). Key attributes: user identifier, creation timestamp, tier (anonymous/identified).
- **Session**: A conversation thread between a user and the chatbot. Key attributes: session identifier, user identifier, title (auto-generated from first message), creation timestamp, last activity timestamp, status (active/archived).
- **Message**: A single exchange within a session. Key attributes: message identifier, session identifier, role (user/assistant), content, timestamp, retrieval mode used, selected text (if applicable), metadata (chunk count, latency).
- **Rate Limit Record**: Tracks request counts per user/IP within time windows. Key attributes: identifier (user ID or IP), window start, request count.

## Scope

### In Scope

- Chat endpoint with streaming (SSE) and non-streaming response modes
- Full-book retrieval mode (via existing Qdrant pipeline) and selected-text-only mode
- Neon Postgres schema for users, sessions, and messages
- Session management (create, list, retrieve history)
- Rate limiting (per-user and per-IP)
- CORS configuration for the Docusaurus frontend
- Health check endpoint
- Input validation and error handling

### Out of Scope

- User authentication/registration system (OAuth, SSO, email/password login) — users are identified by tokens, not authenticated via credentials
- Admin dashboard or analytics UI
- File upload or image processing in chat
- WebSocket-based streaming (SSE only for this iteration)
- Conversation summarization or export features
- Multi-language support for the chat interface
- Push notifications
- Payment or subscription management

## Assumptions

- The existing RAG pipeline (spec 009) provides working retrieval via `rag.retriever.query_engine.retrieve()` and grounding via `rag.response.grounding.apply_grounding_policy()`, which this backend will call. The pipeline will be refactored to use Gemini embeddings (replacing OpenAI) as part of this feature.
- The Docusaurus frontend (ChatKit widget) will be developed separately and will communicate with this backend via HTTP/SSE.
- Neon Postgres is the database provider; connection pooling and SSL are handled by the Neon service.
- Google Gemini is the LLM and embedding provider, accessed via the `google-genai` SDK. The free-tier API provides sufficient quota for development and initial deployment.
- Rate limiting uses an in-memory sliding window for the MVP, with an option to move to a distributed store later.
- Session tokens are opaque bearer tokens generated by the backend — no external identity provider integration is required for the initial version.
- The backend will be deployed as a single containerized service initially (not horizontally scaled), simplifying session token and rate-limit state management.
- Switching from OpenAI to Gemini embeddings requires recreating the Qdrant collection with 768-dimension vectors and re-ingesting all textbook content.

## Dependencies

- **Spec 009 (RAG Ingestion & Retrieval)**: Provides the retrieval pipeline, data models, and grounding policies that this backend orchestrates. Will be refactored to use Gemini embeddings.
- **Neon Postgres**: Managed database for session and message persistence.
- **Qdrant Cloud**: Vector store queried by the retrieval pipeline (via spec 009). Collection must be recreated with 768-dimension vectors.
- **Google Gemini API**: LLM for text generation (`gemini-2.0-flash`) and embeddings (`gemini-embedding-001`), accessed via `google-genai` SDK.
- **Docusaurus Frontend**: The chat widget that consumes this backend's API (separate spec).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students receive the first token of a streamed response within 2 seconds of sending a message, and the full response completes within 15 seconds for 95% of queries.
- **SC-002**: In selected-text-only mode, 100% of responses reference only the provided text — zero contamination from external sources, verifiable by automated test.
- **SC-003**: Conversation history is correctly persisted and retrievable: a session with 20 messages returns all 20 in correct order with correct roles and timestamps, 100% of the time.
- **SC-004**: Rate limiting correctly blocks requests beyond the configured threshold: when a user sends more than the allowed number of messages per minute, 100% of excess requests are rejected with appropriate feedback.
- **SC-005**: The health endpoint correctly reports degraded status within 5 seconds when any dependency becomes unavailable.
- **SC-006**: The system handles 50 concurrent users chatting simultaneously without errors or response degradation beyond the defined latency targets.
