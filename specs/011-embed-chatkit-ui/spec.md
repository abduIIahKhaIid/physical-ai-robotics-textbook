# Feature Specification: Embed ChatKit UI Widget

**Feature Branch**: `011-embed-chatkit-ui`
**Created**: 2026-02-15
**Status**: Draft
**Input**: User description: "Embed ChatKit UI widget into Docusaurus site, including UX for selecting text and asking questions constrained to selection. Define client-server payload, UI states, and responsive behavior."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Opens Chat Widget on Any Page (Priority: P1)

A student reading any chapter of the Physical AI & Humanoid Robotics textbook sees a floating chat button in the bottom-right corner of every documentation page. When they click the button, a chat panel opens (either as an overlay panel on desktop or a full-screen drawer on mobile). The student types a question and receives a streamed response from RoboTutor, grounded in textbook content. The chat panel can be minimized or closed at any time.

**Why this priority**: The chat widget is the primary interaction surface — without it visible and functional on every page, users cannot access the chatbot at all. This is the foundational user journey.

**Independent Test**: Can be fully tested by navigating to any documentation page, clicking the chat button, sending a message, and verifying a streamed response appears in the chat panel.

**Acceptance Scenarios**:

1. **Given** a student is on any documentation page, **When** the page loads, **Then** a floating chat button is visible in the bottom-right corner and does not overlap with essential page content.
2. **Given** the chat button is visible, **When** the student clicks it, **Then** a chat panel opens with an empty conversation view and a text input field focused and ready for typing.
3. **Given** the chat panel is open, **When** the student types a question and submits it, **Then** the message appears in the chat panel as a user message, and a streamed response begins appearing token-by-token within 2 seconds.
4. **Given** the chat panel is open with an active conversation, **When** the student clicks the close/minimize button, **Then** the panel closes and the floating button returns, preserving the conversation state for when the panel is reopened.

---

### User Story 2 - Student Selects Text and Asks About It (Priority: P1)

A student reads a passage in a chapter, highlights a portion of the text on the page, and a contextual action appears (tooltip or floating button near the selection). The student clicks this action and the chat panel opens with the selected text pre-attached as context. The student types a question about the selection, and the response is constrained to only the selected passage — no additional textbook content is retrieved.

**Why this priority**: Selected-text-only mode is a core differentiating feature. It allows students to get targeted explanations of specific passages, making the chatbot contextually aware of what the student is reading. This must work independently of the general chat flow.

**Independent Test**: Can be fully tested by selecting text on a chapter page, triggering the selection action, sending a question, and verifying the response only references the selected content.

**Acceptance Scenarios**:

1. **Given** a student is on a documentation page, **When** they select (highlight) a portion of text, **Then** a contextual action element appears near the selection within 300ms (e.g., a small "Ask about this" tooltip or floating button).
2. **Given** the contextual action is visible, **When** the student clicks it, **Then** the chat panel opens with a visual indicator showing the selected text is attached as context (e.g., a quoted block or chip showing the selection preview).
3. **Given** the chat panel is open with selected text attached, **When** the student types a question and submits, **Then** the client sends the request with the selected text and `mode: "selected_text_only"` to the backend, and the response references only the selected passage.
4. **Given** the student has selected text and opened the chat, **When** they dismiss the selection context (remove the chip/quoted block), **Then** the chat reverts to normal full-book mode for subsequent messages.
5. **Given** the student selects text on a page, **When** the selection is shorter than 10 characters, **Then** the contextual action does NOT appear (to avoid accidental triggers on trivial selections).

---

### User Story 3 - Chat Widget Responsive Behavior (Priority: P2)

The chat widget adapts its layout and behavior based on the viewport size. On desktop (viewport width >= 768px), the chat panel appears as a side panel or overlay anchored to the bottom-right corner. On mobile (viewport width < 768px), the chat panel takes the full screen as a drawer overlay. The widget never obscures critical navigation elements.

**Why this priority**: Responsive behavior ensures the widget is usable across all devices. Students access textbook content on laptops, tablets, and phones; broken mobile UX would block a significant user segment.

**Independent Test**: Can be tested by resizing the viewport and verifying the chat panel adapts its layout at breakpoints.

**Acceptance Scenarios**:

1. **Given** the viewport width is >= 768px, **When** the student opens the chat panel, **Then** it renders as a floating panel (approximately 400px wide, up to 600px tall) anchored to the bottom-right corner, not covering the full screen.
2. **Given** the viewport width is < 768px, **When** the student opens the chat panel, **Then** it renders as a full-screen drawer overlay with a visible close button, and the user can still access the close action without scrolling.
3. **Given** the student has the chat panel open on desktop, **When** they resize the browser below 768px, **Then** the panel transitions to the mobile full-screen layout smoothly.
4. **Given** the chat panel is open on mobile, **When** the student taps outside the panel or uses the close button, **Then** the panel dismisses and the floating button becomes visible again.

---

### User Story 4 - Session Continuity Within a Browser Session (Priority: P2)

When a student opens the chat, sends messages, closes the panel, and later reopens it (without navigating away or refreshing), their conversation history is preserved. If the student navigates between pages within the Docusaurus site, the chat state persists across page transitions (SPA navigation).

**Why this priority**: Without session continuity, students lose their conversation every time they close the panel or navigate, creating a frustrating experience. This relies on client-side state management within the React component tree.

**Independent Test**: Can be tested by sending messages, closing the panel, navigating to another page, reopening the panel, and verifying messages are still present.

**Acceptance Scenarios**:

1. **Given** a student has an active conversation with 3 messages, **When** they close the chat panel and reopen it, **Then** all 3 messages are still displayed in the correct order.
2. **Given** a student has an active conversation, **When** they navigate from one chapter page to another (SPA navigation), **Then** the conversation state persists and the chat panel state (open/closed) is preserved.
3. **Given** a student has an active conversation, **When** they perform a full page refresh (hard reload), **Then** the conversation may be lost from the client but is retrievable from the backend if session persistence is available.

---

### User Story 5 - Loading, Error, and Empty States (Priority: P2)

The chat widget displays appropriate visual feedback for all states: loading (waiting for response), error (backend unreachable or rate-limited), empty (no messages yet), and streaming (response in progress). Users always know the system's current state.

**Why this priority**: Clear state communication prevents user confusion and abandonment. Without loading indicators and error messages, users may think the system is broken.

**Independent Test**: Can be tested by simulating backend delays, errors, and rate limits, then verifying the widget displays appropriate state indicators.

**Acceptance Scenarios**:

1. **Given** the student has submitted a message, **When** the response has not yet begun streaming, **Then** a loading indicator (typing indicator or spinner) is displayed in the chat panel.
2. **Given** the backend returns an error (500, network error), **When** the student sees the error, **Then** a user-friendly error message is displayed in the chat panel with a "Retry" option.
3. **Given** the backend returns a rate-limit response (429), **When** the student sees it, **Then** the message explains the rate limit and shows an approximate wait time.
4. **Given** the chat panel is opened for the first time with no messages, **Then** a welcome message or prompt suggestions are displayed (e.g., "Ask me about any topic in the textbook" with 2-3 suggested starter questions).
5. **Given** a response is streaming, **When** tokens are arriving, **Then** the response message grows progressively and the chat panel auto-scrolls to keep the latest content visible.

---

### Edge Cases

- What happens when the student selects text that spans multiple HTML elements (e.g., across paragraphs or code blocks)? The system MUST capture the full plain-text content of the selection using the browser's Selection API, stripping HTML tags.
- What happens when the student selects text inside a code block? The system MUST preserve the code text as-is (no reformatting) and include it in the selected text payload.
- What happens when the student selects text and then clicks elsewhere (deselects) before clicking "Ask about this"? The contextual action MUST disappear when the selection is cleared, and no stale selection should be sent.
- What happens when the backend is completely unreachable (no network)? The widget MUST display an offline indicator and disable the send button, re-enabling it when connectivity is restored.
- What happens when a Docusaurus theme toggle (dark/light mode) occurs while the chat panel is open? The widget MUST update its appearance to match the current theme without losing conversation state.
- What happens when very long messages are sent or received? Messages MUST be scrollable within the chat panel, and the input field MUST handle multi-line input (expandable textarea, not single-line).
- What happens when the student uses keyboard navigation (Tab, Enter, Escape)? The widget MUST be accessible: Tab focuses the chat button, Enter opens/sends, Escape closes the panel.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST render a floating chat button on every documentation page, positioned in the bottom-right corner with a fixed z-index above page content.
- **FR-002**: System MUST open a chat panel when the floating button is clicked, displaying a message list and text input area.
- **FR-003**: System MUST send user messages to the FastAPI chat backend endpoint and receive streamed responses via Server-Sent Events (SSE).
- **FR-004**: System MUST detect text selection on documentation pages and display a contextual "Ask about this" action near the selection when the selected text is 10 or more characters.
- **FR-005**: System MUST attach selected text as context when the user triggers chat from a text selection, sending it with `mode: "selected_text_only"` in the request payload.
- **FR-006**: System MUST display the selected text as a visible quoted block or chip in the chat panel so the user knows what context is attached.
- **FR-007**: System MUST allow the user to dismiss the selected text context, reverting to normal full-book query mode.
- **FR-008**: System MUST adapt the chat panel layout based on viewport width — floating panel on desktop (>= 768px), full-screen drawer on mobile (< 768px).
- **FR-009**: System MUST preserve conversation state across chat panel open/close cycles and across SPA page navigations within the same browser session.
- **FR-010**: System MUST display distinct UI states: empty (welcome + suggestions), loading (typing indicator), streaming (progressive token display), error (with retry), and rate-limited (with wait time).
- **FR-011**: System MUST auto-scroll the message list to the latest content during streaming responses.
- **FR-012**: System MUST support keyboard accessibility — Tab to focus chat button, Enter to open/submit, Escape to close panel.
- **FR-013**: System MUST respect the active Docusaurus theme (light/dark) and update widget appearance when the theme changes.
- **FR-014**: System MUST include page context metadata (current page URL and title) in each chat request so the backend can contextualize responses.

### Client-Server Payload Contract

**Chat Request** (client → server):

| Field           | Type   | Required | Description                                                              |
| --------------- | ------ | -------- | ------------------------------------------------------------------------ |
| `session_id`    | string | optional | Existing session identifier. Omitted for first message.                  |
| `message`       | string | required | The user's question text.                                                |
| `mode`          | string | required | Either `"normal"` or `"selected_text_only"`.                             |
| `selected_text` | string | optional | The highlighted passage text. Required when mode is `selected_text_only` |
| `page_url`      | string | optional | The current documentation page URL.                                      |
| `page_title`    | string | optional | The current documentation page title.                                    |

**Chat Response** (server → client, via SSE stream):

| SSE Event       | Payload                                    | Description                        |
| --------------- | ------------------------------------------ | ---------------------------------- |
| `message_start` | `{ session_id, message_id }`               | Sent once at stream start          |
| `token`         | `{ content }`                              | Individual token or text chunk     |
| `citation`      | `{ source_url, source_title, snippet }`    | Inline citation reference          |
| `message_end`   | `{ finish_reason }`                        | Signals response completion        |
| `error`         | `{ code, message, retry_after? }`          | Error information                  |

### Key Entities

- **Chat Session**: A conversation thread identified by `session_id`, containing an ordered sequence of messages. Created on first interaction, persisted on the server.
- **Message**: A single user or assistant message within a session. Contains role, content, timestamp, and optionally selected text context and citations.
- **Text Selection**: A user-highlighted passage from a documentation page, captured as plain text with source page metadata. Exists only on the client as transient state until sent with a message.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chat widget is visible and functional on 100% of documentation pages within the site.
- **SC-002**: Users can open the chat panel and receive a streamed response within 3 seconds of submitting a message (measuring time from submit to first token received, excluding backend processing).
- **SC-003**: Selected-text mode correctly captures the highlighted passage and constrains responses to that context in 95% of interactions (measured via QA testing across 20+ distinct selections).
- **SC-004**: The widget is fully usable on viewports from 320px to 2560px wide, with no overlapping elements or unusable controls at any size.
- **SC-005**: Conversation state persists across at least 10 consecutive page navigations within the same browser session without data loss.
- **SC-006**: All interactive elements (chat button, input field, close button, retry button) are reachable via keyboard navigation without a mouse.
- **SC-007**: Widget appearance matches the active site theme (light/dark) with no visual artifacts or flash of wrong theme on load.

## Assumptions

- The FastAPI chat backend (spec 010) is deployed and accessible at a configurable base URL.
- The backend SSE streaming endpoint follows the event schema defined in the payload contract above.
- The Docusaurus site uses the `@docusaurus/preset-classic` theme and supports React component swizzling (confirmed in existing codebase).
- The chat widget will be implemented as a React component injected via Docusaurus theme swizzling (e.g., `src/theme/Root.js` wrapper) to ensure it appears on every page.
- Selected text is captured using the browser's `window.getSelection()` API, which is supported in all modern browsers.
- The site is deployed as a static site on GitHub Pages; the chat widget communicates with an external backend API (CORS must be configured on the backend).
- No user authentication is required for the initial version — sessions are identified by a client-generated UUID stored in browser storage.
- The existing `ChatbotTeaser` component on the homepage is a static marketing element and is separate from this functional chat widget.

## Dependencies

- **Spec 010 (FastAPI Chat Backend)**: The chat widget depends on the backend being deployed and exposing the streaming chat endpoint.
- **Spec 009 (RAG Ingestion & Retrieval)**: The backend's ability to provide grounded answers depends on the RAG pipeline having indexed the textbook content.
