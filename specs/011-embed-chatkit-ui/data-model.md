# Data Model: Embed ChatKit UI Widget

**Feature**: 011-embed-chatkit-ui
**Date**: 2026-02-15

## Client-Side State Model

This feature is entirely client-side (React components in the Docusaurus site). There are no new database tables or server-side entities — the backend data model is already defined in spec 010. The model below describes the React state structure.

### ChatContext (React Context)

Top-level state held in `src/theme/Root.js` context provider, persists across SPA navigations.

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `isOpen` | `boolean` | `false` | Whether the chat panel is open |
| `sessionId` | `string \| null` | `null` | Current session UUID, set after first backend response |
| `messages` | `Message[]` | `[]` | Ordered list of conversation messages |
| `selectedText` | `TextSelection \| null` | `null` | Currently attached text selection context |
| `uiState` | `UIState` | `'empty'` | Current widget state |
| `streamingContent` | `string` | `''` | Accumulated content of message currently being streamed |

### Message

| Field | Type | Description |
|-------|------|-------------|
| `id` | `string` | Unique identifier (from backend `message_id` or client-generated) |
| `role` | `'user' \| 'assistant'` | Message author |
| `content` | `string` | Full message text |
| `citations` | `Citation[]` | Source citations (assistant messages only) |
| `selectedText` | `string \| null` | Selected text context that was attached (user messages only) |
| `timestamp` | `number` | Unix timestamp in milliseconds |

### Citation

| Field | Type | Description |
|-------|------|-------------|
| `title` | `string` | Source document title |
| `section` | `string` | Section within the document |
| `url` | `string` | Link to the source page |
| `module` | `string` | Module identifier |
| `chapter` | `string` | Chapter identifier |

### TextSelection

| Field | Type | Description |
|-------|------|-------------|
| `text` | `string` | Plain-text content of the selection |
| `pageUrl` | `string` | URL of the page where text was selected |
| `pageTitle` | `string` | Title of the page where text was selected |

### UIState (enum)

| Value | Description |
|-------|-------------|
| `'empty'` | No messages; show welcome screen with suggestions |
| `'idle'` | Has messages; waiting for user input |
| `'loading'` | Message sent, waiting for first token |
| `'streaming'` | Receiving tokens from backend |
| `'error'` | Last request failed; show error with retry option |
| `'rate_limited'` | Rate limit hit; show wait time |
| `'offline'` | No network connectivity |

### State Transitions

```
empty → loading (user sends first message)
idle → loading (user sends message)
loading → streaming (first token received)
loading → error (backend error or timeout)
loading → rate_limited (429 response)
streaming → idle (done event received)
streaming → error (error event received or network drop)
error → loading (user clicks retry)
rate_limited → idle (wait time expires)
offline → idle/empty (network restored)
* → offline (network lost)
```

## Storage

| Store | Key | Value | Lifetime |
|-------|-----|-------|----------|
| `sessionStorage` | `robotutor_session_id` | UUID string | Tab lifetime |
| React Context | `ChatContext` | Full state object | SPA session (never unmounts) |
