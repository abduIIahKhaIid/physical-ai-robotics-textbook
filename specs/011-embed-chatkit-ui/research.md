# Research: Embed ChatKit UI Widget

**Feature**: 011-embed-chatkit-ui
**Date**: 2026-02-15

## R1: Docusaurus Integration Strategy for a Global Floating Widget

**Decision**: Use `src/theme/Root.js` swizzle to inject the chat widget at the top of the React tree.

**Rationale**:
- The `<Root>` component wraps the entire React tree above `<Layout>` and **never unmounts** across navigations (confirmed via Context7 Docusaurus docs). This makes it ideal for persistent UI like a chat widget.
- The existing codebase already uses swizzled theme components (`src/theme/Footer/index.js`, `src/theme/Navbar/index.js`), so this pattern is established.
- `Root.js` can hold React Context providers for chat state, ensuring state persists across SPA navigations without re-rendering or losing conversations.
- Alternative: Docusaurus `clientModules` plugin — rejected because it runs JS globally but doesn't provide a React mounting point for stateful components.
- Alternative: Footer wrapper — rejected because chat widget needs to float independently of page content flow.

**Alternatives considered**:
1. `clientModules` in `docusaurus.config.js` — doesn't provide React component lifecycle
2. Swizzle `Layout` — too invasive; wrapping would lose the original layout
3. Custom Docusaurus plugin with `contentLoaded` hook — unnecessary complexity for a React component

## R2: SSE Client Implementation for Browser

**Decision**: Use the native browser `EventSource` API for SSE streaming from the backend.

**Rationale**:
- The backend endpoint `POST /api/v1/chat` uses `sse-starlette`'s `EventSourceResponse` and yields dict events with `event` and `data` keys.
- However, the native `EventSource` API only supports GET requests. Since the backend uses POST, we must use `fetch()` with `ReadableStream` to parse SSE manually.
- The SSE protocol from the backend yields three event types: `token` (content chunk), `error` (failure), and `done` (completion with session_id, message_id, citations).
- Library alternatives (eventsource-polyfill, @microsoft/fetch-event-source) add dependencies. A lightweight custom SSE parser (~30 lines) using `fetch` + `ReadableStream` + `TextDecoder` is sufficient and dependency-free.

**Alternatives considered**:
1. `@microsoft/fetch-event-source` — good library but adds a dependency for a simple task
2. Native `EventSource` — only supports GET, not POST
3. WebSocket — backend doesn't support it; SSE is the established contract

## R3: Text Selection Capture and Tooltip Positioning

**Decision**: Use `document.addEventListener('mouseup')` + `window.getSelection()` API with a floating tooltip positioned via selection bounding rect.

**Rationale**:
- `window.getSelection()` is supported in all modern browsers and returns the highlighted text as plain string via `toString()`.
- The `Range.getBoundingClientRect()` method provides the pixel coordinates of the selection, allowing precise tooltip positioning.
- The tooltip ("Ask about this") should appear above the selection, offset by a small margin, and dismiss when the selection changes or is cleared.
- Minimum selection length: 10 characters (per spec FR-004) to avoid accidental triggers.
- Touch devices: `touchend` event mirrors `mouseup` for selection capture.

**Alternatives considered**:
1. MutationObserver on selection — overly complex; `mouseup` + `selectionchange` events are simpler
2. Custom highlight overlay — unnecessary; native browser selection provides visual feedback

## R4: Theme Integration (Light/Dark Mode)

**Decision**: Use `useColorMode` hook from `@docusaurus/theme-common` to detect and react to theme changes.

**Rationale**:
- Context7 docs confirm `useColorMode()` returns `{ colorMode, setColorMode }` — `colorMode` is `'light'` or `'dark'`.
- The hook requires the component to be a child of `Layout`, but since `Root.js` wraps above `Layout`, the widget component itself must be rendered **inside** the Layout tree or use CSS custom properties instead.
- Better approach: use the `data-theme` attribute on `<html>` element (set by Docusaurus) and define CSS variables in the widget styles. This avoids hook placement issues and works with CSS-in-JS or CSS modules.
- The existing `custom.css` already defines dark mode CSS using `[data-theme='dark']` selectors (confirmed in codebase).

**Alternatives considered**:
1. `useColorMode` hook — requires Layout ancestor; Root.js renders above Layout
2. `prefers-color-scheme` media query — doesn't respect Docusaurus user toggle
3. CSS custom properties referencing `data-theme` — chosen; works everywhere

## R5: Session Management (Client-Side)

**Decision**: Store `session_id` in `sessionStorage` (per-tab) and conversation messages in React state within the Root-level context.

**Rationale**:
- `sessionStorage` is scoped to a browser tab and cleared when the tab closes — appropriate for a per-visit conversation.
- React state in the Root context persists across SPA navigations (Root never unmounts).
- On first message, the backend creates a session and returns `session_id` in the `done` event. The client stores this and includes it in subsequent requests.
- `localStorage` was considered for cross-session persistence but is deferred — the spec states conversation MAY be lost on hard reload, with server-side retrieval as a future option.
- No auth tokens needed — sessions are anonymous (spec assumption).

**Alternatives considered**:
1. `localStorage` — persists beyond tab close; premature for MVP
2. Cookie-based session — unnecessary without authentication
3. URL-based session ID — leaks session in URLs; rejected

## R6: Responsive Layout Strategy

**Decision**: CSS `@media` queries with `position: fixed` for the chat panel, using Tailwind CSS utility classes (existing in project).

**Rationale**:
- The project already uses Tailwind CSS (confirmed: `tailwindcss` in devDependencies, PostCSS plugin configured).
- Desktop (>= 768px): Fixed panel 400px wide, max 600px tall, bottom-right corner.
- Mobile (< 768px): Full-screen overlay with `position: fixed; inset: 0`.
- Tailwind's responsive prefixes (`md:`) align with the 768px breakpoint.
- Z-index: Use `z-[9999]` to ensure the widget floats above Docusaurus navigation (`z-index` typically 100-200 in Docusaurus theme).

**Alternatives considered**:
1. CSS Modules — would work but Tailwind is already set up and used throughout
2. Styled-components — adds a dependency; not used in the project
3. Inline styles — harder to maintain; no responsive support

## R7: Backend API Alignment

**Decision**: The client will call `POST /api/v1/chat` matching the exact contract from spec 010.

**Rationale**:
- Backend endpoint confirmed at `backend/routers/chat.py:27` — `@router.post("/chat")` under prefix `/api/v1`.
- Request body `ChatRequest` (from `backend/models/api_models.py`) accepts: `message`, `session_id?`, `mode`, `selected_text?`, `source_doc_path?`, `source_section?`, `filters?`.
- The spec 011 payload adds `page_url` and `page_title` — these map to `source_doc_path` and `source_section` respectively in the existing backend model. No backend changes needed.
- SSE response events: `token` (with `content`), `error` (with `code`, `message`), `done` (with `message_id`, `session_id`, `citations`, `persistence_warning?`).
- CORS: Backend allows configurable origins via `CORS_ORIGINS` env var. The GitHub Pages domain must be added.

**Field mapping (spec 011 → backend)**:
| Spec 011 Field | Backend Field | Notes |
|---|---|---|
| `message` | `message` | Direct |
| `session_id` | `session_id` | UUID string → UUID |
| `mode` | `mode` | `"normal"` or `"selected_text_only"` |
| `selected_text` | `selected_text` | Direct |
| `page_url` | `source_doc_path` | Map client URL to doc path |
| `page_title` | `source_section` | Map page title to section name |
