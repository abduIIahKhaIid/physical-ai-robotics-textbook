# Implementation Plan: Embed ChatKit UI Widget

**Branch**: `011-embed-chatkit-ui` | **Date**: 2026-02-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/011-embed-chatkit-ui/spec.md`

## Summary

Embed a functional RoboTutor chat widget into the Docusaurus textbook site. The widget provides a floating chat button on every page, a responsive chat panel (floating overlay on desktop, full-screen drawer on mobile), SSE-streamed responses from the spec 010 FastAPI backend, and a text-selection-to-chat flow that sends highlighted passages in `selected_text_only` mode. The implementation uses Docusaurus theme swizzling (`src/theme/Root.js`) to inject a persistent React component tree with context-based state management, a custom SSE client using `fetch` + `ReadableStream`, and Tailwind CSS for responsive styling.

## Technical Context

**Language/Version**: TypeScript/JSX (React 18) within Docusaurus 3.6.3
**Primary Dependencies**: React 18, @docusaurus/core 3.6.3, @docusaurus/theme-common, Tailwind CSS 3.x (existing)
**Storage**: `sessionStorage` (session_id), React Context (conversation state) — no new server-side storage
**Testing**: Manual browser testing, Docusaurus build validation (`npm run build`), optional Playwright E2E
**Target Platform**: Modern browsers (Chrome, Firefox, Safari, Edge — last 3 versions), GitHub Pages static hosting
**Project Type**: Frontend-only (Docusaurus site extension)
**Performance Goals**: <300ms to open chat panel, <100ms tooltip appearance after text selection, zero build regressions
**Constraints**: No SSR access to `window`/`document` (must guard all browser APIs), static site on GitHub Pages with `baseUrl: /physical-ai-robotics-textbook/`, CORS to external backend
**Scale/Scope**: 1 global widget component, ~8 React component files, ~2 utility modules, ~1 CSS module

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Documentation-First | PASS | Spec written before implementation. Plan defines all artifacts. |
| II. Selected Text Only Mode | PASS | Widget sends `mode: "selected_text_only"` with `selected_text` payload. UI clearly distinguishes selected-text vs. normal mode. Spec FR-005/FR-006/FR-007 enforce this. |
| III. Test-First (NON-NEGOTIABLE) | PASS | Plan defines test approach: Docusaurus build verification (no SSR errors), manual QA script, and optional Playwright tests for widget interactions. |
| IV. Secure Architecture | PASS | No secrets in client code. Backend URL configured via env var / `customFields`. No auth tokens stored. Session IDs are anonymous UUIDs in `sessionStorage`. |
| V. Scalable Cloud Infrastructure | PASS | Client-side only — no server scaling concerns. Backend scaling is spec 010's responsibility. |
| VI. Modular Component Design | PASS | Widget is a self-contained component tree: ChatProvider → ChatButton + ChatPanel + SelectionTooltip. Each independently testable. |
| Performance Standards | PASS | Chat panel < 3s to first token (spec SC-002). Widget itself renders instantly (no heavy JS). Build does not increase page load time beyond 50KB JS. |
| Quality Gates | PASS | Build must pass (`npm run build`). No broken links. Widget visible on all doc pages. |

## Project Structure

### Documentation (this feature)

```text
specs/011-embed-chatkit-ui/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0: technology decisions
├── data-model.md        # Phase 1: client-side state model
├── quickstart.md        # Phase 1: local dev setup
├── contracts/
│   └── chat-client-api.yaml  # Phase 1: client→backend contract
└── tasks.md             # Phase 2 output (from /sp.tasks)
```

### Source Code (repository root)

```text
website/
├── docusaurus.config.js          # MODIFY: add customFields.chatApiUrl
├── src/
│   ├── theme/
│   │   └── Root.js               # CREATE: wraps app with ChatProvider
│   ├── components/
│   │   └── ChatWidget/
│   │       ├── index.tsx          # CREATE: main export — ChatButton + ChatPanel + SelectionTooltip
│   │       ├── ChatProvider.tsx   # CREATE: React Context provider for chat state
│   │       ├── ChatButton.tsx     # CREATE: floating FAB button (bottom-right)
│   │       ├── ChatPanel.tsx      # CREATE: message list + input area + header
│   │       ├── MessageBubble.tsx  # CREATE: single message (user or assistant)
│   │       ├── SelectionTooltip.tsx  # CREATE: "Ask about this" tooltip
│   │       ├── SelectedTextChip.tsx  # CREATE: quoted selection preview in chat input
│   │       ├── WelcomeScreen.tsx  # CREATE: empty state with suggestions
│   │       └── ChatWidget.css     # CREATE: widget-specific styles (Tailwind + custom)
│   └── utils/
│       ├── sseClient.ts          # CREATE: fetch-based SSE parser for POST requests
│       └── chatConfig.ts         # CREATE: config reader (backend URL, constants)
└── package.json                  # NO CHANGES (no new dependencies)
```

**Structure Decision**: All chat widget code lives under `website/src/components/ChatWidget/` as a self-contained module. The only file outside this directory is `src/theme/Root.js` (Docusaurus swizzle point) and `src/utils/` for shared utilities. No new npm dependencies — uses existing React 18, Tailwind CSS, and browser-native APIs.

## Architecture Design

### Component Tree

```
<Root>                          ← src/theme/Root.js (swizzled, never unmounts)
  <ChatProvider>                ← Context provider (session, messages, UI state)
    <BrowserOnly>               ← Docusaurus guard for SSR safety
      <ChatButton />            ← Fixed FAB, bottom-right, opens panel
      <ChatPanel />             ← Overlay panel (desktop) or drawer (mobile)
        <WelcomeScreen />       ← Empty state
        <MessageBubble />       ← Per-message display
        <SelectedTextChip />    ← Quoted selection preview
        <InputArea />           ← Textarea + send button
      <SelectionTooltip />      ← Appears near text selections on doc pages
    </BrowserOnly>
    {children}                  ← Original Docusaurus app tree
  </ChatProvider>
</Root>
```

### Runtime Flow: Normal Chat

```
1. User clicks ChatButton
   → ChatProvider: setIsOpen(true)

2. User types message, presses Enter
   → ChatPanel dispatches sendMessage(text)
   → ChatProvider:
     a. Add user message to messages[]
     b. Set uiState = 'loading'
     c. Call sseClient.postChat({
          message: text,
          session_id: sessionId (from sessionStorage, or null),
          mode: 'normal',
          source_doc_path: window.location.pathname,
          source_section: document.title
        })

3. SSE stream begins
   → event: token → append to streamingContent, set uiState = 'streaming'
   → event: token → append to streamingContent
   → event: done  → finalize message, store session_id, set uiState = 'idle'
   → event: error → set uiState = 'error', show error message

4. On done event:
   → Extract session_id → sessionStorage.setItem('robotutor_session_id', session_id)
   → Add assistant message (with citations) to messages[]
   → Clear streamingContent
```

### Runtime Flow: Selected-Text Chat

```
1. User selects text on a doc page (mouseup/touchend)
   → SelectionTooltip:
     a. Check window.getSelection().toString().length >= 10
     b. If yes: position tooltip at selection bounding rect
     c. If no: hide tooltip

2. User clicks "Ask about this" tooltip
   → ChatProvider:
     a. Store selectedText = { text, pageUrl, pageTitle }
     b. Set isOpen = true
   → SelectionTooltip: hide

3. Chat panel opens with SelectedTextChip showing quoted text
   → User types question, presses Enter
   → ChatProvider dispatches sendMessage(text) with:
     mode: 'selected_text_only'
     selected_text: selectedText.text
     source_doc_path: selectedText.pageUrl
     source_section: selectedText.pageTitle

4. User can dismiss SelectedTextChip:
   → ChatProvider: set selectedText = null
   → Subsequent messages use mode: 'normal'
```

### SSE Client Design (`sseClient.ts`)

```
function postChat(request: ChatRequest): AsyncIterable<SSEEvent>
  1. fetch(backendUrl + '/api/v1/chat', {
       method: 'POST',
       headers: { 'Content-Type': 'application/json', 'Accept': 'text/event-stream' },
       body: JSON.stringify(request)
     })
  2. If response.status === 429: yield { type: 'rate_limited', retryAfter: headers['Retry-After'] }
  3. If response.status >= 400: yield { type: 'error', code: status, message: statusText }
  4. Read response.body (ReadableStream) with TextDecoder
  5. Parse SSE format: split on '\n\n', extract 'event:' and 'data:' lines
  6. For each parsed event:
     - event: 'token' → yield { type: 'token', content: parsed.content }
     - event: 'done'  → yield { type: 'done', sessionId, messageId, citations }
     - event: 'error' → yield { type: 'error', code: parsed.code, message: parsed.message }
```

### Responsive Layout

| Breakpoint | Panel Style | Dimensions | Behavior |
|-----------|------------|------------|----------|
| >= 768px (desktop) | `position: fixed; bottom: 24px; right: 24px;` | `width: 400px; max-height: 600px;` | Floating panel, rounded corners, shadow |
| < 768px (mobile) | `position: fixed; inset: 0;` | `width: 100%; height: 100%;` | Full-screen drawer, slide-up animation |

Chat button: `position: fixed; bottom: 24px; right: 24px; width: 56px; height: 56px; border-radius: 50%; z-index: 9999;`

### Theme Integration

The widget CSS uses Docusaurus CSS custom properties and `[data-theme='dark']` selectors:

```css
.chat-panel {
  background: var(--ifm-background-color);
  color: var(--ifm-font-color-base);
  border: 1px solid var(--ifm-color-emphasis-300);
}

[data-theme='dark'] .chat-panel {
  border-color: var(--ifm-color-emphasis-700);
}
```

This automatically follows the theme toggle without needing the `useColorMode` hook (which requires Layout ancestor — Root.js renders above Layout).

### Accessibility

| Element | ARIA | Keyboard |
|---------|------|----------|
| Chat button | `role="button"`, `aria-label="Open RoboTutor chat"`, `aria-expanded` | Tab-focusable, Enter to open |
| Chat panel | `role="dialog"`, `aria-modal="true"`, `aria-label="RoboTutor Chat"` | Escape to close, focus trap |
| Message list | `role="log"`, `aria-live="polite"` | Auto-announces new messages |
| Input field | `aria-label="Type your message"` | Enter to send, Shift+Enter for newline |
| Selection tooltip | `role="tooltip"` | Appears on mouseup only (no keyboard trigger for selection — browser limitation) |
| Close button | `aria-label="Close chat"` | Tab-focusable, Enter/Escape to activate |

### GitHub Pages / baseUrl Considerations

- The backend URL is an **absolute URL** (e.g., `https://api.example.com`), not a relative path — no `baseUrl` conflict.
- All internal links within the widget (e.g., citation URLs) must prepend `baseUrl` if they reference doc pages. Use `useBaseUrl` hook from `@docusaurus/useBaseUrl`.
- The widget itself is bundled into the Docusaurus build — no separate static assets to serve.
- No `<script>` tags or CDN loads needed — pure React component.

### Error Handling Matrix

| Error Condition | Detection | UI Response |
|----------------|-----------|-------------|
| Network offline | `navigator.onLine` + `fetch` failure | Disable send, show "You're offline" banner |
| Backend 429 (rate limit) | HTTP status 429 + `Retry-After` header | Show "Rate limited. Try again in Xs." with countdown |
| Backend 5xx | HTTP status >= 500 | Show "Something went wrong" + Retry button |
| Backend unreachable | `fetch` throws `TypeError` | Show "Cannot reach server" + Retry button |
| SSE stream error event | `event: error` in stream | Show error message from backend |
| SSE stream interrupted | `ReadableStream` closes unexpectedly | If partial response exists, show it + "Response interrupted" |
| Session not found (404) | HTTP status 404 | Clear `session_id`, start new session |
| Validation error (422) | HTTP status 422 | Show "Invalid message" (unlikely with client validation) |

## Validation Steps

### Pre-merge Checks

1. **Build**: `cd website && npm run build` — zero errors, zero warnings related to chat widget
2. **SSR Safety**: Build succeeds without `window is not defined` or `document is not defined` errors
3. **Serve**: `npm run serve` — widget visible and functional on `localhost:3000/physical-ai-robotics-textbook/docs/`
4. **Responsive**: Chrome DevTools device mode — verify 320px, 768px, 1920px viewports
5. **Theme**: Toggle dark/light mode — widget updates without state loss
6. **Text selection**: Select text on 3 different doc pages — tooltip appears, chat opens with selection
7. **Keyboard**: Tab to chat button → Enter → type message → Enter → Escape to close
8. **Error states**: Start without backend → send message → verify error UI with retry

### Post-deploy Checks (GitHub Pages)

1. Chat button visible on `https://abduIIahKhaIid.github.io/physical-ai-robotics-textbook/docs/`
2. Chat panel opens on click
3. Backend CORS allows the GitHub Pages origin
4. SSE streaming works over HTTPS
5. No mixed content warnings
