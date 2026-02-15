# Tasks: Embed ChatKit UI Widget

**Input**: Design documents from `/specs/011-embed-chatkit-ui/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/chat-client-api.yaml

**Tests**: Not explicitly requested — validation via Docusaurus build checks and manual QA checklist.

**Organization**: Tasks grouped by user story. US1 is the MVP. US2 is co-P1 but independent. US3–US5 are P2 enhancements.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1–US5)
- All paths relative to repository root

---

## Phase 1: Setup

**Purpose**: Create directory structure and configure Docusaurus for the chat widget

- [x] T001 Create ChatWidget component directory at `website/src/components/ChatWidget/` and utils directory at `website/src/utils/`
- [x] T002 Add `customFields.chatApiUrl` to `website/docusaurus.config.js` — read from `process.env.REACT_APP_CHAT_API_URL` with fallback to a placeholder URL string `'https://your-backend.example.com'`. Add inside the existing `export default { ... }` block at the top level alongside `title`, `tagline`, etc.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core utilities and providers that ALL user stories depend on. No user story can begin until this phase completes.

- [x] T003 [P] Create config reader in `website/src/utils/chatConfig.ts`. Export: `getChatApiUrl(): string` — reads `useDocusaurusContext().siteConfig.customFields.chatApiUrl` or falls back to `window.__CHAT_API_URL__`. Also export constants: `MIN_SELECTION_LENGTH = 10`, `MAX_MESSAGE_LENGTH = 4000`, `MAX_SELECTED_TEXT_LENGTH = 10000`, `SESSION_STORAGE_KEY = 'robotutor_session_id'`.
- [x] T004 [P] Create SSE client in `website/src/utils/sseClient.ts`. Export `async function* postChat(backendUrl: string, request: ChatRequest): AsyncGenerator<SSEEvent>`. Implementation: (1) `fetch(backendUrl + '/api/v1/chat', { method: 'POST', headers: {'Content-Type':'application/json', 'Accept':'text/event-stream'}, body: JSON.stringify(request) })`. (2) Handle HTTP errors: 429 → yield `{type:'rate_limited', retryAfter}`, 404 → yield `{type:'session_not_found'}`, >=400 → yield `{type:'error', code, message}`. (3) Read `response.body` as `ReadableStream` with `TextDecoder`. (4) Parse SSE text format: split on `\n\n`, extract `event:` and `data:` lines. (5) Yield typed events: `token` → `{type:'token', content}`, `done` → `{type:'done', sessionId, messageId, citations}`, `error` → `{type:'error', code, message}`. Also export TypeScript types: `ChatRequest`, `SSEEvent`, `TokenEvent`, `DoneEvent`, `ErrorEvent`, `Citation`.
- [x] T005 [P] Create base CSS in `website/src/components/ChatWidget/ChatWidget.css`. Define classes: `.chat-button` (fixed, bottom-right, 56px circle, z-[9999], gradient blue-to-indigo background, white icon, shadow-lg, hover scale), `.chat-panel` (fixed, bottom-24 right-24, w-[400px] max-h-[600px], rounded-2xl, shadow-xl, flex column, z-[9998], background/color/border using Docusaurus CSS vars `var(--ifm-background-color)`, `var(--ifm-font-color-base)`, `var(--ifm-color-emphasis-300)`), `[data-theme='dark'] .chat-panel` overrides for dark borders. Include `.chat-panel-mobile` (position fixed inset-0, w-full h-full, no border-radius). Include `.message-list` (flex-1 overflow-y-auto), `.typing-indicator` (animated dots), `.selection-tooltip` (absolute, z-[10000], bg-blue-600 text-white rounded-lg px-3 py-1.5 shadow-lg cursor-pointer).
- [x] T006 Create ChatProvider in `website/src/components/ChatWidget/ChatProvider.tsx`. Implement React Context per data-model.md: state fields `isOpen`, `sessionId`, `messages`, `selectedText`, `uiState`, `streamingContent`. Export `ChatContext` and `useChatContext()` hook. Implement `ChatProvider` component that: (a) initializes `sessionId` from `sessionStorage.getItem('robotutor_session_id')` on mount, (b) exposes `togglePanel()`, `sendMessage(text: string)`, `setSelectedText(selection: TextSelection | null)`, `retryLastMessage()`. The `sendMessage` function: adds user message to `messages[]`, sets `uiState='loading'`, calls `postChat()` from sseClient.ts with the correct payload (mapping `page_url` → `source_doc_path`, `page_title` → `source_section`), iterates the async generator to handle `token`/`done`/`error` events updating state. On `done`: store `session_id` to `sessionStorage`, add assistant message with citations. On error: set `uiState='error'`. Uses `mode: selectedText ? 'selected_text_only' : 'normal'` and includes `selected_text: selectedText?.text`.
- [x] T007 Create `website/src/theme/Root.js` — Docusaurus swizzle point. Import `React` and `BrowserOnly` from `@docusaurus/BrowserOnly`. Import `ChatProvider` from `../components/ChatWidget/ChatProvider`. Wrap `{children}` with `<ChatProvider>`. Lazy-import the ChatWidget barrel inside `<BrowserOnly>` to avoid SSR issues with `window`/`document`. Structure: `<ChatProvider>{children}<BrowserOnly>{() => <ChatWidgetOverlay />}</BrowserOnly></ChatProvider>` where `ChatWidgetOverlay` is a lazy-loaded component that renders ChatButton + ChatPanel + SelectionTooltip.

**Checkpoint**: Foundation ready — `npm run build` in `website/` must pass with zero `window`/`document` SSR errors.

---

## Phase 3: User Story 1 — Student Opens Chat Widget on Any Page (P1) — MVP

**Goal**: Floating chat button visible on every doc page. Click opens panel. User can type a message, submit, and see a streamed response from the backend.

**Independent Test**: Navigate to any doc page → see chat button → click → panel opens → type question → streamed response appears token-by-token → close panel → button reappears.

### Implementation for User Story 1

- [x] T008 [P] [US1] Create ChatButton in `website/src/components/ChatWidget/ChatButton.tsx`. Render a `<button>` with class `chat-button`, a chat/message icon (use inline SVG or lucide-react `MessageCircle` already in project deps), `aria-label="Open RoboTutor chat"`, `aria-expanded={isOpen}`, `onClick={togglePanel}`. Hide button when `isOpen` is true. Apply Tailwind classes for the gradient background: `bg-gradient-to-r from-blue-600 to-indigo-600 hover:scale-110 transition-transform`.
- [x] T009 [P] [US1] Create MessageBubble in `website/src/components/ChatWidget/MessageBubble.tsx`. Accept props: `message: Message`. Render differently based on `role`: user messages right-aligned with blue background, assistant messages left-aligned with gray background. Display `content` as text. If `citations` array is non-empty, render them as small linked references below the message content (each citation: `<a href={citation.url}>{citation.title} — {citation.section}</a>`). If `selectedText` is present on a user message, show a small italic label "Re: selected text" above the content.
- [x] T010 [P] [US1] Create WelcomeScreen in `website/src/components/ChatWidget/WelcomeScreen.tsx`. Display when `uiState === 'empty'`. Show a bot icon, heading "Ask RoboTutor", subtitle "Ask me about any topic in the textbook", and 3 clickable suggestion chips: "What is sim-to-real transfer?", "Explain inverse kinematics", "How does SLAM work?". Each chip calls `sendMessage(chipText)` on click.
- [x] T011 [US1] Create ChatPanel in `website/src/components/ChatWidget/ChatPanel.tsx`. Render only when `isOpen` is true. Structure: (a) Header bar with "RoboTutor" title and close button (`aria-label="Close chat"`, calls `togglePanel()`). (b) Message list area (`role="log"`, `aria-live="polite"`) — map `messages[]` to `<MessageBubble />` components. If `uiState === 'empty'`, show `<WelcomeScreen />`. If `uiState === 'streaming'`, render an in-progress `<MessageBubble>` with `streamingContent` as content. (c) Input area: `<textarea>` with `aria-label="Type your message"`, placeholder "Ask a question...", auto-grow up to 4 lines. Send on Enter (not Shift+Enter). Send button with arrow icon. Disable textarea + button when `uiState` is `'loading'` or `'streaming'`. Import and apply `.chat-panel` class from ChatWidget.css.
- [x] T012 [US1] Create barrel export in `website/src/components/ChatWidget/index.tsx`. Export a `ChatWidgetOverlay` component that renders `<ChatButton />`, `<ChatPanel />`, and (placeholder for Phase 4) `<SelectionTooltip />`. Import ChatWidget.css here. This is the component lazy-loaded in Root.js's `<BrowserOnly>`.
- [x] T013 [US1] Wire auto-scroll in ChatPanel: add a `useEffect` that scrolls the message list container to the bottom whenever `messages.length` changes or `streamingContent` changes. Use `ref.current.scrollTop = ref.current.scrollHeight`.

**Checkpoint**: `npm run build` passes. `npm start` → open any doc page → chat button visible → click opens panel → type message → if backend running: streamed response appears; if backend not running: fetch error (Phase 7 will handle error UI).

---

## Phase 4: User Story 2 — Student Selects Text and Asks About It (P1)

**Goal**: Highlight text on a doc page → "Ask about this" tooltip appears → click → chat opens with selection attached → question is sent in `selected_text_only` mode → selection can be dismissed.

**Independent Test**: On a doc page, highlight >=10 chars → tooltip appears near selection → click tooltip → chat panel opens with quoted selection chip → type question → submit → Network tab shows `mode: "selected_text_only"` and `selected_text` in request body.

**Depends on**: Phase 3 (ChatPanel, ChatProvider)

### Implementation for User Story 2

- [x] T014 [P] [US2] Create SelectionTooltip in `website/src/components/ChatWidget/SelectionTooltip.tsx`. On mount, add `mouseup` and `touchend` event listeners on `document`. On event: (a) call `window.getSelection()`, (b) get text via `.toString()`, (c) if text.length >= `MIN_SELECTION_LENGTH` (from chatConfig): get bounding rect via `selection.getRangeAt(0).getBoundingClientRect()`, position tooltip above the rect center (`top: rect.top - 40 + window.scrollY`, `left: rect.left + rect.width/2`), show tooltip. (d) If text.length < `MIN_SELECTION_LENGTH`: hide tooltip. Add `selectionchange` listener that hides tooltip when selection is cleared. Tooltip content: "Ask about this" button with `role="tooltip"`, class `selection-tooltip`. On click: call `setSelectedText({ text: selectedString, pageUrl: window.location.pathname, pageTitle: document.title })` then `togglePanel()` (to open chat), then clear the browser selection and hide tooltip. Clean up all listeners on unmount.
- [x] T015 [P] [US2] Create SelectedTextChip in `website/src/components/ChatWidget/SelectedTextChip.tsx`. Accept no props — read `selectedText` from `useChatContext()`. Render a quoted block showing a truncated preview of `selectedText.text` (first 120 chars + "..."), a small "x" dismiss button. On dismiss click: call `setSelectedText(null)`. Style: left border blue accent, light blue/gray background, italic text, small font. Dark mode: use `[data-theme='dark']` overrides.
- [x] T016 [US2] Integrate selection components into ChatWidget. In `website/src/components/ChatWidget/index.tsx`: add `<SelectionTooltip />` to the `ChatWidgetOverlay` render. In `website/src/components/ChatWidget/ChatPanel.tsx`: render `<SelectedTextChip />` above the input area when `selectedText` is not null. Add a visual mode indicator in the header: show "Selection mode" badge when `selectedText` is present.
- [x] T017 [US2] Verify mode switching in ChatProvider: confirm `sendMessage()` in `website/src/components/ChatWidget/ChatProvider.tsx` correctly sets `mode: 'selected_text_only'` and includes `selected_text` when `selectedText` state is non-null, and sets `mode: 'normal'` with no `selected_text` when it is null. After sending a message in selection mode, do NOT auto-clear `selectedText` — let the user dismiss it manually (per spec US2 scenario 4).

**Checkpoint**: `npm run build` passes. Select text on doc page → tooltip appears → click → chat opens with chip → send message → request payload includes correct mode and selected_text.

---

## Phase 5: User Story 3 — Responsive Behavior (P2)

**Goal**: Desktop: floating panel bottom-right. Mobile (<768px): full-screen drawer. Smooth transitions between breakpoints.

**Independent Test**: Chrome DevTools → toggle device toolbar → 375px mobile: full-screen drawer. 1024px desktop: floating panel. Resize dynamically: layout transitions.

**Depends on**: Phase 3 (ChatPanel CSS)

### Implementation for User Story 3

- [x] T018 [US3] Add responsive CSS to `website/src/components/ChatWidget/ChatWidget.css`. Add `@media (max-width: 767px)` rule that applies `.chat-panel-mobile` styles to `.chat-panel`: `position: fixed; inset: 0; width: 100%; height: 100%; max-height: 100%; border-radius: 0;`. Add slide-up animation: `@keyframes slideUp { from { transform: translateY(100%); } to { transform: translateY(0); } }` applied to `.chat-panel-mobile`. On desktop (>=768px), the `.chat-panel` keeps the default floating styles. Reposition `.chat-button` on mobile: `bottom: 16px; right: 16px;` for thumb-friendly placement.
- [x] T019 [US3] Add viewport detection in `website/src/components/ChatWidget/ChatPanel.tsx`. Use a `useEffect` with `window.matchMedia('(max-width: 767px)')` to detect mobile. Apply `.chat-panel-mobile` class conditionally. On mobile: add an overlay backdrop (`position: fixed; inset: 0; bg-black/50`) behind the panel that dismisses the panel on click. Ensure close button is always visible at top-right on mobile (no need to scroll to find it).
- [x] T020 [US3] Move chat button position when panel is open on mobile: in `website/src/components/ChatWidget/ChatButton.tsx`, hide the button entirely when `isOpen && isMobile` (the panel has its own close button). The button reappears when the panel is closed.

**Checkpoint**: Resize viewport between 320px–2560px. Panel adapts. No overlaps. Close button always reachable.

---

## Phase 6: User Story 4 — Session Continuity (P2)

**Goal**: Conversation survives panel close/reopen and SPA navigation. `session_id` persisted in `sessionStorage`.

**Independent Test**: Send 3 messages → close panel → navigate to different chapter → reopen panel → all 3 messages visible. Check sessionStorage has `robotutor_session_id`.

**Depends on**: Phase 3 (ChatProvider)

### Implementation for User Story 4

- [x] T021 [US4] Verify session persistence in `website/src/components/ChatWidget/ChatProvider.tsx`. Confirm: (a) `sessionId` is initialized from `sessionStorage.getItem(SESSION_STORAGE_KEY)` on component mount. (b) On `done` SSE event, `sessionStorage.setItem(SESSION_STORAGE_KEY, sessionId)` is called. (c) `messages[]` state lives in the ChatProvider context which is mounted in Root.js (never unmounts on SPA navigation). (d) `isOpen` state persists across navigations (Root context). If any of these are missing from T006 implementation, add them now.
- [x] T022 [US4] Handle stale session recovery in `website/src/components/ChatWidget/ChatProvider.tsx`. If the backend returns a `session_not_found` SSE event or HTTP 404: (a) clear `sessionStorage.removeItem(SESSION_STORAGE_KEY)`, (b) set `sessionId = null`, (c) retry the message as a new session (omit `session_id` from request). Log a warning to console.

**Checkpoint**: Send messages → close panel → navigate 10 pages → reopen → messages present. Hard reload → session_id in sessionStorage → next message resumes session.

---

## Phase 7: User Story 5 — Loading, Error, and Empty States (P2)

**Goal**: Clear visual feedback for all UI states: empty (welcome), loading (typing dots), streaming (progressive), error (with retry), rate-limited (countdown), offline (disabled).

**Independent Test**: (a) Open chat fresh → welcome screen with suggestions. (b) Send message → typing indicator → streaming text → idle. (c) Kill backend → send message → error with retry button. (d) Toggle airplane mode → offline indicator.

**Depends on**: Phase 3 (ChatPanel, ChatProvider)

### Implementation for User Story 5

- [x] T023 [P] [US5] Add typing indicator to `website/src/components/ChatWidget/ChatPanel.tsx`. When `uiState === 'loading'`, render a typing indicator (three animated dots) in the message list area below the last message. Use the `.typing-indicator` class from ChatWidget.css. CSS animation: `@keyframes bounce { 0%, 80%, 100% { transform: translateY(0); } 40% { transform: translateY(-6px); } }` on 3 spans with staggered delays.
- [x] T024 [P] [US5] Add error state to `website/src/components/ChatWidget/ChatPanel.tsx`. When `uiState === 'error'`, render an error message bubble (red accent border, icon) with the error text and a "Retry" button that calls `retryLastMessage()`. Style: light red background, dark red text, rounded, with a refresh icon on the retry button.
- [x] T025 [P] [US5] Add rate-limit state to `website/src/components/ChatWidget/ChatPanel.tsx`. When `uiState === 'rate_limited'`, render a warning message showing "Too many requests. Try again in Xs." with a countdown timer. Store `retryAfter` seconds in ChatProvider state (set from the `rate_limited` SSE event). Use `setInterval` to tick down every second, and transition to `uiState='idle'` when countdown reaches 0.
- [x] T026 [US5] Add offline detection to `website/src/components/ChatWidget/ChatProvider.tsx`. On mount, listen to `window.addEventListener('online', ...)` and `window.addEventListener('offline', ...)`. When offline: set `uiState='offline'`. When back online: restore to previous state (`'idle'` or `'empty'`). In `website/src/components/ChatWidget/ChatPanel.tsx`: when `uiState === 'offline'`, show a banner "You're offline" and disable the input/send button.
- [x] T027 [US5] Add auto-scroll refinement to `website/src/components/ChatWidget/ChatPanel.tsx`. Only auto-scroll if the user is already at or near the bottom of the message list (within 50px). If the user has scrolled up to read history, do NOT force-scroll them down. Implement: compare `scrollTop + clientHeight` to `scrollHeight` before scrolling.

**Checkpoint**: All 7 UI states (empty, idle, loading, streaming, error, rate_limited, offline) display correctly with visual differentiation.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Accessibility, keyboard navigation, theme polish, and final build/deploy validation.

- [x] T028 Add ARIA attributes and keyboard handlers across all widget components. In `website/src/components/ChatWidget/ChatPanel.tsx`: add `role="dialog"`, `aria-modal="true"`, `aria-label="RoboTutor Chat"`. Implement focus trap: on open, focus the textarea; Tab cycles through close button → message list → textarea → send button → close button. On Escape key, close the panel. In `website/src/components/ChatWidget/ChatButton.tsx`: ensure `tabIndex={0}`, `onKeyDown` handles Enter to toggle panel. In ChatPanel textarea: Enter sends message, Shift+Enter inserts newline.
- [x] T029 Add dark mode polish to `website/src/components/ChatWidget/ChatWidget.css`. Add `[data-theme='dark']` selectors for: `.chat-button` (adjust gradient for dark context), `.message-bubble-user` (dark blue), `.message-bubble-assistant` (dark gray-800), `.selection-tooltip` (dark blue-700), `.selected-text-chip` (dark gray-800 with blue-400 border), error/rate-limit/offline banners (dark background variants). Verify all text is readable against dark backgrounds using Docusaurus CSS vars.
- [x] T030 Run Docusaurus build validation: execute `cd website && npm run build`. Verify: (a) zero build errors, (b) no `ReferenceError: window is not defined` or `document is not defined` SSR errors, (c) no new broken link warnings. If any browser-only code leaks into SSR, wrap it in `useEffect` or move it inside the `BrowserOnly` boundary in Root.js.
- [x] T031 Run local serve test: execute `cd website && npm run serve`. Open `http://localhost:3000/physical-ai-robotics-textbook/docs/` in browser. Walk through the full verification checklist (see below). Check at least 3 different doc pages to confirm widget appears on all of them.
- [x] T032 Create post-deploy verification checklist as a comment block at the bottom of `website/src/components/ChatWidget/index.tsx`. Document the 5 GitHub Pages checks from plan.md: (1) chat button visible on deployed docs URL, (2) panel opens on click, (3) backend CORS allows GitHub Pages origin, (4) SSE streaming works over HTTPS, (5) no mixed-content warnings.

---

## Dependencies

```
Phase 1 (Setup)
  ↓
Phase 2 (Foundational: T003–T007)
  ↓
Phase 3 (US1: Chat Widget MVP) ← T008–T013
  ↓                       ↓
Phase 4 (US2: Selection)  Phase 5 (US3: Responsive)  Phase 6 (US4: Sessions)  Phase 7 (US5: States)
  [independent]            [independent]              [independent]            [independent]
  ↓                       ↓                          ↓                        ↓
Phase 8 (Polish & Validation) ← waits for all stories
```

**Parallel opportunities within phases**:
- Phase 2: T003, T004, T005 are all in different files → run in parallel
- Phase 3: T008, T009, T010 are all in different component files → run in parallel
- Phase 4: T014, T015 are in different files → run in parallel
- Phase 7: T023, T024, T025 are additive to ChatPanel → can be parallelized if careful

**Story independence**: US3, US4, US5 can all be implemented in any order after US1. US2 depends on US1 (needs ChatPanel to exist).

## Implementation Strategy

1. **MVP** (Phase 1–3): Chat button + panel + SSE streaming on all pages. Delivers US1 — the core interaction. Ship and validate.
2. **Selection Mode** (Phase 4): Text selection → contextual chat. Delivers US2 — the differentiating feature.
3. **Polish** (Phase 5–8): Responsive layout, session continuity, UI states, accessibility. All P2 stories + cross-cutting polish. Can be done in any order.

## Verification Checklist

### Normal Chat Flow
- [ ] V01 Chat button visible on `http://localhost:3000/physical-ai-robotics-textbook/docs/module-1/`
- [ ] V02 Click button → panel opens with welcome screen and suggestion chips
- [ ] V03 Click a suggestion chip → message sent, typing indicator shown
- [ ] V04 Streamed response appears token-by-token (if backend running)
- [ ] V05 Close panel → button reappears → reopen → conversation preserved
- [ ] V06 Navigate to different doc page → reopen panel → conversation still present
- [ ] V07 Check `sessionStorage` → `robotutor_session_id` key has UUID value

### Selection-Only Chat Flow
- [ ] V08 On a doc page, highlight >= 10 characters of text
- [ ] V09 "Ask about this" tooltip appears near the selection
- [ ] V10 Click tooltip → panel opens with quoted selection chip visible
- [ ] V11 Type question and submit → Network tab shows `mode: "selected_text_only"` and `selected_text` in request
- [ ] V12 Dismiss chip (click x) → mode indicator disappears → next message sends `mode: "normal"`
- [ ] V13 Select < 10 chars → tooltip does NOT appear

### Responsive & Accessibility
- [ ] V14 Viewport 375px → panel is full-screen drawer with visible close button
- [ ] V15 Viewport 1920px → panel is floating bottom-right (400x600)
- [ ] V16 Tab → focuses chat button → Enter → panel opens → Tab through controls → Escape closes
- [ ] V17 Toggle dark mode → widget colors update, no flash of wrong theme

### Error States
- [ ] V18 Backend offline → send message → error message with Retry button displayed
- [ ] V19 Click Retry → message re-sent
- [ ] V20 `npm run build` → zero errors, zero SSR warnings
