# Quickstart: Embed ChatKit UI Widget

**Feature**: 011-embed-chatkit-ui
**Date**: 2026-02-15

## Prerequisites

- Node.js >= 18
- The Docusaurus site builds successfully (`cd website && npm run build`)
- Access to the FastAPI backend URL (local or deployed)

## Local Development

### 1. Start the Docusaurus dev server

```bash
cd website
npm start
```

The site runs at `http://localhost:3000/physical-ai-robotics-textbook/`.

### 2. (Optional) Start the FastAPI backend locally

```bash
# From repo root
uvicorn backend.main:app --reload --port 8000
```

Backend runs at `http://localhost:8000`. Ensure `CORS_ORIGINS` includes `http://localhost:3000`.

### 3. Configure the backend URL

The chat widget reads the backend URL from an environment variable or config constant:

```bash
# Option A: Environment variable (for dev)
REACT_APP_CHAT_API_URL=http://localhost:8000

# Option B: Docusaurus customFields in docusaurus.config.js
customFields: {
  chatApiUrl: process.env.REACT_APP_CHAT_API_URL || 'https://your-backend.example.com',
}
```

### 4. Verify the widget

1. Open any documentation page (e.g., `http://localhost:3000/physical-ai-robotics-textbook/docs/module-1/`)
2. Confirm the floating chat button appears in the bottom-right corner
3. Click the button → chat panel opens
4. Type a message and press Enter
5. If backend is running: verify streamed response appears
6. If backend is NOT running: verify error state with retry button

### 5. Test text selection

1. On any doc page, highlight a passage of text (>= 10 characters)
2. Confirm "Ask about this" tooltip appears near the selection
3. Click the tooltip → chat panel opens with quoted selection visible
4. Type a question and submit
5. Verify the request includes `mode: "selected_text_only"` (check browser Network tab)

### 6. Test responsive behavior

1. Open browser DevTools → toggle device toolbar
2. Set viewport to 375px width (mobile)
3. Click chat button → verify full-screen drawer layout
4. Set viewport to 1024px width (desktop)
5. Click chat button → verify floating panel layout

## Build Verification

```bash
cd website
npm run build
npm run serve
```

Verify:
- No build errors or warnings related to chat widget
- Widget renders on served build at `http://localhost:3000/physical-ai-robotics-textbook/`
- No `window`/`document` SSR errors during build (all browser APIs guarded with `BrowserOnly` or `useEffect`)

## GitHub Pages Deployment

- Ensure `CORS_ORIGINS` on the backend includes `https://abduIIahKhaIid.github.io`
- The widget uses relative paths and respects `baseUrl: '/physical-ai-robotics-textbook/'`
- Backend URL must be an absolute HTTPS URL (not relative)
- Verify: `https://abduIIahKhaIid.github.io/physical-ai-robotics-textbook/docs/` shows the chat button
