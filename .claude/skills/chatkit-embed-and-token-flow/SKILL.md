---
name: chatkit-embed-and-token-flow
description: Implement ChatKit widget embed in Docusaurus sites and define secure token/session flow between UI and FastAPI backend. Use when building RAG chatbots for textbooks, implementing chat interfaces in documentation sites, adding selected text capture functionality, setting up token-based authentication for chat widgets, or deploying chat features to GitHub Pages. Handles both normal (full-book query) and selection-only (query selected text) modes.
---

# ChatKit Embed and Token Flow

## Overview

Embed a production-ready ChatKit widget in Docusaurus with secure backend communication. Supports normal chat mode (queries entire book/content) and selection-only mode (queries only user-selected text). Handles token-based authentication, session management, and GitHub Pages deployment.

## Non-Negotiable Rules

1. **Never expose server secrets in client code** - All privileged operations happen backend-side
2. **Explicitly capture selected text** - Never infer or guess what the user selected
3. **Validate all inputs** - Both client and server must validate message content and selected text
4. **Respect baseUrl** - All URLs must account for GitHub Pages baseUrl configuration
5. **Handle CORS properly** - Backend must explicitly allow frontend origin

## Quick Start

Use the generator scripts for fastest implementation:

```bash
# Generate widget component
python scripts/generate_chatkit_widget.py \
  --backend-url https://your-backend.vercel.app \
  --mode normal \
  --auth none \
  --output-dir ./generated

# Generate FastAPI backend
python scripts/generate_fastapi_backend.py \
  --auth none \
  --db memory \
  --output ./backend.py
```

Copy generated files to your Docusaurus project:
- `ChatKitWidget.js` → `src/components/`
- `Layout.js` → `src/theme/Layout/`
- Update `docusaurus.config.js` with config snippet

## Core Implementation Workflow

### Step 1: Choose Integration Scope

**Global (site-wide):**
- User can ask questions from any page
- Widget always visible
- Session persists across pages
- Best for: Comprehensive textbook navigation

**Per-chapter:**
- Widget appears only on specific pages
- Can switch between normal and selection modes per chapter
- Best for: Targeted learning experiences

### Step 2: Configure Backend URL

Create environment-aware config:

```javascript
// src/config/chat.js
export const getChatConfig = () => {
  const isDev = process.env.NODE_ENV === 'development';
  return {
    backendUrl: isDev 
      ? 'http://localhost:8000'
      : 'https://your-backend.vercel.app',
    mode: 'normal',
    authType: 'none'
  };
};
```

### Step 3: Implement Widget Component

Either use the generator script or copy from `assets/ChatKitWidget.js`. Key features to include:

**Session Management:**
```javascript
// Generate unique session ID
const sessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
localStorage.setItem('chatkit_session_id', sessionId);
```

**Selection Capture (for selection-only mode):**
```javascript
document.addEventListener('mouseup', () => {
  const selection = window.getSelection();
  const text = selection.toString().trim();
  
  if (text.length > 0 && text.length < 12000) {
    setSelectedText(text);
  }
});
```

**Message Sending:**
```javascript
const payload = {
  message: userMessage,
  session_id: sessionId,
  mode: 'normal', // or 'selection_only'
  ...(selectedText && { selected_text: selectedText })
};

const response = await fetch(`${backendUrl}/chat`, {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify(payload)
});
```

### Step 4: Implement Backend

Use the generator or `assets/backend_minimal.py` as starting point. Critical endpoints:

**POST /chat:**
```python
@app.post("/chat")
async def chat_endpoint(request: ChatRequest):
    # Validate inputs
    if not request.message.strip():
        raise HTTPException(400, "Message cannot be empty")
    
    if request.mode == "selection_only" and not request.selected_text:
        raise HTTPException(400, "Selected text required")
    
    # TODO: Integrate your RAG pipeline
    # context = get_rag_context(request)
    # response = call_openai(request.message, context)
    
    return ChatResponse(response=response_text, ...)
```

**Session Storage:**
- Development: In-memory dictionary
- Production: Neon Postgres or Redis
- Index on session_id for fast lookup

### Step 5: Configure CORS

Backend must explicitly allow your GitHub Pages URL:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://username.github.io",
        "http://localhost:3000"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)
```

### Step 6: Deploy

**Backend:** Deploy to Vercel, Railway, or similar (see `references/deployment.md`)

**Frontend:** 
```bash
npm run build
npm run deploy  # or use GitHub Actions
```

## Selection-Only Mode Implementation

When implementing "Ask about selection" functionality:

1. **User selects text** on page (mouseup/keyup events)
2. **Validate selection:**
   - Min: 1 character (after trim)
   - Max: 12,000 characters
   - Reject if outside range
3. **Send to backend** with `mode: "selection_only"`
4. **Backend queries** only the selected text, not entire book
5. **Display response** contextually

Example user flow:
```
User highlights: "ROS 2 uses DDS for middleware"
User asks: "What is DDS?"
Bot responds using only the selected text as context
```

## Authentication Options

### Option 1: No Auth (Development)
- Client generates session_id
- Backend trusts all sessions
- Fast to implement
- Not suitable for production with user accounts

### Option 2: Better Auth
- User logs in via Better Auth
- Backend issues JWT tokens
- Token included in chat requests
- Session tied to user account
- For detailed implementation, see `references/api_reference.md`

## GitHub Pages Deployment Considerations

**baseUrl Configuration:**
```javascript
// docusaurus.config.js
module.exports = {
  url: 'https://username.github.io',
  baseUrl: '/repo-name/',  // Critical!
};
```

**Asset Paths:**
All fetch URLs must respect baseUrl. Use `useBaseUrl()` hook or configure paths correctly.

**CORS:** Backend must allow the exact GitHub Pages origin.

## Troubleshooting

**Widget not loading:**
- Check browser console for errors
- Verify baseUrl matches deployment URL
- Ensure script imports are correct

**CORS errors:**
- Update backend `allow_origins` with exact GitHub Pages URL
- Check protocol (http vs https) matches

**Selection not captured:**
- Verify mode is 'selection_only'
- Check event listeners are attached
- Test text length constraints

**401 Unauthorized (auth mode):**
- Verify token is being sent in Authorization header
- Check token hasn't expired
- Ensure backend token validation matches client token format

## Bundled Resources

### Scripts
- `generate_chatkit_widget.py` - Generates widget component with all modes
- `generate_fastapi_backend.py` - Creates backend template with auth options

### References
- `api_reference.md` - Complete API specification and token flow details
- `deployment.md` - Step-by-step deployment guide for GitHub Pages

### Assets
- `ChatKitWidget.js` - Production-ready React component
- `backend_minimal.py` - Minimal FastAPI backend to customize