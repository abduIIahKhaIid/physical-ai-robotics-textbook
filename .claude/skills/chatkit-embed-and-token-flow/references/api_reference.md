# ChatKit Integration API Reference

## Overview

This reference covers the complete token and session flow between the Docusaurus UI and FastAPI backend for ChatKit widget integration.

## Architecture

```
┌─────────────────────┐
│  Docusaurus Client  │
│   (ChatKit Widget)  │
└──────────┬──────────┘
           │ 1. Request token (optional)
           │ 2. Send chat message + session_id
           ▼
┌─────────────────────┐
│   FastAPI Backend   │
│  /auth/token        │
│  /chat              │
└──────────┬──────────┘
           │ 3. Query RAG (Qdrant)
           │ 4. Call OpenAI
           ▼
┌─────────────────────┐
│   RAG Pipeline      │
│  Qdrant + OpenAI    │
└─────────────────────┘
```

## Authentication Flows

### Flow 1: No Authentication (Development)

**Client Side:**
- Generates session_id locally
- Stores in localStorage
- Sends with every request

**Backend:**
- No token validation
- Trusts session_id from client
- Stores messages keyed by session_id

### Flow 2: Token-Based (Better Auth)

**Client Side:**
1. User logs in via Better Auth
2. Widget requests token: `GET /auth/token`
3. Receives short-lived JWT (1 hour)
4. Includes token in chat requests: `Authorization: Bearer <token>`

**Backend:**
1. Validates JWT on every request
2. Extracts user_id from token
3. Associates session with user
4. Stores user-specific history

## API Endpoints

### POST /chat

Send a chat message to the bot.

**Request (Normal Mode):**
```json
{
  "message": "What is Physical AI?",
  "session_id": "session_1234567890_abc123",
  "mode": "normal",
  "selected_text": null
}
```

**Request (Selection Mode):**
```json
{
  "message": "Explain this concept",
  "session_id": "session_1234567890_abc123",
  "mode": "selection_only",
  "selected_text": "Physical AI represents the convergence of artificial intelligence with robotic systems that operate in the physical world..."
}
```

**Response:**
```json
{
  "response": "Physical AI refers to AI systems that understand and interact with the physical world...",
  "session_id": "session_1234567890_abc123",
  "message_id": "msg_uuid_here"
}
```

**Status Codes:**
- `200`: Success
- `400`: Invalid request
- `401`: Invalid or expired token (auth mode only)
- `500`: Server error

### GET /auth/token (Better Auth Mode)

Request a short-lived token for chat session.

**Response:**
```json
{
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "expires_in": 3600
}
```

### GET /sessions/{session_id}

Retrieve session history.

**Response:**
```json
{
  "messages": [
    {
      "id": "msg_1",
      "message": "What is ROS 2?",
      "response": "ROS 2 is a robotic middleware...",
      "selected_text": null,
      "timestamp": "2025-11-30T10:30:00Z"
    }
  ],
  "created_at": "2025-11-30T10:00:00Z"
}
```

## Selected Text Handling

### Client-Side Capture

```javascript
document.addEventListener('mouseup', () => {
  const selection = window.getSelection();
  const text = selection.toString().trim();
  
  if (text.length > 0 && text.length < 12000) {
    setSelectedText(text);
  }
});
```

### Constraints

- **Minimum length:** 1 character (after trim)
- **Maximum length:** 12,000 characters
- **Sanitization:** Trim whitespace, normalize line breaks
- **Validation:** Backend must reject if mode is `selection_only` but no text provided

## Session Management

### Session ID Format

```
session_{timestamp}_{random_string}
```

Example: `session_1701341234_abc123def`

### Storage Options

**Development (In-Memory):**
```python
sessions = {
    "session_id": {
        "messages": [...],
        "created_at": datetime,
        "user_id": None
    }
}
```

**Production (Database):**
- Use Neon Serverless Postgres
- Index on session_id, user_id, timestamp
- TTL of 24 hours for expired sessions