# Data Model: FastAPI Chat Backend

**Date**: 2026-02-14 | **Branch**: `010-fastapi-chat-backend`

## Entities

### 1. users

Represents a chatbot user, either anonymous (IP-based) or identified (token-based).

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, default gen_random_uuid() | Unique user identifier |
| token | VARCHAR(255) | UNIQUE, NOT NULL | Opaque bearer token for authentication |
| tier | VARCHAR(20) | NOT NULL, default 'anonymous' | User tier: 'anonymous' or 'identified' |
| ip_address | INET | nullable | IP address (for anonymous users) |
| created_at | TIMESTAMPTZ | NOT NULL, default now() | Account creation time |
| last_seen_at | TIMESTAMPTZ | NOT NULL, default now() | Last activity time |

**Indexes**: `idx_users_token` on `token`

### 2. sessions

A conversation thread between a user and the chatbot.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, default gen_random_uuid() | Unique session identifier |
| user_id | UUID | FK → users.id, NOT NULL | Owning user |
| title | VARCHAR(255) | NOT NULL, default '' | Auto-generated from first message |
| status | VARCHAR(20) | NOT NULL, default 'active' | Session status: 'active' or 'archived' |
| created_at | TIMESTAMPTZ | NOT NULL, default now() | Session creation time |
| updated_at | TIMESTAMPTZ | NOT NULL, default now() | Last activity time |

**Indexes**: `idx_sessions_user_id` on `user_id`, `idx_sessions_updated_at` on `updated_at`

### 3. messages

A single message within a session (user question or assistant response).

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, default gen_random_uuid() | Unique message identifier |
| session_id | UUID | FK → sessions.id, NOT NULL | Parent session |
| role | VARCHAR(20) | NOT NULL | Message role: 'user' or 'assistant' |
| content | TEXT | NOT NULL | Message text content |
| retrieval_mode | VARCHAR(30) | nullable | 'normal' or 'selected_text_only' |
| selected_text | TEXT | nullable | User's selected text (if applicable) |
| chunk_count | INTEGER | nullable | Number of chunks retrieved |
| latency_ms | INTEGER | nullable | Response generation time in ms |
| created_at | TIMESTAMPTZ | NOT NULL, default now() | Message timestamp |

**Indexes**: `idx_messages_session_id` on `session_id`, `idx_messages_created_at` on `created_at`

### 4. schema_migrations (internal)

Tracks applied database migrations.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| version | VARCHAR(20) | PK | Migration version identifier |
| applied_at | TIMESTAMPTZ | NOT NULL, default now() | When migration was applied |

## Relationships

```
users 1──* sessions 1──* messages
```

- A user has many sessions
- A session has many messages (ordered by `created_at`)
- Cascade: deleting a user cascades to sessions and messages

## State Transitions

### Session Status
```
active → archived (after 30 days of inactivity, or manual archive)
```

### User Tier
```
anonymous → identified (upgrade when explicitly identified)
```

## Validation Rules

- `role` must be one of: `user`, `assistant`
- `status` must be one of: `active`, `archived`
- `tier` must be one of: `anonymous`, `identified`
- `retrieval_mode` must be one of: `normal`, `selected_text_only`, or null
- `content` must not be empty or whitespace-only
- `selected_text` must not be empty when `retrieval_mode` is `selected_text_only`
