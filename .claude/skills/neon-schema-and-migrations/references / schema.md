# Neon Postgres Schema Reference

Complete schema documentation for the RAG chatbot application.

## Core Entities

### users
Primary user identity table.

```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    name VARCHAR(255),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

**Indexes:**
- `idx_users_email` on `email` (for login lookups)

**Usage:**
- Authentication and user identification
- Linked to chat_sessions and user_profiles

---

### user_profiles
Extended user information from onboarding.

```sql
CREATE TABLE user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    software_background TEXT,
    hardware_background TEXT,
    learning_goals TEXT,
    experience_level VARCHAR(50),
    preferred_language VARCHAR(10) DEFAULT 'en',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(user_id)
);
```

**Indexes:**
- `idx_user_profiles_user_id` on `user_id`

**Usage:**
- Store onboarding questionnaire responses
- Personalize content based on background
- Support language preferences

**Example onboarding fields:**
- `software_background`: "Python, JavaScript, basic SQL"
- `hardware_background`: "Raspberry Pi, Arduino"
- `experience_level`: "beginner", "intermediate", "advanced"
- `preferred_language`: "en", "ur" (for Urdu translation)

---

### chat_sessions
Conversation threads/sessions.

```sql
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    title VARCHAR(500),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB DEFAULT '{}'
);
```

**Indexes:**
- `idx_chat_sessions_user_id` on `user_id` (for user's session list)
- `idx_chat_sessions_created_at` on `created_at DESC` (for recent sessions)

**Usage:**
- Group related messages into conversations
- Support multiple concurrent conversations
- Allow anonymous sessions (user_id can be NULL)

**Metadata JSONB examples:**
```json
{
  "chapter": "module-1-ros2",
  "personalized": true,
  "language": "ur"
}
```

---

### messages
Individual chat messages within sessions.

```sql
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role VARCHAR(50) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    selected_text TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB DEFAULT '{}'
);
```

**Indexes:**
- `idx_messages_session_id` on `session_id` (for loading conversation)
- `idx_messages_created_at` on `created_at` (for chronological order)

**Usage:**
- Store conversation messages
- Support text selection queries (`selected_text` field)
- Track message role (user/assistant/system)

**Special fields:**
- `selected_text`: User-highlighted text for context-specific questions
- `metadata`: Additional message properties

**Metadata JSONB examples:**
```json
{
  "tokens": 1250,
  "model": "gpt-4",
  "processing_time_ms": 3420,
  "selected_text_range": {"start": 100, "end": 500}
}
```

---

### message_citations
RAG source citations for assistant messages.

```sql
CREATE TABLE message_citations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES messages(id) ON DELETE CASCADE,
    document_id VARCHAR(255),
    chunk_id VARCHAR(255),
    score FLOAT,
    citation_index INTEGER,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

**Indexes:**
- `idx_message_citations_message_id` on `message_id`
- `idx_message_citations_document_id` on `document_id`

**Usage:**
- Track which document chunks were used for RAG
- Store relevance scores
- Support citation numbering in responses

**Example data:**
```sql
INSERT INTO message_citations (message_id, document_id, chunk_id, score, citation_index)
VALUES 
    ('...', 'module-1-ros2', 'chunk_023', 0.89, 1),
    ('...', 'module-2-gazebo', 'chunk_045', 0.82, 2);
```

---

## Schema Migrations Tracking

### schema_migrations
Track applied migrations for version control.

```sql
CREATE TABLE schema_migrations (
    id SERIAL PRIMARY KEY,
    version VARCHAR(255) UNIQUE NOT NULL,
    applied_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    description TEXT
);
```

**Usage:**
- Prevent duplicate migrations
- Track deployment history
- Support rollback planning

---

## Common Query Patterns

### Get user's recent sessions
```sql
SELECT s.*, COUNT(m.id) as message_count
FROM chat_sessions s
LEFT JOIN messages m ON s.id = m.session_id
WHERE s.user_id = $1
GROUP BY s.id
ORDER BY s.updated_at DESC
LIMIT 20;
```

### Get full conversation
```sql
SELECT m.*, mc.document_id, mc.score
FROM messages m
LEFT JOIN message_citations mc ON m.id = mc.message_id
WHERE m.session_id = $1
ORDER BY m.created_at ASC;
```

### Get user profile with preferences
```sql
SELECT u.*, up.software_background, up.hardware_background, up.preferred_language
FROM users u
LEFT JOIN user_profiles up ON u.id = up.user_id
WHERE u.email = $1;
```

### Search messages by selected text
```sql
SELECT m.*, s.title
FROM messages m
JOIN chat_sessions s ON m.session_id = s.id
WHERE m.user_id = $1
  AND m.selected_text IS NOT NULL
  AND m.selected_text ILIKE $2
ORDER BY m.created_at DESC;
```

---

## Extension Support

### For personalization features
Add to `chat_sessions.metadata`:
```json
{
  "personalization_settings": {
    "difficulty_level": "beginner",
    "show_hardware_examples": true,
    "code_language_preference": "python"
  }
}
```

### For translation features
Add to `messages.metadata`:
```json
{
  "translation": {
    "original_language": "en",
    "translated_to": "ur",
    "translation_timestamp": "2025-01-29T10:30:00Z"
  }
}
```

### For RAG enhancement
Store vector embeddings metadata in `messages.metadata`:
```json
{
  "rag_context": {
    "query_embedding_model": "text-embedding-3-small",
    "chunks_retrieved": 5,
    "avg_similarity": 0.85
  }
}
```