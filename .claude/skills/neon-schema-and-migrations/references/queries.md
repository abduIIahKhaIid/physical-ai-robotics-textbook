# Common SQL Queries

Frequently used queries for the RAG chatbot schema.

## User Queries

### Create new user with profile
```sql
WITH new_user AS (
  INSERT INTO users (email, name)
  VALUES ($1, $2)
  RETURNING id
)
INSERT INTO user_profiles (user_id, software_background, hardware_background, experience_level, preferred_language)
SELECT id, $3, $4, $5, $6
FROM new_user
RETURNING *;
```

### Get user with profile
```sql
SELECT 
  u.id, u.email, u.name,
  up.software_background, up.hardware_background, 
  up.learning_goals, up.experience_level, up.preferred_language
FROM users u
LEFT JOIN user_profiles up ON u.id = up.user_id
WHERE u.email = $1;
```

### Update user preferences
```sql
UPDATE user_profiles
SET 
  preferred_language = $2,
  updated_at = CURRENT_TIMESTAMP
WHERE user_id = $1;
```

---

## Session Queries

### Create new session
```sql
INSERT INTO chat_sessions (user_id, title, metadata)
VALUES ($1, $2, $3::jsonb)
RETURNING *;
```

### Get user's recent sessions
```sql
SELECT 
  s.id, s.title, s.created_at, s.updated_at,
  COUNT(m.id) as message_count,
  MAX(m.created_at) as last_message_at
FROM chat_sessions s
LEFT JOIN messages m ON s.id = m.session_id
WHERE s.user_id = $1
GROUP BY s.id
ORDER BY s.updated_at DESC
LIMIT 20;
```

### Update session title
```sql
UPDATE chat_sessions
SET 
  title = $2,
  updated_at = CURRENT_TIMESTAMP
WHERE id = $1;
```

---

## Message Queries

### Insert message with citations
```sql
-- Insert message
WITH new_message AS (
  INSERT INTO messages (session_id, role, content, selected_text, metadata)
  VALUES ($1, $2, $3, $4, $5::jsonb)
  RETURNING id
)
-- Insert citations
INSERT INTO message_citations (message_id, document_id, chunk_id, score, citation_index)
SELECT 
  new_message.id,
  unnest($6::varchar[]),  -- document_ids array
  unnest($7::varchar[]),  -- chunk_ids array
  unnest($8::float[]),    -- scores array
  unnest($9::int[])       -- citation_indexes array
FROM new_message;
```

### Get conversation with citations
```sql
SELECT 
  m.id, m.role, m.content, m.selected_text, m.created_at, m.metadata,
  json_agg(
    json_build_object(
      'document_id', mc.document_id,
      'chunk_id', mc.chunk_id,
      'score', mc.score,
      'citation_index', mc.citation_index
    ) ORDER BY mc.citation_index
  ) FILTER (WHERE mc.id IS NOT NULL) as citations
FROM messages m
LEFT JOIN message_citations mc ON m.id = mc.message_id
WHERE m.session_id = $1
GROUP BY m.id
ORDER BY m.created_at ASC;
```

### Search messages by content
```sql
SELECT 
  m.*, s.title, s.user_id,
  ts_rank(to_tsvector('english', m.content), query) as rank
FROM messages m
JOIN chat_sessions s ON m.session_id = s.id,
     plainto_tsquery('english', $1) query
WHERE to_tsvector('english', m.content) @@ query
  AND s.user_id = $2
ORDER BY rank DESC
LIMIT 20;
```

### Get messages with selected text
```sql
SELECT m.*, s.title
FROM messages m
JOIN chat_sessions s ON m.session_id = s.id
WHERE m.selected_text IS NOT NULL
  AND s.user_id = $1
ORDER BY m.created_at DESC
LIMIT 50;
```

---

## Citation Queries

### Get most cited documents
```sql
SELECT 
  document_id,
  COUNT(*) as citation_count,
  AVG(score) as avg_score
FROM message_citations
WHERE document_id IS NOT NULL
  AND created_at > NOW() - INTERVAL '30 days'
GROUP BY document_id
ORDER BY citation_count DESC
LIMIT 10;
```

### Get citations for a message
```sql
SELECT 
  mc.document_id, mc.chunk_id, mc.score, mc.citation_index
FROM message_citations mc
WHERE mc.message_id = $1
ORDER BY mc.citation_index;
```

---

## Analytics Queries

### User activity summary
```sql
SELECT 
  u.id, u.email, u.name,
  COUNT(DISTINCT s.id) as session_count,
  COUNT(m.id) as message_count,
  MAX(m.created_at) as last_active
FROM users u
LEFT JOIN chat_sessions s ON u.id = s.user_id
LEFT JOIN messages m ON s.id = m.session_id
GROUP BY u.id
ORDER BY last_active DESC NULLS LAST;
```

### Daily message volume
```sql
SELECT 
  DATE(created_at) as date,
  COUNT(*) as message_count,
  COUNT(DISTINCT session_id) as session_count
FROM messages
WHERE created_at > NOW() - INTERVAL '30 days'
GROUP BY DATE(created_at)
ORDER BY date DESC;
```

### Session length distribution
```sql
SELECT 
  COUNT(*) as session_count,
  CASE 
    WHEN message_count = 1 THEN '1 message'
    WHEN message_count BETWEEN 2 AND 5 THEN '2-5 messages'
    WHEN message_count BETWEEN 6 AND 10 THEN '6-10 messages'
    WHEN message_count BETWEEN 11 AND 20 THEN '11-20 messages'
    ELSE '20+ messages'
  END as length_bucket
FROM (
  SELECT session_id, COUNT(*) as message_count
  FROM messages
  GROUP BY session_id
) session_lengths
GROUP BY length_bucket
ORDER BY MIN(message_count);
```

---

## Maintenance Queries

### Find orphaned sessions
```sql
SELECT s.*
FROM chat_sessions s
LEFT JOIN messages m ON s.id = m.session_id
WHERE m.id IS NULL
  AND s.created_at < NOW() - INTERVAL '7 days';
```

### Delete old anonymous sessions
```sql
DELETE FROM chat_sessions
WHERE user_id IS NULL
  AND created_at < NOW() - INTERVAL '30 days'
  AND id NOT IN (
    SELECT DISTINCT session_id 
    FROM messages 
    WHERE created_at > NOW() - INTERVAL '7 days'
  );
```

### Cleanup old citations
```sql
DELETE FROM message_citations
WHERE message_id IN (
  SELECT m.id
  FROM messages m
  JOIN chat_sessions s ON m.session_id = s.id
  WHERE s.created_at < NOW() - INTERVAL '90 days'
);
```

---

## Advanced JSONB Queries

### Search metadata
```sql
SELECT *
FROM chat_sessions
WHERE metadata @> '{"personalized": true}'::jsonb
  AND user_id = $1;
```

### Update JSONB field
```sql
UPDATE messages
SET metadata = metadata || '{"translated": true, "language": "ur"}'::jsonb
WHERE id = $1;
```

### Extract JSONB value
```sql
SELECT 
  id, content,
  metadata->>'model' as model_used,
  (metadata->>'tokens')::int as token_count
FROM messages
WHERE metadata ? 'model'
  AND session_id = $1;
```

### Filter by JSONB array
```sql
SELECT *
FROM user_profiles
WHERE (preferences->'interests')::jsonb ? 'robotics'
  AND experience_level = 'intermediate';
```

---

## Performance Tips

### Use prepared statements
```python
# Instead of string interpolation
cursor.execute(
    "SELECT * FROM users WHERE email = %s",
    (email,)
)
```

### Batch inserts
```python
# Insert multiple messages at once
psycopg2.extras.execute_batch(
    cursor,
    "INSERT INTO messages (session_id, role, content) VALUES (%s, %s, %s)",
    [(session_id, 'user', msg) for msg in messages]
)
```

### Use indexes for JSONB
```sql
-- For exact match queries
CREATE INDEX idx_metadata_personalized 
ON chat_sessions ((metadata->>'personalized'));

-- For containment queries
CREATE INDEX idx_metadata_gin 
ON chat_sessions USING GIN (metadata);
```