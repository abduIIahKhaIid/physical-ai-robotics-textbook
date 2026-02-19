---
name: neon-schema-and-migrations
description: Define Neon Postgres schema and migration workflow for chatbot sessions, messages, users, and optional onboarding profile fields. Use when setting up database persistence for RAG chatbots, adding authentication/onboarding tables, or creating new migrations for message metadata (selected text, citations, document IDs). Supports init scripts, Alembic, Prisma, and raw SQL workflows.
---

# Neon Schema and Migrations

Production-ready Postgres schema and migration toolkit for RAG chatbot applications on Neon serverless Postgres.

## Non-Negotiable Rules

1. **Must use asyncpg for queries** - All database operations must use asyncpg for async Postgres access; do not use synchronous drivers in production code
2. **Must support rollback** - Every migration must include rollback SQL (DOWN section) to enable safe reversion
3. **No raw credentials** - Never hardcode database passwords or connection strings; use environment variables (`$DATABASE_URL`) and `.env` files
4. **Test on branch first** - All migrations must be tested on a Neon branch or dev database before applying to production
5. **Use transactions for multi-step changes** - Wrap multi-statement migrations in transactions to ensure atomicity

## Core Schema

The schema supports:
- **Users & Profiles**: Authentication + onboarding questionnaire data (software/hardware background, language preferences)
- **Chat Sessions**: Conversation threads with JSONB metadata
- **Messages**: User/assistant messages with optional selected text context
- **Citations**: RAG source tracking with relevance scores

See `references/schema.md` for complete table definitions.

## Quick Start

### Option 1: Initialize with Python (Recommended)

```bash
export DATABASE_URL="postgresql://user:pass@ep-xxx.neon.tech/db?sslmode=require"
python scripts/init_neon_db.py --scope all
```

**Scopes:**
- `all`: Complete schema (users, sessions, messages, citations)
- `chat`: Chat-only (sessions, messages, citations)
- `auth`: Auth-only (users, profiles)

### Option 2: Use Prisma

Copy the schema:
```bash
cp assets/schema.prisma prisma/schema.prisma
```

Generate client and migrate:
```bash
npx prisma generate
npx prisma db push
```

### Option 3: Use Alembic (SQLAlchemy)

Setup Alembic:
```bash
python scripts/setup_alembic.py
```

Create models, then:
```bash
alembic revision --autogenerate -m "initial schema"
alembic upgrade head
```

## Adding Features

### Create a Migration

```bash
python scripts/create_migration.py "add user timezone preference"
```

Edit the generated SQL file in `migrations/`, then apply:
```bash
psql $DATABASE_URL -f migrations/20250129103000_add_user_timezone_preference.sql
```

### Common Patterns

**Add nullable column (safe):**
```sql
ALTER TABLE messages ADD COLUMN metadata JSONB DEFAULT '{}';
```

**Add indexed column:**
```sql
ALTER TABLE messages ADD COLUMN user_id UUID;
CREATE INDEX idx_messages_user_id ON messages(user_id);
```

**Add foreign key:**
```sql
ALTER TABLE messages ADD CONSTRAINT fk_messages_user
  FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE;
```

See `references/workflow.md` for comprehensive migration patterns.

## Key Features

### Selected Text Support
Store user-highlighted text for context-specific questions:
```sql
INSERT INTO messages (session_id, role, content, selected_text)
VALUES ($1, 'user', 'What does this mean?', 'ROS 2 uses DDS for communication');
```

### RAG Citations
Track document chunks used in responses:
```sql
INSERT INTO message_citations (message_id, document_id, chunk_id, score, citation_index)
VALUES
  ($1, 'module-1-ros2', 'chunk_023', 0.89, 1),
  ($1, 'module-2-gazebo', 'chunk_045', 0.82, 2);
```

### User Personalization
Store onboarding data for content customization:
```sql
INSERT INTO user_profiles
  (user_id, software_background, hardware_background, experience_level, preferred_language)
VALUES
  ($1, 'Python, JavaScript', 'Raspberry Pi', 'intermediate', 'en');
```

### JSONB Metadata
Flexible metadata for sessions and messages:
```sql
-- Session metadata
UPDATE chat_sessions
SET metadata = metadata || '{"personalized": true, "chapter": "module-1"}'::jsonb
WHERE id = $1;

-- Message metadata
UPDATE messages
SET metadata = metadata || '{"translated_to": "ur", "tokens": 1250}'::jsonb
WHERE id = $1;
```

## Core Implementation Workflow

**Choose based on your stack:**

1. **Python + psycopg2**: Use `init_neon_db.py` + `create_migration.py`
2. **TypeScript + Prisma**: Use `assets/schema.prisma`
3. **Python + SQLAlchemy**: Use `setup_alembic.py` → Alembic workflow
4. **Direct SQL**: Use `create_migration.py` → edit SQL → apply manually

See `references/workflow.md` for detailed workflows.

## Resources

### Scripts (Deterministic Execution)
- `scripts/init_neon_db.py` - Initialize database with full schema
- `scripts/create_migration.py` - Generate timestamped migration files
- `scripts/setup_alembic.py` - Configure Alembic for SQLAlchemy projects

### References (Load as Needed)
- `references/schema.md` - Complete table definitions and relationships
- `references/workflow.md` - Migration workflows and best practices
- `references/queries.md` - Common SQL query patterns

### Assets (Copy to Project)
- `assets/schema.prisma` - Prisma schema definition

## Neon-Specific Notes

**Connection String:**
Ensure SSL is required:
```
postgresql://user:pass@ep-xxx.neon.tech/db?sslmode=require
```

**Branching for Testing:**
```bash
neonctl branches create --name test-migration
neonctl connection-string test-migration
# Test migration on branch
# If good → apply to main
# If bad → delete branch
```

**Auto-suspend:**
First query after idle may be slower (~1-2s) due to database wake-up.

## Migration Safety

Always include rollback SQL:
```sql
-- UP
ALTER TABLE users ADD COLUMN phone VARCHAR(20);

-- DOWN (in comments)
-- ALTER TABLE users DROP COLUMN phone;
-- DELETE FROM schema_migrations WHERE version = '20250129103000';
```

For production migrations:
1. Test on dev/branch first
2. Create database backup
3. Use transactions for multi-step changes
4. Create indexes `CONCURRENTLY` to avoid locks

## Common Queries

Get conversation with citations:
```sql
SELECT
  m.id, m.role, m.content, m.created_at,
  json_agg(mc.document_id) FILTER (WHERE mc.id IS NOT NULL) as cited_docs
FROM messages m
LEFT JOIN message_citations mc ON m.id = mc.message_id
WHERE m.session_id = $1
GROUP BY m.id
ORDER BY m.created_at ASC;
```

See `references/queries.md` for 30+ production-ready queries.

## Acceptance Checklist

- [ ] Schema initialized successfully with chosen scope (all/chat/auth)
- [ ] All migrations include rollback SQL (DOWN section)
- [ ] asyncpg used for all async database operations
- [ ] No raw credentials in code or config files; `$DATABASE_URL` used via environment variables
- [ ] Migration tested on Neon branch or dev database before production
- [ ] Multi-step migrations wrapped in transactions
- [ ] SSL required in connection string (`sslmode=require`)
- [ ] Indexes created for frequently queried columns (session_id, user_id)
