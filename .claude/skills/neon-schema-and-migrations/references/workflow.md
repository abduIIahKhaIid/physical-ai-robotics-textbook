# Migration Workflow Guide

Best practices for managing Neon Postgres schema migrations.

## Quick Start Decision Tree

**Choose your migration approach:**

1. **Just getting started?** → Use `init_neon_db.py`
2. **Need to add a new table/column?** → Use `create_migration.py`
3. **Working with SQLAlchemy models?** → Use Alembic workflow
4. **Quick prototype/testing?** → Direct SQL with rollback plan

---

## Workflow 1: Initial Setup (From Zero)

**When:** First time setting up the database.

**Steps:**

1. **Set environment variable:**
   ```bash
   export DATABASE_URL="postgresql://user:pass@ep-xyz.neon.tech/dbname?sslmode=require"
   ```

2. **Run initialization script:**
   ```bash
   python scripts/init_neon_db.py --scope all
   ```

3. **Verify schema:**
   ```bash
   psql $DATABASE_URL -c "\dt"
   ```

**What happens:**
- Creates all tables (users, chat_sessions, messages, etc.)
- Sets up indexes
- Initializes `schema_migrations` tracking table

**Scopes:**
- `--scope chat`: Only chat-related tables (sessions, messages, citations)
- `--scope auth`: Only user-related tables (users, profiles)
- `--scope all`: Everything (default)

---

## Workflow 2: Adding New Features (Incremental)

**When:** Adding new columns, tables, or indexes to existing schema.

**Steps:**

1. **Generate migration file:**
   ```bash
   python scripts/create_migration.py "add user preferences column"
   ```

2. **Edit the generated SQL file:**
   ```sql
   -- Migration: add user preferences column
   -- Created: 2025-01-29T10:30:00
   -- Version: 20250129103000

   -- UP Migration
   ALTER TABLE user_profiles 
   ADD COLUMN preferences JSONB DEFAULT '{}';

   CREATE INDEX idx_user_profiles_preferences 
   ON user_profiles USING GIN (preferences);

   INSERT INTO schema_migrations (version, description) 
   VALUES ('20250129103000', 'add user preferences column');

   -- DOWN Migration (Rollback)
   ALTER TABLE user_profiles DROP COLUMN preferences;
   DELETE FROM schema_migrations WHERE version = '20250129103000';
   ```

3. **Test on dev database:**
   ```bash
   psql $DATABASE_URL_DEV -f migrations/20250129103000_add_user_preferences_column.sql
   ```

4. **Apply to production:**
   ```bash
   psql $DATABASE_URL -f migrations/20250129103000_add_user_preferences_column.sql
   ```

**Best Practices:**
- Always include rollback SQL in comments
- Test on dev/staging first
- Use transactions for complex migrations
- Back up before production migrations

---

## Workflow 3: Alembic (SQLAlchemy Integration)

**When:** Using SQLAlchemy ORM models in your application.

**Initial Setup:**

1. **Setup Alembic:**
   ```bash
   python scripts/setup_alembic.py
   ```

2. **Define your models** (example):
   ```python
   # models.py
   from sqlalchemy import Column, String, Text, DateTime
   from sqlalchemy.dialects.postgresql import UUID
   from sqlalchemy.ext.declarative import declarative_base
   import uuid

   Base = declarative_base()

   class User(Base):
       __tablename__ = 'users'
       id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
       email = Column(String(255), unique=True, nullable=False)
       name = Column(String(255))
   ```

3. **Generate initial migration:**
   ```bash
   alembic revision --autogenerate -m "initial schema"
   ```

4. **Apply migrations:**
   ```bash
   alembic upgrade head
   ```

**Adding Changes:**

1. **Update your models:**
   ```python
   class User(Base):
       # ... existing fields ...
       phone = Column(String(20))  # NEW FIELD
   ```

2. **Auto-generate migration:**
   ```bash
   alembic revision --autogenerate -m "add user phone"
   ```

3. **Review generated migration:**
   ```python
   # alembic/versions/abc123_add_user_phone.py
   def upgrade():
       op.add_column('users', sa.Column('phone', sa.String(20)))

   def downgrade():
       op.drop_column('users', 'phone')
   ```

4. **Apply:**
   ```bash
   alembic upgrade head
   ```

**Useful Alembic Commands:**
```bash
alembic current                    # Show current version
alembic history                    # Show all migrations
alembic upgrade +1                 # Apply next migration
alembic downgrade -1               # Rollback one migration
alembic stamp head                 # Mark as up-to-date without running
```

---

## Workflow 4: Direct SQL (Quick Changes)

**When:** Quick fixes, one-off changes, or operations not suitable for migration scripts.

**Safe Pattern:**

1. **Start transaction:**
   ```sql
   BEGIN;
   ```

2. **Make changes:**
   ```sql
   ALTER TABLE messages ADD COLUMN is_deleted BOOLEAN DEFAULT false;
   ```

3. **Verify:**
   ```sql
   SELECT column_name, data_type 
   FROM information_schema.columns 
   WHERE table_name = 'messages';
   ```

4. **Commit or rollback:**
   ```sql
   COMMIT;    -- if looks good
   -- or
   ROLLBACK;  -- if something wrong
   ```

**Always Record:**
```sql
INSERT INTO schema_migrations (version, description, applied_at)
VALUES ('manual_20250129', 'add is_deleted flag to messages', NOW());
```

---

## Common Migration Patterns

### Add nullable column (safe)
```sql
ALTER TABLE messages ADD COLUMN metadata JSONB;
```

### Add non-nullable column (requires default)
```sql
ALTER TABLE messages ADD COLUMN role VARCHAR(50) NOT NULL DEFAULT 'user';
```

### Add column with check constraint
```sql
ALTER TABLE messages ADD COLUMN role VARCHAR(50);
ALTER TABLE messages ADD CONSTRAINT check_role 
  CHECK (role IN ('user', 'assistant', 'system'));
```

### Create index concurrently (no table lock)
```sql
CREATE INDEX CONCURRENTLY idx_messages_created_at 
ON messages(created_at);
```

### Add foreign key to existing table
```sql
ALTER TABLE messages ADD COLUMN user_id UUID;
ALTER TABLE messages ADD CONSTRAINT fk_messages_user 
  FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE SET NULL;
CREATE INDEX idx_messages_user_id ON messages(user_id);
```

### Modify JSONB field
```sql
-- Add a key to existing JSONB
UPDATE user_profiles 
SET preferences = preferences || '{"theme": "dark"}'::jsonb
WHERE preferences IS NOT NULL;
```

---

## Rollback Strategies

### Method 1: Save rollback SQL
```sql
-- migration_up.sql
ALTER TABLE users ADD COLUMN phone VARCHAR(20);

-- migration_down.sql (saved separately)
ALTER TABLE users DROP COLUMN phone;
```

### Method 2: Use transactions
```sql
BEGIN;
  -- your changes
  ALTER TABLE users ADD COLUMN phone VARCHAR(20);
  
  -- test query
  SELECT * FROM users LIMIT 1;
  
  -- if something wrong: ROLLBACK
  -- if all good: COMMIT
COMMIT;
```

### Method 3: Database backup
```bash
# Before migration
pg_dump $DATABASE_URL > backup_before_migration.sql

# After migration (if needed)
psql $DATABASE_URL < backup_before_migration.sql
```

---

## Neon-Specific Considerations

### Connection Pooling
Neon uses connection pooling. For migrations:
```python
# Use direct connection (not pooled)
DATABASE_URL = os.getenv('DATABASE_URL').replace('?', '?options=endpoint%3Ddirect&')
```

### Branching for Testing
```bash
# Create a branch for testing migrations
neonctl branches create --name migration-test

# Get branch connection string
neonctl connection-string migration-test

# Test migration
psql <branch-url> -f migration.sql

# If good, apply to main
# If bad, delete branch
neonctl branches delete migration-test
```

### Auto-suspend
Neon databases auto-suspend after inactivity. Migrations might wake them up:
```python
# First query might be slower due to wake-up
conn = psycopg2.connect(DATABASE_URL)
# Give it a moment
time.sleep(1)
```

---

## Troubleshooting

### Migration already applied
```
ERROR: duplicate key value violates unique constraint "schema_migrations_version_key"
```
**Fix:** Check what was applied:
```sql
SELECT * FROM schema_migrations ORDER BY applied_at DESC;
```

### Column already exists
```
ERROR: column "phone" of relation "users" already exists
```
**Fix:** Use `IF NOT EXISTS` or check first:
```sql
ALTER TABLE users ADD COLUMN IF NOT EXISTS phone VARCHAR(20);
```

### Foreign key violation
```
ERROR: insert or update on table "messages" violates foreign key constraint
```
**Fix:** Ensure referenced records exist:
```sql
-- Check orphaned records
SELECT m.* FROM messages m
LEFT JOIN chat_sessions s ON m.session_id = s.id
WHERE s.id IS NULL;
```

### Lock timeout during migration
```
ERROR: canceling statement due to lock timeout
```
**Fix:** Use `CONCURRENTLY` for indexes:
```sql
CREATE INDEX CONCURRENTLY idx_name ON table(column);
```

---

## Migration Checklist

Before applying any migration:

- [ ] Tested on local/dev database
- [ ] Rollback SQL documented
- [ ] Migration tracked in `schema_migrations`
- [ ] Breaking changes communicated to team
- [ ] Database backup created (for production)
- [ ] No long-running locks expected
- [ ] Compatible with current application code
- [ ] Indexes created `CONCURRENTLY` if needed