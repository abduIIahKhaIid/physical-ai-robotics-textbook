# Quickstart: Authentication & Onboarding (013)

## Prerequisites

- Node.js 18+ (for Better-Auth service)
- Python 3.11+ (for FastAPI backend)
- Neon Postgres database (existing from spec 010)
- Environment variables configured

## Environment Variables

Add to `.env` (never commit):

```bash
# Existing
DATABASE_URL=postgresql://user:pass@host/db
GEMINI_API_KEY=your-key
CORS_ORIGINS=["http://localhost:3000","http://localhost:4000"]

# New: Better-Auth service
BETTER_AUTH_URL=http://localhost:4000
BETTER_AUTH_SECRET=your-secret-min-32-chars  # openssl rand -base64 32
BETTER_AUTH_PORT=4000

# New: Frontend config
NEXT_PUBLIC_AUTH_URL=http://localhost:4000
```

## Local Development Setup

### 1. Install auth service dependencies

```bash
cd auth-service
npm install
```

### 2. Run Better-Auth migration (auto-creates tables)

```bash
cd auth-service
npx @better-auth/cli migrate
```

### 3. Run application migration (user_profiles table)

```bash
cd /workspaces/physical-ai-robotics-textbook
python -m backend.db.migrate
```

### 4. Start services (3 terminals)

```bash
# Terminal 1: Better-Auth service
cd auth-service && npm run dev
# Runs on http://localhost:4000

# Terminal 2: FastAPI backend
uvicorn backend.main:app --reload --port 8000
# Runs on http://localhost:8000

# Terminal 3: Docusaurus dev server
cd website && npm start
# Runs on http://localhost:3000
```

## Smoke Test

### 1. Register a user

```bash
curl -X POST http://localhost:4000/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"name":"Test User","email":"test@example.com","password":"password123"}' \
  -v
# Look for set-auth-token header in response
```

### 2. Verify session

```bash
TOKEN="<token-from-step-1>"
curl http://localhost:4000/api/auth/get-session \
  -H "Authorization: Bearer $TOKEN"
```

### 3. Create onboarding profile

```bash
curl -X POST http://localhost:8000/api/v1/profile \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "software_level": "intermediate",
    "programming_languages": "Python, C++",
    "hardware_level": "hobbyist",
    "available_hardware": ["jetson_nano_orin"],
    "learning_goal": "Build a humanoid robot",
    "preferred_pace": "self_paced"
  }'
# Expect 201 with full profile
```

### 4. Fetch profile

```bash
curl http://localhost:8000/api/v1/profile \
  -H "Authorization: Bearer $TOKEN"
# Expect 200 with onboarding_completed=true
```

### 5. Verify chat still works (backward compatibility)

```bash
# Anonymous (no token)
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"message":"Hello","session_id":null}'
# Should work with anonymous user creation

# Authenticated
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"message":"Hello","session_id":null}'
# Should work with authenticated user
```

## Database Verification

```sql
-- Check Better-Auth tables exist
SELECT tablename FROM pg_tables WHERE tablename IN ('user', 'session', 'account', 'verification');

-- Check application table exists
SELECT tablename FROM pg_tables WHERE tablename = 'user_profiles';

-- Check a profile was saved
SELECT * FROM user_profiles WHERE onboarding_completed = true;
```

## Test Commands

```bash
# Run all tests
pytest tests/ -v

# Run only auth tests
pytest tests/test_auth_dependency.py -v

# Run only profile tests
pytest tests/test_profile_endpoints.py -v

# Run migration test
pytest tests/test_migrations.py -v
```
