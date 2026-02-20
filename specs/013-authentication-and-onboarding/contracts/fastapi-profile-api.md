# FastAPI Profile & Auth Integration API Contract

**Service**: `backend/` (existing FastAPI app)
**Base URL**: `/api/v1` (existing prefix)
**Auth**: Bearer token (validated against Better-Auth `session` table in shared Neon DB)

## New Endpoints

### GET /api/v1/profile

Retrieve the authenticated user's onboarding profile.

**Headers**:
```
Authorization: Bearer <token>
```

**Response (200)** — Profile exists:
```json
{
  "user_id": "string (Better-Auth user ID)",
  "software_level": "beginner | intermediate | advanced",
  "programming_languages": "string",
  "hardware_level": "none | hobbyist | academic | professional",
  "available_hardware": ["jetson_nano_orin", "simulation_only"],
  "learning_goal": "string",
  "preferred_pace": "self_paced | structured_weekly",
  "onboarding_completed": true,
  "created_at": "ISO8601",
  "updated_at": "ISO8601"
}
```

**Response (200)** — No profile yet:
```json
{
  "user_id": "string",
  "software_level": "beginner",
  "programming_languages": "",
  "hardware_level": "none",
  "available_hardware": [],
  "learning_goal": "",
  "preferred_pace": "self_paced",
  "onboarding_completed": false,
  "created_at": null,
  "updated_at": null
}
```

**Errors**:
- 401: Invalid or expired token

---

### POST /api/v1/profile

Create or fully replace the user's onboarding profile. Used on initial onboarding submission.

**Headers**:
```
Authorization: Bearer <token>
```

**Request**:
```json
{
  "software_level": "intermediate",
  "programming_languages": "Python, C++",
  "hardware_level": "hobbyist",
  "available_hardware": ["jetson_nano_orin", "simulation_only"],
  "learning_goal": "Build a humanoid robot controller",
  "preferred_pace": "self_paced"
}
```

**Validation**:
- `software_level`: required, must be one of `beginner`, `intermediate`, `advanced`
- `programming_languages`: optional, max 200 characters
- `hardware_level`: required, must be one of `none`, `hobbyist`, `academic`, `professional`
- `available_hardware`: optional, each item must be one of predefined options
- `learning_goal`: optional, max 500 characters
- `preferred_pace`: required, must be one of `self_paced`, `structured_weekly`

**Response (201)**:
```json
{
  "user_id": "string",
  "software_level": "intermediate",
  "programming_languages": "Python, C++",
  "hardware_level": "hobbyist",
  "available_hardware": ["jetson_nano_orin", "simulation_only"],
  "learning_goal": "Build a humanoid robot controller",
  "preferred_pace": "self_paced",
  "onboarding_completed": true,
  "created_at": "ISO8601",
  "updated_at": "ISO8601"
}
```

**Errors**:
- 401: Invalid or expired token
- 422: Validation error (invalid field values, length exceeded)

---

### PATCH /api/v1/profile

Partially update the user's onboarding profile. Used when editing individual fields.

**Headers**:
```
Authorization: Bearer <token>
```

**Request** (all fields optional):
```json
{
  "software_level": "advanced",
  "available_hardware": ["jetson_nano_orin", "gpu_workstation"]
}
```

**Response (200)**: Full updated profile (same shape as GET response).

**Errors**:
- 401: Invalid or expired token
- 404: Profile not found (user hasn't completed onboarding)
- 422: Validation error

---

### POST /api/v1/auth/migrate-sessions

Migrate anonymous chat sessions to the authenticated user. Called by the frontend after first login/registration.

**Headers**:
```
Authorization: Bearer <token>
```

**Request**:
```json
{
  "anonymous_token": "anon-192.168.1.1"
}
```

**Response (200)**:
```json
{
  "migrated_count": 3
}
```

**Errors**:
- 401: Invalid or expired token
- 404: No anonymous sessions found for the given token

---

## Modified Existing Endpoints

### Updated: `get_current_user` Dependency

The existing `get_current_user` FastAPI dependency will be updated:

**Current behavior**: Looks up user in `users` table by token, falls back to anonymous.

**New behavior**:
1. Extract Bearer token from `Authorization` header
2. Query Better-Auth `session` table: `SELECT * FROM session WHERE token = $1 AND "expiresAt" > now()`
3. If valid session found → return `{id: session.userId, tier: "identified", ...}`
4. If no valid session → fall back to anonymous user creation (existing behavior for backward compatibility per FR-014)

```python
# Pseudocode for updated dependency
async def get_current_user(request, pool):
    token = extract_bearer_token(request)
    if token and not token.startswith("anon-"):
        # Check Better-Auth session table
        session = await query_ba_session(pool, token)
        if session and session["expiresAt"] > now():
            return {"id": session["userId"], "tier": "identified", "token": token}
    # Fall back to anonymous
    return create_or_get_anonymous_user(request, pool)
```

## Authentication Flow Sequence

```
┌──────────┐     ┌──────────────┐     ┌──────────────┐     ┌──────┐
│  Browser  │     │ Better-Auth  │     │   FastAPI     │     │ Neon │
│ (Docusaurus)│   │  (Node.js)   │     │  (Python)     │     │  DB  │
└─────┬────┘     └──────┬───────┘     └──────┬───────┘     └──┬───┘
      │                  │                     │                │
      │ 1. POST /api/auth/sign-up/email        │                │
      │─────────────────>│                     │                │
      │                  │ 2. INSERT user, session, account     │
      │                  │─────────────────────────────────────>│
      │                  │                     │                │
      │ 3. 200 + set-auth-token header         │                │
      │<─────────────────│                     │                │
      │                  │                     │                │
      │ 4. Store token in localStorage         │                │
      │                  │                     │                │
      │ 5. POST /api/v1/profile (Bearer token) │                │
      │────────────────────────────────────────>│                │
      │                  │                     │ 6. SELECT session WHERE token=$1
      │                  │                     │───────────────>│
      │                  │                     │ 7. Valid session│
      │                  │                     │<───────────────│
      │                  │                     │ 8. INSERT user_profiles
      │                  │                     │───────────────>│
      │ 9. 201 Profile created                 │                │
      │<───────────────────────────────────────│                │
      │                  │                     │                │
      │ 10. POST /api/v1/chat (Bearer token)   │                │
      │────────────────────────────────────────>│                │
      │                  │                     │ 11. Validate session
      │                  │                     │───────────────>│
      │                  │                     │ 12. Process chat│
      │ 13. SSE stream response                │                │
      │<═══════════════════════════════════════│                │
```
