# Data Model: Authentication & Onboarding Profile

**Feature**: 013-authentication-and-onboarding
**Date**: 2026-02-19

## Overview

This feature introduces two layers of data:
1. **Better-Auth managed tables** — auto-created by Better-Auth in the shared Neon database (user, session, account, verification)
2. **Application tables** — custom `user_profiles` table for onboarding questionnaire data, managed by our migration system

The existing spec-010 tables (`users`, `sessions`, `messages`) will be **migrated**: the old `users` table will be renamed to `legacy_users` and a foreign-key bridge will link old anonymous sessions to Better-Auth user IDs.

## Entity Relationship Diagram

```
┌──────────────────────┐
│   BA: user           │  (Better-Auth managed)
│──────────────────────│
│ id         TEXT PK   │
│ name       TEXT       │
│ email      TEXT UQ    │
│ emailVerified BOOL   │
│ image      TEXT       │
│ createdAt  TIMESTAMP │
│ updatedAt  TIMESTAMP │
└──────┬───────────────┘
       │ 1
       │
       │ 1..N
┌──────┴───────────────┐
│   BA: session        │  (Better-Auth managed)
│──────────────────────│
│ id         TEXT PK   │
│ userId     TEXT FK   │
│ token      TEXT UQ   │  ← Bearer token sent by client
│ expiresAt  TIMESTAMP │
│ ipAddress  TEXT       │
│ userAgent  TEXT       │
│ createdAt  TIMESTAMP │
│ updatedAt  TIMESTAMP │
└──────────────────────┘

┌──────────────────────┐
│   BA: account        │  (Better-Auth managed)
│──────────────────────│
│ id         TEXT PK   │
│ userId     TEXT FK   │
│ accountId  TEXT       │
│ providerId TEXT       │  ← "credential" for email/password
│ password   TEXT       │  ← hashed password
│ createdAt  TIMESTAMP │
│ updatedAt  TIMESTAMP │
└──────────────────────┘

┌──────────────────────┐
│   BA: verification   │  (Better-Auth managed)
│──────────────────────│
│ id         TEXT PK   │
│ identifier TEXT       │
│ value      TEXT       │
│ expiresAt  TIMESTAMP │
│ createdAt  TIMESTAMP │
│ updatedAt  TIMESTAMP │
└──────────────────────┘

       ┌──────────────────────┐
       │   user_profiles      │  (Application managed)
       │──────────────────────│
       │ id           UUID PK │
       │ user_id      TEXT FK │──→ BA: user.id (UNIQUE)
       │ software_level VARCHAR(20)  │  ← beginner|intermediate|advanced
       │ programming_languages TEXT  │  ← free-text, max 200 chars
       │ hardware_level VARCHAR(20)  │  ← none|hobbyist|academic|professional
       │ available_hardware TEXT[]   │  ← array of predefined options
       │ learning_goal TEXT          │  ← free-text, max 500 chars
       │ preferred_pace VARCHAR(20)  │  ← self_paced|structured_weekly
       │ onboarding_completed BOOL  │  ← default false
       │ created_at  TIMESTAMPTZ    │
       │ updated_at  TIMESTAMPTZ    │
       └──────────────────────┘

       ┌──────────────────────┐
       │   chat_sessions      │  (Renamed from sessions, FK updated)
       │──────────────────────│
       │ id          UUID PK  │
       │ user_id     TEXT FK  │──→ BA: user.id (was UUID FK → users.id)
       │ title       VARCHAR  │
       │ status      VARCHAR  │
       │ created_at  TIMESTAMPTZ │
       │ updated_at  TIMESTAMPTZ │
       └──────────────────────┘
```

## Table Details

### Better-Auth Tables (auto-managed)

Better-Auth auto-creates and manages these tables. We do NOT write migrations for them. Better-Auth uses TEXT IDs (not UUIDs) by default.

| Table | Purpose | Key Columns |
|-------|---------|-------------|
| `user` | Registered user accounts | id, name, email, emailVerified, image, createdAt, updatedAt |
| `session` | Active auth sessions | id, userId, token (the bearer token), expiresAt, ipAddress, userAgent |
| `account` | Auth provider links | id, userId, accountId, providerId ("credential" for email/pw), password (hashed) |
| `verification` | Email/password reset tokens | id, identifier, value, expiresAt |

### Application Tables (our migrations)

#### `user_profiles` (NEW — migration 002)

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PK, DEFAULT gen_random_uuid() | Profile record ID |
| user_id | TEXT | FK → user.id, UNIQUE, NOT NULL | Links to Better-Auth user |
| software_level | VARCHAR(20) | NOT NULL, DEFAULT 'beginner' | beginner, intermediate, advanced |
| programming_languages | TEXT | DEFAULT '' | Free-text, max 200 chars (enforced at app layer) |
| hardware_level | VARCHAR(20) | NOT NULL, DEFAULT 'none' | none, hobbyist, academic, professional |
| available_hardware | TEXT[] | DEFAULT '{}' | Array of predefined hardware options |
| learning_goal | TEXT | DEFAULT '' | Free-text, max 500 chars (enforced at app layer) |
| preferred_pace | VARCHAR(20) | NOT NULL, DEFAULT 'self_paced' | self_paced, structured_weekly |
| onboarding_completed | BOOLEAN | NOT NULL, DEFAULT false | Whether user has completed onboarding |
| created_at | TIMESTAMPTZ | NOT NULL, DEFAULT now() | Record creation time |
| updated_at | TIMESTAMPTZ | NOT NULL, DEFAULT now() | Last modification time |

**Indexes**:
- `idx_user_profiles_user_id` on `user_id` (UNIQUE already creates one)

**Check constraints**:
- `software_level IN ('beginner', 'intermediate', 'advanced')`
- `hardware_level IN ('none', 'hobbyist', 'academic', 'professional')`
- `preferred_pace IN ('self_paced', 'structured_weekly')`

#### Predefined `available_hardware` options

These are the canonical values for the `available_hardware` array:
- `jetson_nano_orin`
- `raspberry_pi`
- `ros2_workstation`
- `gpu_workstation`
- `simulation_only`

### Migration Strategy

#### Migration 002: Create user_profiles + bridge legacy users

```sql
-- 002_auth_onboarding.sql

-- 1. Create user_profiles table
CREATE TABLE IF NOT EXISTS user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id TEXT UNIQUE NOT NULL,
    software_level VARCHAR(20) NOT NULL DEFAULT 'beginner'
        CHECK (software_level IN ('beginner', 'intermediate', 'advanced')),
    programming_languages TEXT NOT NULL DEFAULT '',
    hardware_level VARCHAR(20) NOT NULL DEFAULT 'none'
        CHECK (hardware_level IN ('none', 'hobbyist', 'academic', 'professional')),
    available_hardware TEXT[] NOT NULL DEFAULT '{}',
    learning_goal TEXT NOT NULL DEFAULT '',
    preferred_pace VARCHAR(20) NOT NULL DEFAULT 'self_paced'
        CHECK (preferred_pace IN ('self_paced', 'structured_weekly')),
    onboarding_completed BOOLEAN NOT NULL DEFAULT false,
    created_at TIMESTAMPTZ NOT NULL DEFAULT now(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT now()
);

-- 2. Add better_auth_user_id to existing sessions table for migration bridge
ALTER TABLE sessions ADD COLUMN IF NOT EXISTS ba_user_id TEXT;

-- 3. Index for the new column
CREATE INDEX IF NOT EXISTS idx_sessions_ba_user_id ON sessions (ba_user_id);
```

#### Migration 003: Anonymous session migration function

```sql
-- 003_anon_migration_function.sql

-- Function to migrate anonymous sessions to authenticated user
CREATE OR REPLACE FUNCTION migrate_anonymous_sessions(
    p_anon_token TEXT,
    p_ba_user_id TEXT
) RETURNS INTEGER AS $$
DECLARE
    migrated_count INTEGER;
BEGIN
    UPDATE sessions s
    SET ba_user_id = p_ba_user_id
    FROM users u
    WHERE u.id = s.user_id
      AND u.token = p_anon_token
      AND s.ba_user_id IS NULL;

    GET DIAGNOSTICS migrated_count = ROW_COUNT;
    RETURN migrated_count;
END;
$$ LANGUAGE plpgsql;
```

## State Transitions

### User Lifecycle

```
Anonymous → [Register] → Registered (onboarding_completed=false)
                            → [Complete questionnaire] → Registered (onboarding_completed=true)
                            → [Skip questionnaire] → Registered (onboarding_completed=true, defaults)
Registered → [Login] → Authenticated session
Authenticated → [Logout] → Session ended
Authenticated → [Token expires] → Session ended
```

### Profile Update Flow

```
onboarding_completed=false → [Submit profile] → onboarding_completed=true, updated_at=now()
onboarding_completed=true → [Update profile] → updated_at=now() (onboarding_completed stays true)
```

## Validation Rules

| Field | Rule | Error |
|-------|------|-------|
| email | Valid email format, case-insensitive unique | "Invalid email" / "Email already registered" |
| password | Minimum 8 characters | "Password must be at least 8 characters" |
| software_level | Must be one of: beginner, intermediate, advanced | "Invalid software level" |
| hardware_level | Must be one of: none, hobbyist, academic, professional | "Invalid hardware level" |
| available_hardware | Each item must be from predefined list | "Invalid hardware option" |
| programming_languages | Max 200 characters | "Programming languages too long" |
| learning_goal | Max 500 characters | "Learning goal too long" |
| preferred_pace | Must be one of: self_paced, structured_weekly | "Invalid pace preference" |
