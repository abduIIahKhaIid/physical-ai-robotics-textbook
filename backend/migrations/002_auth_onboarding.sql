-- 002_auth_onboarding.sql
-- Create user_profiles table for onboarding questionnaire data
-- and add Better-Auth bridge column to existing sessions table.

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

-- 2. Add better_auth user_id bridge to existing sessions table
ALTER TABLE sessions ADD COLUMN IF NOT EXISTS ba_user_id TEXT;

-- 3. Index for the bridge column
CREATE INDEX IF NOT EXISTS idx_sessions_ba_user_id ON sessions (ba_user_id);
