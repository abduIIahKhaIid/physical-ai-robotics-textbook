-- 003_anon_migration_function.sql
-- Function to migrate anonymous sessions to an authenticated user.

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
