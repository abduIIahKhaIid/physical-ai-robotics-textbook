-- Migration: --help
-- Created: 2026-01-29T13:59:24.711541
-- Version: 20260129135924

-- ============================================
-- UP Migration
-- ============================================

-- TODO: Add your schema changes here
-- Example:
-- ALTER TABLE messages ADD COLUMN new_field TEXT;

-- Record migration
INSERT INTO schema_migrations (version, description) 
VALUES ('20260129135924', '--help');

-- ============================================
-- DOWN Migration (Rollback)
-- ============================================

-- To rollback this migration, run:
-- TODO: Add rollback SQL here
-- Example:
-- ALTER TABLE messages DROP COLUMN new_field;
-- DELETE FROM schema_migrations WHERE version = '20260129135924';