"""Tests for database migration SQL files."""

from __future__ import annotations

from pathlib import Path

import pytest

MIGRATIONS_DIR = Path(__file__).parent.parent.parent.parent / "backend" / "migrations"


class TestMigration002:
    """Verify migration 002_auth_onboarding.sql content."""

    def test_002_migration_file_exists(self):
        """Migration 002 file exists."""
        path = MIGRATIONS_DIR / "002_auth_onboarding.sql"
        assert path.exists(), f"Migration file not found: {path}"

    def test_002_creates_user_profiles_table(self):
        """T063: Migration 002 contains CREATE TABLE user_profiles."""
        sql = (MIGRATIONS_DIR / "002_auth_onboarding.sql").read_text()
        assert "CREATE TABLE" in sql
        assert "user_profiles" in sql

    def test_002_has_required_columns(self):
        """Migration 002 includes all required columns."""
        sql = (MIGRATIONS_DIR / "002_auth_onboarding.sql").read_text()
        required = [
            "user_id", "software_level", "programming_languages",
            "hardware_level", "available_hardware", "learning_goal",
            "preferred_pace", "onboarding_completed",
            "created_at", "updated_at",
        ]
        for col in required:
            assert col in sql, f"Missing column: {col}"

    def test_002_has_check_constraints(self):
        """Migration 002 includes CHECK constraints for enum columns."""
        sql = (MIGRATIONS_DIR / "002_auth_onboarding.sql").read_text()
        assert "beginner" in sql
        assert "intermediate" in sql
        assert "advanced" in sql
        assert "hobbyist" in sql
        assert "self_paced" in sql

    def test_002_adds_ba_user_id_to_sessions(self):
        """Migration 002 adds ba_user_id column to sessions table."""
        sql = (MIGRATIONS_DIR / "002_auth_onboarding.sql").read_text()
        assert "ba_user_id" in sql
        assert "ALTER TABLE sessions" in sql

    def test_002_creates_index(self):
        """Migration 002 creates index on ba_user_id."""
        sql = (MIGRATIONS_DIR / "002_auth_onboarding.sql").read_text()
        assert "idx_sessions_ba_user_id" in sql


class TestMigration003:
    """Verify migration 003_anon_migration_function.sql content."""

    def test_003_migration_file_exists(self):
        """Migration 003 file exists."""
        path = MIGRATIONS_DIR / "003_anon_migration_function.sql"
        assert path.exists(), f"Migration file not found: {path}"

    def test_003_creates_migration_function(self):
        """T064: Migration 003 creates migrate_anonymous_sessions function."""
        sql = (MIGRATIONS_DIR / "003_anon_migration_function.sql").read_text()
        assert "CREATE OR REPLACE FUNCTION" in sql
        assert "migrate_anonymous_sessions" in sql

    def test_003_function_has_correct_params(self):
        """Migration 003 function accepts anon_token and ba_user_id."""
        sql = (MIGRATIONS_DIR / "003_anon_migration_function.sql").read_text()
        assert "p_anon_token" in sql
        assert "p_ba_user_id" in sql

    def test_003_returns_integer(self):
        """Migration 003 function returns INTEGER."""
        sql = (MIGRATIONS_DIR / "003_anon_migration_function.sql").read_text()
        assert "RETURNS INTEGER" in sql
