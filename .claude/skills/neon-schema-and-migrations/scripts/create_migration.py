#!/usr/bin/env python3
"""
Generate a new migration file for Neon Postgres.
Creates timestamped SQL migration with up/down templates.
"""
import sys
from datetime import datetime
from pathlib import Path

MIGRATION_TEMPLATE = """-- Migration: {description}
-- Created: {timestamp}
-- Version: {version}

-- ============================================
-- UP Migration
-- ============================================

-- TODO: Add your schema changes here
-- Example:
-- ALTER TABLE messages ADD COLUMN new_field TEXT;

-- Record migration
INSERT INTO schema_migrations (version, description) 
VALUES ('{version}', '{description}');

-- ============================================
-- DOWN Migration (Rollback)
-- ============================================

-- To rollback this migration, run:
-- TODO: Add rollback SQL here
-- Example:
-- ALTER TABLE messages DROP COLUMN new_field;
-- DELETE FROM schema_migrations WHERE version = '{version}';
"""

def create_migration(description, output_dir='migrations'):
    """
    Create a new migration file.
    
    Args:
        description: Brief description of the migration
        output_dir: Directory to save migration file
    """
    # Create migrations directory if it doesn't exist
    migrations_path = Path(output_dir)
    migrations_path.mkdir(exist_ok=True)
    
    # Generate version number (timestamp-based)
    timestamp = datetime.now()
    version = timestamp.strftime('%Y%m%d%H%M%S')
    
    # Sanitize description for filename
    filename_desc = description.lower().replace(' ', '_').replace('/', '_')
    filename_desc = ''.join(c for c in filename_desc if c.isalnum() or c == '_')
    
    # Create filename
    filename = f"{version}_{filename_desc}.sql"
    filepath = migrations_path / filename
    
    # Write migration file
    content = MIGRATION_TEMPLATE.format(
        description=description,
        timestamp=timestamp.isoformat(),
        version=version
    )
    
    filepath.write_text(content)
    
    print(f"✅ Created migration: {filepath}")
    print(f"   Version: {version}")
    print(f"   Description: {description}")
    print(f"\nNext steps:")
    print(f"1. Edit {filepath} to add your SQL changes")
    print(f"2. Test the migration on a dev database")
    print(f"3. Apply with: psql $DATABASE_URL -f {filepath}")
    
    return filepath

def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: create_migration.py <description> [output_dir]")
        print("\nExample:")
        print("  create_migration.py 'add user preferences table'")
        print("  create_migration.py 'add indexes for messages' ./db/migrations")
        sys.exit(1)
    
    description = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else 'migrations'
    
    create_migration(description, output_dir)

if __name__ == '__main__':
    main()