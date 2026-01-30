#!/usr/bin/env python3
"""
Initialize Neon Postgres database with RAG chatbot schema.
Supports both full initialization and incremental migrations.
"""
import os
import sys
from datetime import datetime
from pathlib import Path
import psycopg2
from psycopg2.extensions import ISOLATION_LEVEL_AUTOCOMMIT

def get_connection_string():
    """Get Neon connection string from environment."""
    conn_str = os.getenv('DATABASE_URL')
    if not conn_str:
        print("❌ Error: DATABASE_URL environment variable not set")
        sys.exit(1)
    return conn_str

def connect_db():
    """Connect to Neon Postgres database."""
    try:
        conn = psycopg2.connect(get_connection_string())
        return conn
    except Exception as e:
        print(f"❌ Database connection failed: {e}")
        sys.exit(1)

def init_schema(conn, scope='all'):
    """
    Initialize database schema.
    
    Args:
        conn: Database connection
        scope: 'chat', 'auth', or 'all'
    """
    cursor = conn.cursor()
    
    # Create migrations table if it doesn't exist
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS schema_migrations (
            id SERIAL PRIMARY KEY,
            version VARCHAR(255) UNIQUE NOT NULL,
            applied_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            description TEXT
        );
    """)
    
    migrations = []
    
    if scope in ['auth', 'all']:
        migrations.append(('001_create_users', """
            CREATE TABLE IF NOT EXISTS users (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                email VARCHAR(255) UNIQUE NOT NULL,
                name VARCHAR(255),
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
            
            CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);
        """, "Create users table"))
        
        migrations.append(('002_create_user_profiles', """
            CREATE TABLE IF NOT EXISTS user_profiles (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
                software_background TEXT,
                hardware_background TEXT,
                learning_goals TEXT,
                experience_level VARCHAR(50),
                preferred_language VARCHAR(10) DEFAULT 'en',
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                UNIQUE(user_id)
            );
            
            CREATE INDEX IF NOT EXISTS idx_user_profiles_user_id ON user_profiles(user_id);
        """, "Create user profiles for onboarding data"))
    
    if scope in ['chat', 'all']:
        migrations.append(('003_create_chat_sessions', """
            CREATE TABLE IF NOT EXISTS chat_sessions (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                user_id UUID REFERENCES users(id) ON DELETE CASCADE,
                title VARCHAR(500),
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                metadata JSONB DEFAULT '{}'
            );
            
            CREATE INDEX IF NOT EXISTS idx_chat_sessions_user_id ON chat_sessions(user_id);
            CREATE INDEX IF NOT EXISTS idx_chat_sessions_created_at ON chat_sessions(created_at DESC);
        """, "Create chat sessions table"))
        
        migrations.append(('004_create_messages', """
            CREATE TABLE IF NOT EXISTS messages (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
                role VARCHAR(50) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
                content TEXT NOT NULL,
                selected_text TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                metadata JSONB DEFAULT '{}'
            );
            
            CREATE INDEX IF NOT EXISTS idx_messages_session_id ON messages(session_id);
            CREATE INDEX IF NOT EXISTS idx_messages_created_at ON messages(created_at);
        """, "Create messages table with selected_text support"))
        
        migrations.append(('005_create_message_citations', """
            CREATE TABLE IF NOT EXISTS message_citations (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                message_id UUID NOT NULL REFERENCES messages(id) ON DELETE CASCADE,
                document_id VARCHAR(255),
                chunk_id VARCHAR(255),
                score FLOAT,
                citation_index INTEGER,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
            
            CREATE INDEX IF NOT EXISTS idx_message_citations_message_id ON message_citations(message_id);
            CREATE INDEX IF NOT EXISTS idx_message_citations_document_id ON message_citations(document_id);
        """, "Create message citations for RAG sources"))
    
    # Apply migrations
    for version, sql, description in migrations:
        cursor.execute(
            "SELECT version FROM schema_migrations WHERE version = %s",
            (version,)
        )
        if cursor.fetchone():
            print(f"⏭️  Skipping {version}: {description} (already applied)")
            continue
        
        try:
            cursor.execute(sql)
            cursor.execute(
                "INSERT INTO schema_migrations (version, description) VALUES (%s, %s)",
                (version, description)
            )
            conn.commit()
            print(f"✅ Applied {version}: {description}")
        except Exception as e:
            conn.rollback()
            print(f"❌ Failed to apply {version}: {e}")
            raise
    
    cursor.close()

def main():
    """Main entry point."""
    import argparse
    parser = argparse.ArgumentParser(description='Initialize Neon database schema')
    parser.add_argument('--scope', choices=['chat', 'auth', 'all'], default='all',
                       help='Scope of initialization (default: all)')
    parser.add_argument('--dry-run', action='store_true',
                       help='Show what would be done without applying')
    args = parser.parse_args()
    
    print(f"🚀 Initializing Neon database (scope: {args.scope})")
    
    if args.dry_run:
        print("⚠️  DRY RUN MODE - No changes will be made")
        return
    
    conn = connect_db()
    try:
        init_schema(conn, args.scope)
        print("✅ Database initialization complete!")
    finally:
        conn.close()

if __name__ == '__main__':
    main()