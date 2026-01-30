#!/usr/bin/env python3
"""
Setup Alembic for Neon Postgres migrations.
Generates alembic.ini and env.py configured for Neon.
"""
import os
import sys
from pathlib import Path

ALEMBIC_INI = """# Alembic Configuration for Neon Postgres

[alembic]
script_location = alembic
prepend_sys_path = .
version_path_separator = os

[post_write_hooks]

[loggers]
keys = root,sqlalchemy,alembic

[handlers]
keys = console

[formatters]
keys = generic

[logger_root]
level = WARN
handlers = console
qualname =

[logger_sqlalchemy]
level = WARN
handlers =
qualname = sqlalchemy.engine

[logger_alembic]
level = INFO
handlers =
qualname = alembic

[handler_console]
class = StreamHandler
args = (sys.stderr,)
level = NOTSET
formatter = generic

[formatter_generic]
format = %(levelname)-5.5s [%(name)s] %(message)s
datefmt = %H:%M:%S
"""

ENV_PY = """import os
from logging.config import fileConfig
from sqlalchemy import engine_from_config, pool
from alembic import context

# Alembic Config object
config = context.config

# Interpret the config file for Python logging
if config.config_file_name is not None:
    fileConfig(config.config_file_name)

# Set target metadata (import your models here)
# from myapp.models import Base
# target_metadata = Base.metadata
target_metadata = None

def get_url():
    \"\"\"Get database URL from environment.\"\"\"
    return os.getenv('DATABASE_URL', config.get_main_option("sqlalchemy.url"))

def run_migrations_offline() -> None:
    \"\"\"Run migrations in 'offline' mode.\"\"\"
    url = get_url()
    context.configure(
        url=url,
        target_metadata=target_metadata,
        literal_binds=True,
        dialect_opts={"paramstyle": "named"},
    )

    with context.begin_transaction():
        context.run_migrations()

def run_migrations_online() -> None:
    \"\"\"Run migrations in 'online' mode.\"\"\"
    configuration = config.get_section(config.config_ini_section)
    configuration["sqlalchemy.url"] = get_url()
    
    connectable = engine_from_config(
        configuration,
        prefix="sqlalchemy.",
        poolclass=pool.NullPool,
    )

    with connectable.connect() as connection:
        context.configure(
            connection=connection, 
            target_metadata=target_metadata
        )

        with context.begin_transaction():
            context.run_migrations()

if context.is_offline_mode():
    run_migrations_offline()
else:
    run_migrations_online()
"""

def setup_alembic():
    """Setup Alembic for Neon Postgres."""
    # Create alembic directory
    alembic_dir = Path('alembic')
    alembic_dir.mkdir(exist_ok=True)
    
    versions_dir = alembic_dir / 'versions'
    versions_dir.mkdir(exist_ok=True)
    
    # Write alembic.ini
    Path('alembic.ini').write_text(ALEMBIC_INI)
    print("✅ Created alembic.ini")
    
    # Write env.py
    (alembic_dir / 'env.py').write_text(ENV_PY)
    print("✅ Created alembic/env.py")
    
    # Write script.py.mako
    (alembic_dir / 'script.py.mako').write_text("""\"\"\"${message}

Revision ID: ${up_revision}
Revises: ${down_revision | comma,n}
Create Date: ${create_date}

\"\"\"
from typing import Sequence, Union
from alembic import op
import sqlalchemy as sa
${imports if imports else ""}

# revision identifiers, used by Alembic.
revision: str = ${repr(up_revision)}
down_revision: Union[str, None] = ${repr(down_revision)}
branch_labels: Union[str, Sequence[str], None] = ${repr(branch_labels)}
depends_on: Union[str, Sequence[str], None] = ${repr(depends_on)}

def upgrade() -> None:
    ${upgrades if upgrades else "pass"}

def downgrade() -> None:
    ${downgrades if downgrades else "pass"}
""")
    print("✅ Created alembic/script.py.mako")
    
    print("\n🎉 Alembic setup complete!")
    print("\nNext steps:")
    print("1. Set DATABASE_URL environment variable")
    print("2. Create initial migration: alembic revision --autogenerate -m 'initial'")
    print("3. Apply migrations: alembic upgrade head")
    print("\nUseful commands:")
    print("  alembic revision -m 'description'  # Create new migration")
    print("  alembic upgrade head               # Apply all migrations")
    print("  alembic downgrade -1               # Rollback one migration")
    print("  alembic history                    # Show migration history")

if __name__ == '__main__':
    setup_alembic()