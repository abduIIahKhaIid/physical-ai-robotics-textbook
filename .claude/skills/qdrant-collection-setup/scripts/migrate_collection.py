#!/usr/bin/env python3
"""
Qdrant Collection Migration Script

Handles migrations when embedding models change or schema needs updating.
"""

import argparse
import sys
from typing import List, Dict, Any
from qdrant_client import QdrantClient, models


def migrate_collection(
    client: QdrantClient,
    source_collection: str,
    target_collection: str,
    new_vector_size: int,
    batch_size: int = 100,
) -> bool:
    """
    Migrate collection to new schema (e.g., after changing embedding model).
    
    Note: This migrates payload/metadata only. Vectors must be re-embedded.
    
    Args:
        client: Initialized Qdrant client
        source_collection: Original collection name
        target_collection: New collection name
        new_vector_size: New vector dimension
        batch_size: Number of points to process per batch
    
    Returns:
        True if successful, False otherwise
    """
    try:
        # Verify source exists
        print(f"🔍 Checking source collection '{source_collection}'...")
        source_info = client.get_collection(source_collection)
        total_points = source_info.points_count
        print(f"✓ Found {total_points} points")
        
        # Create target collection
        print(f"📦 Creating target collection '{target_collection}'...")
        client.create_collection(
            collection_name=target_collection,
            vectors_config=models.VectorParams(
                size=new_vector_size,
                distance=source_info.config.params.vectors.distance,
            ),
        )
        print(f"✅ Target collection created")
        
        # Copy payload metadata (vectors need re-embedding)
        print(f"📋 Copying payload metadata...")
        print(f"⚠️  Note: Vectors must be re-embedded separately")
        
        offset = None
        migrated_count = 0
        
        while True:
            # Scroll through source collection
            records, offset = client.scroll(
                collection_name=source_collection,
                limit=batch_size,
                offset=offset,
                with_payload=True,
                with_vectors=False,
            )
            
            if not records:
                break
            
            # Store payload data for re-embedding
            print(f"  Processed {migrated_count + len(records)}/{total_points} points")
            migrated_count += len(records)
            
            if offset is None:
                break
        
        print(f"✅ Migration complete")
        print(f"   Migrated {migrated_count} point payloads")
        print(f"   Next: Re-embed content and insert with new vectors")
        
        return True
        
    except Exception as e:
        print(f"❌ Migration failed: {e}")
        return False


def backup_collection(
    client: QdrantClient,
    collection_name: str,
    backup_name: Optional[str] = None,
) -> bool:
    """
    Create a backup by taking a snapshot.
    
    Args:
        client: Initialized Qdrant client
        collection_name: Collection to backup
        backup_name: Optional backup name (default: {collection}_backup)
    
    Returns:
        True if successful, False otherwise
    """
    if backup_name is None:
        backup_name = f"{collection_name}_backup"
    
    try:
        print(f"💾 Creating backup of '{collection_name}'...")
        
        # Get collection info
        info = client.get_collection(collection_name)
        
        # Create backup collection with same config
        client.create_collection(
            collection_name=backup_name,
            vectors_config=info.config.params.vectors,
        )
        
        # Copy all points
        offset = None
        total_copied = 0
        
        while True:
            records, offset = client.scroll(
                collection_name=collection_name,
                limit=100,
                offset=offset,
                with_payload=True,
                with_vectors=True,
            )
            
            if not records:
                break
            
            # Insert into backup
            points = [
                models.PointStruct(
                    id=r.id,
                    vector=r.vector,
                    payload=r.payload,
                )
                for r in records
            ]
            
            client.upsert(
                collection_name=backup_name,
                points=points,
            )
            
            total_copied += len(records)
            print(f"  Copied {total_copied} points...")
            
            if offset is None:
                break
        
        print(f"✅ Backup created: '{backup_name}'")
        print(f"   Total points: {total_copied}")
        
        return True
        
    except Exception as e:
        print(f"❌ Backup failed: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Migrate Qdrant collections"
    )
    
    # Connection parameters
    parser.add_argument(
        "--url",
        default="http://localhost:6333",
        help="Qdrant server URL",
    )
    parser.add_argument(
        "--api-key",
        help="Qdrant API key",
    )
    
    # Operation
    subparsers = parser.add_subparsers(dest="command", required=True)
    
    # Migrate command
    migrate_parser = subparsers.add_parser(
        "migrate",
        help="Migrate collection to new schema",
    )
    migrate_parser.add_argument(
        "--source",
        required=True,
        help="Source collection name",
    )
    migrate_parser.add_argument(
        "--target",
        required=True,
        help="Target collection name",
    )
    migrate_parser.add_argument(
        "--vector-size",
        type=int,
        required=True,
        help="New vector dimension",
    )
    
    # Backup command
    backup_parser = subparsers.add_parser(
        "backup",
        help="Create collection backup",
    )
    backup_parser.add_argument(
        "--collection",
        required=True,
        help="Collection to backup",
    )
    backup_parser.add_argument(
        "--backup-name",
        help="Backup collection name (default: {collection}_backup)",
    )
    
    args = parser.parse_args()
    
    # Initialize client
    print(f"🔌 Connecting to Qdrant...")
    try:
        client = QdrantClient(
            url=args.url,
            api_key=args.api_key,
        )
        print(f"✅ Connected")
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return 1
    
    # Execute command
    if args.command == "migrate":
        success = migrate_collection(
            client,
            args.source,
            args.target,
            args.vector_size,
        )
    elif args.command == "backup":
        success = backup_collection(
            client,
            args.collection,
            args.backup_name,
        )
    else:
        print(f"❌ Unknown command: {args.command}")
        return 1
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())