#!/usr/bin/env python3
"""
Qdrant Collection Setup Script

Provisions Qdrant collections with proper configuration for RAG pipelines.
Handles creation, validation, and safe lifecycle operations.
"""

import argparse
import sys
from typing import Optional, Dict, Any
from qdrant_client import QdrantClient, models


def create_collection(
    client: QdrantClient,
    collection_name: str,
    vector_size: int,
    distance: str = "Cosine",
    recreate: bool = False,
) -> bool:
    """
    Create a Qdrant collection with specified parameters.
    
    Args:
        client: Initialized Qdrant client
        collection_name: Name of the collection
        vector_size: Dimension of the embedding vectors
        distance: Distance metric (Cosine, Euclidean, Dot)
        recreate: If True, delete existing collection before creating
    
    Returns:
        True if successful, False otherwise
    """
    distance_map = {
        "Cosine": models.Distance.COSINE,
        "Euclidean": models.Distance.EUCLID,
        "Dot": models.Distance.DOT,
    }
    
    if distance not in distance_map:
        print(f"❌ Invalid distance metric: {distance}")
        print(f"   Valid options: {', '.join(distance_map.keys())}")
        return False
    
    try:
        # Check if collection exists
        collections = client.get_collections().collections
        exists = any(c.name == collection_name for c in collections)
        
        if exists:
            if recreate:
                print(f"⚠️  Collection '{collection_name}' exists. Deleting...")
                client.delete_collection(collection_name)
                print(f"✅ Deleted existing collection")
            else:
                print(f"⚠️  Collection '{collection_name}' already exists")
                print(f"   Use --recreate to delete and recreate")
                return False
        
        # Create collection
        print(f"📦 Creating collection '{collection_name}'...")
        print(f"   Vector size: {vector_size}")
        print(f"   Distance metric: {distance}")
        
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=vector_size,
                distance=distance_map[distance],
            ),
        )
        
        print(f"✅ Collection created successfully")
        return True
        
    except Exception as e:
        print(f"❌ Error creating collection: {e}")
        return False


def create_payload_indexes(
    client: QdrantClient,
    collection_name: str,
    indexes: Dict[str, str],
) -> bool:
    """
    Create payload indexes for fast filtering.
    
    Args:
        client: Initialized Qdrant client
        collection_name: Name of the collection
        indexes: Dict mapping field names to field types
                 e.g., {"chapter": "keyword", "section": "keyword", "page": "integer"}
    
    Returns:
        True if successful, False otherwise
    """
    field_type_map = {
        "keyword": models.PayloadSchemaType.KEYWORD,
        "integer": models.PayloadSchemaType.INTEGER,
        "float": models.PayloadSchemaType.FLOAT,
        "text": models.PayloadSchemaType.TEXT,
        "bool": models.PayloadSchemaType.BOOL,
    }
    
    try:
        print(f"🔍 Creating payload indexes...")
        
        for field_name, field_type in indexes.items():
            if field_type not in field_type_map:
                print(f"⚠️  Skipping '{field_name}': invalid type '{field_type}'")
                continue
            
            print(f"   Creating index: {field_name} ({field_type})")
            client.create_payload_index(
                collection_name=collection_name,
                field_name=field_name,
                field_schema=field_type_map[field_type],
            )
        
        print(f"✅ Payload indexes created")
        return True
        
    except Exception as e:
        print(f"❌ Error creating indexes: {e}")
        return False


def validate_collection(
    client: QdrantClient,
    collection_name: str,
    expected_vector_size: Optional[int] = None,
) -> bool:
    """
    Validate collection exists and check configuration.
    
    Args:
        client: Initialized Qdrant client
        collection_name: Name of the collection
        expected_vector_size: If provided, verify vector dimension matches
    
    Returns:
        True if validation passes, False otherwise
    """
    try:
        print(f"🔍 Validating collection '{collection_name}'...")
        
        # Get collection info
        info = client.get_collection(collection_name)
        
        # Display configuration
        vector_config = info.config.params.vectors
        print(f"✓ Collection exists")
        print(f"  Vector size: {vector_config.size}")
        print(f"  Distance: {vector_config.distance.name}")
        print(f"  Points count: {info.points_count}")
        
        # Validate vector size if provided
        if expected_vector_size and vector_config.size != expected_vector_size:
            print(f"❌ Vector size mismatch!")
            print(f"   Expected: {expected_vector_size}")
            print(f"   Actual: {vector_config.size}")
            return False
        
        print(f"✅ Validation passed")
        return True
        
    except Exception as e:
        print(f"❌ Validation failed: {e}")
        return False


def insert_test_vector(
    client: QdrantClient,
    collection_name: str,
    vector_size: int,
) -> bool:
    """
    Insert a test vector to verify collection works.
    
    Args:
        client: Initialized Qdrant client
        collection_name: Name of the collection
        vector_size: Dimension of vectors
    
    Returns:
        True if successful, False otherwise
    """
    try:
        print(f"🧪 Inserting test vector...")
        
        import numpy as np
        
        # Create test vector (normalized random)
        test_vector = np.random.randn(vector_size)
        test_vector = test_vector / np.linalg.norm(test_vector)
        
        # Insert test point
        client.upsert(
            collection_name=collection_name,
            points=[
                models.PointStruct(
                    id=0,
                    vector=test_vector.tolist(),
                    payload={
                        "text": "Test vector for validation",
                        "type": "test",
                        "chapter": "validation",
                    },
                )
            ],
        )
        
        print(f"✅ Test vector inserted (ID: 0)")
        
        # Search for the test vector
        print(f"🔍 Testing search...")
        results = client.search(
            collection_name=collection_name,
            query_vector=test_vector.tolist(),
            limit=1,
        )
        
        if results and results[0].id == 0:
            print(f"✅ Search working correctly")
            print(f"   Score: {results[0].score:.4f}")
            return True
        else:
            print(f"❌ Search failed to find test vector")
            return False
            
    except Exception as e:
        print(f"❌ Test insertion failed: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Setup and manage Qdrant collections for RAG pipelines"
    )
    
    # Connection parameters
    parser.add_argument(
        "--url",
        default="http://localhost:6333",
        help="Qdrant server URL (default: http://localhost:6333)",
    )
    parser.add_argument(
        "--api-key",
        help="Qdrant API key (for cloud deployments)",
    )
    
    # Collection parameters
    parser.add_argument(
        "--collection",
        default="textbook_chunks",
        help="Collection name (default: textbook_chunks)",
    )
    parser.add_argument(
        "--vector-size",
        type=int,
        required=True,
        help="Vector dimension (e.g., 1536 for OpenAI ada-002, 3072 for text-embedding-3-large)",
    )
    parser.add_argument(
        "--distance",
        default="Cosine",
        choices=["Cosine", "Euclidean", "Dot"],
        help="Distance metric (default: Cosine)",
    )
    
    # Operations
    parser.add_argument(
        "--recreate",
        action="store_true",
        help="Delete and recreate collection if it exists",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Only validate existing collection",
    )
    parser.add_argument(
        "--skip-test",
        action="store_true",
        help="Skip test vector insertion",
    )
    
    args = parser.parse_args()
    
    # Initialize client
    print(f"🔌 Connecting to Qdrant at {args.url}...")
    try:
        client = QdrantClient(
            url=args.url,
            api_key=args.api_key,
        )
        print(f"✅ Connected to Qdrant")
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return 1
    
    # Validate-only mode
    if args.validate_only:
        success = validate_collection(client, args.collection, args.vector_size)
        return 0 if success else 1
    
    # Create collection
    success = create_collection(
        client,
        args.collection,
        args.vector_size,
        args.distance,
        args.recreate,
    )
    
    if not success:
        return 1
    
    # Create recommended payload indexes
    indexes = {
        "chapter": "keyword",
        "section": "keyword",
        "page": "integer",
        "type": "keyword",
    }
    
    success = create_payload_indexes(client, args.collection, indexes)
    
    if not success:
        return 1
    
    # Validate collection
    success = validate_collection(client, args.collection, args.vector_size)
    
    if not success:
        return 1
    
    # Insert test vector
    if not args.skip_test:
        success = insert_test_vector(client, args.collection, args.vector_size)
        
        if not success:
            return 1
    
    print(f"\n🎉 Collection setup complete!")
    print(f"   Collection name: {args.collection}")
    print(f"   Ready for embeddings ingestion")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())