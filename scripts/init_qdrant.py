#!/usr/bin/env python3
"""
Qdrant Collection Initialization Script

Creates the physical_ai_course collection with proper vector configuration for RAG.

Vector Configuration:
- Embedding Model: OpenAI text-embedding-3-large
- Vector Size: 1536 dimensions
- Distance Metric: Cosine similarity

Usage:
    python scripts/init_qdrant.py
    python scripts/init_qdrant.py --url http://localhost:6333
    python scripts/init_qdrant.py --collection custom_name

Environment Variables:
    QDRANT_URL: Qdrant instance URL (default: http://localhost:6333)
    QDRANT_API_KEY: Qdrant API key (optional for local, required for cloud)
"""
import os
import sys
from typing import Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


def init_qdrant_collection(
    url: Optional[str] = None,
    api_key: Optional[str] = None,
    collection_name: str = "physical_ai_course"
) -> QdrantClient:
    """
    Initialize Qdrant collection for course content chunks.

    Args:
        url: Qdrant instance URL
        api_key: Qdrant API key (cloud only)
        collection_name: Name of the collection to create

    Returns:
        Initialized QdrantClient instance
    """
    # Use environment variables or provided parameters
    qdrant_url = url or os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key = api_key or os.getenv("QDRANT_API_KEY")

    print(f"Connecting to Qdrant at: {qdrant_url}")

    # Initialize client with error handling
    try:
        if qdrant_api_key:
            client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        else:
            client = QdrantClient(url=qdrant_url)
        print("✅ Successfully connected to Qdrant")
    except Exception as e:
        print(f"❌ Failed to connect to Qdrant: {e}")
        print("\nTroubleshooting:")
        print("1. Check QDRANT_URL in .env file")
        print("2. Verify Qdrant is running:")
        print("   - Local: docker ps | grep qdrant")
        print("   - Cloud: check cluster status at https://cloud.qdrant.io")
        print("3. For cloud: verify QDRANT_API_KEY is correct")
        sys.exit(1)

    # Check if collection already exists
    try:
        existing_collections = client.get_collections()
        collection_exists = any(
            col.name == collection_name for col in existing_collections.collections
        )
    except Exception as e:
        print(f"❌ Error checking collections: {e}")
        sys.exit(1)

    # Create or recreate collection
    try:
        if collection_exists:
            print(f"⚠️  Collection '{collection_name}' already exists")
            print("   Recreating with fresh configuration...")
        else:
            print(f"Creating new collection '{collection_name}'...")

        client.recreate_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
        )
        print(f"✅ Collection '{collection_name}' created successfully")
    except Exception as e:
        print(f"❌ Failed to create collection: {e}")
        sys.exit(1)

    # Verify collection was created
    try:
        collection_info = client.get_collection(collection_name=collection_name)
        print("\n" + "="*60)
        print("Collection Configuration:")
        print("="*60)
        print(f"  Name: {collection_name}")
        print(f"  Vector Size: {collection_info.config.params.vectors.size}")
        print(f"  Distance Metric: {collection_info.config.params.vectors.distance}")
        print(f"  Points Count: {collection_info.points_count}")
        print("="*60)
    except Exception as e:
        print(f"⚠️  Warning: Could not verify collection: {e}")

    # Document expected payload schema
    payload_schema = {
        "source_id": "Unique citation identifier (e.g., 'Source 1', 'Source 2')",
        "module": "Module ID (e.g., 'module-1-ros2')",
        "section": "Section ID (e.g., 'module-1-ros2/installation')",
        "heading": "Section heading or title",
        "text": "Chunk content (markdown text)",
        "tokens": "Token count for chunk",
        "chunk_index": "Sequential index within section",
        "code_language": "Programming language if code block (optional)",
        "metadata": "Additional context (JSON object)"
    }

    print("\nExpected Payload Schema:")
    for field, description in payload_schema.items():
        print(f"  • {field}: {description}")

    print("\n" + "="*60)
    print("Next Steps:")
    print("="*60)
    print("1. Load course content: python scripts/load_content.py")
    print("2. Verify vector count: curl http://localhost:6333/collections/physical_ai_course")
    print("3. Start backend: cd backend && docker-compose up -d")
    print()

    return client


def main() -> None:
    """Main entry point for CLI usage."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Initialize Qdrant collection for Physical AI course content"
    )
    parser.add_argument(
        "--url",
        default=None,
        help="Qdrant instance URL (default: from QDRANT_URL env var)"
    )
    parser.add_argument(
        "--api-key",
        default=None,
        help="Qdrant API key (default: from QDRANT_API_KEY env var)"
    )
    parser.add_argument(
        "--collection",
        default="physical_ai_course",
        help="Collection name (default: physical_ai_course)"
    )

    args = parser.parse_args()

    print("\n" + "="*60)
    print("Physical AI Chatbot - Qdrant Collection Initialization")
    print("="*60 + "\n")

    # Run the initialization
    try:
        init_qdrant_collection(
            url=args.url,
            api_key=args.api_key,
            collection_name=args.collection
        )
    except Exception as e:
        print(f"\n❌ Failed to initialize Qdrant collection: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()