#!/usr/bin/env python3
"""
Qdrant collection initialization script
Creates the book_chunks collection with proper schema for RAG system
"""
import asyncio
import os
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from qdrant_client.http import models
from typing import Optional


async def init_qdrant_collection(
    url: Optional[str] = None,
    api_key: Optional[str] = None,
    collection_name: str = "book_chunks"
):
    """
    Initialize Qdrant collection for book content chunks
    """
    # Use environment variables or provided parameters
    qdrant_url = url or os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key = api_key or os.getenv("QDRANT_API_KEY")

    print(f"Connecting to Qdrant at: {qdrant_url}")

    # Initialize client
    if qdrant_api_key:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    else:
        client = QdrantClient(url=qdrant_url)

    # Check if collection already exists
    try:
        existing_collections = client.get_collections()
        collection_exists = any(col.name == collection_name for col in existing_collections.collections)
    except Exception as e:
        print(f"Error checking collections: {e}")
        collection_exists = False

    if collection_exists:
        print(f"Collection '{collection_name}' already exists, recreating...")
        client.recreate_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
        )
    else:
        print(f"Creating new collection '{collection_name}'...")
        client.recreate_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
        )

    # Define the payload schema (metadata structure)
    # Note: Qdrant doesn't enforce strict schema but this documents expected fields
    payload_schema = {
        "chunk_id": "keyword",  # UUID as string
        "module_id": "keyword",  # e.g., "module-1-ros2"
        "section_id": "keyword",  # e.g., "module-1-ros2/2-pubsub"
        "heading": "text",  # Section heading
        "text": "text",  # Chunk content
        "tokens": "integer",  # Token count
        "page_range": "keyword",  # Estimated page in PDF
        "code_language": "keyword",  # If chunk contains code
        "has_diagram": "bool",  # Whether chunk has diagram reference
        "diagram_ref": "keyword"  # Path to diagram image
    }

    print(f"Collection '{collection_name}' initialized successfully!")
    print("Expected payload schema:")
    for field, type_hint in payload_schema.items():
        print(f"  - {field}: {type_hint}")

    return client


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Initialize Qdrant collection for book chunks")
    parser.add_argument("--url", default=None, help="Qdrant URL (default: from QDRANT_URL env var or http://localhost:6333)")
    parser.add_argument("--api-key", default=None, help="Qdrant API key (default: from QDRANT_API_KEY env var)")
    parser.add_argument("--collection", default="book_chunks", help="Collection name (default: book_chunks)")

    args = parser.parse_args()

    # Run the async function
    asyncio.run(init_qdrant_collection(
        url=args.url,
        api_key=args.api_key,
        collection_name=args.collection
    ))


if __name__ == "__main__":
    main()