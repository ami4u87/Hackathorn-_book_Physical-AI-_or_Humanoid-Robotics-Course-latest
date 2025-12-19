#!/usr/bin/env python3
"""
Course Content Loader for Qdrant Vector Database

Reads Docusaurus markdown files, chunks them intelligently, generates embeddings,
and loads them into Qdrant for RAG-based chatbot retrieval.

Features:
- Parses markdown with frontmatter
- Intelligent chunking by headings and sections
- Token counting with tiktoken
- OpenAI text-embedding-3-large embeddings (1536 dimensions)
- Progress tracking and statistics

Usage:
    python scripts/load_content.py
    python scripts/load_content.py --docs-path ./docs --dry-run
    python scripts/load_content.py --chunk-size 512 --overlap 50

Environment Variables:
    OPENAI_API_KEY: OpenAI API key (required)
    QDRANT_URL: Qdrant instance URL
    QDRANT_API_KEY: Qdrant API key (cloud only)
"""

import os
import sys
import re
import time
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass
from dotenv import load_dotenv

# Load environment variables from backend/.env
from pathlib import Path
dotenv_path = Path(__file__).parent.parent / "backend" / ".env"
load_dotenv(dotenv_path=dotenv_path)

try:
    import tiktoken
    from sentence_transformers import SentenceTransformer
    from qdrant_client import QdrantClient
    from qdrant_client.models import PointStruct, Distance, VectorParams
except ImportError as e:
    print(f"[ERROR] Missing required package: {e}")
    print("\nPlease install required packages:")
    print("  pip install --user sentence-transformers tiktoken qdrant-client python-dotenv")
    sys.exit(1)


@dataclass
class ContentChunk:
    """Represents a chunk of course content."""
    text: str
    module: str
    section: str
    heading: str
    tokens: int
    chunk_index: int
    source_id: str
    file_path: str
    metadata: Dict[str, Any]


class MarkdownParser:
    """Parse Docusaurus markdown files with frontmatter."""

    FRONTMATTER_REGEX = re.compile(r'^---\s*\n(.*?)\n---\s*\n', re.DOTALL)
    HEADING_REGEX = re.compile(r'^(#{1,6})\s+(.+)$', re.MULTILINE)

    @staticmethod
    def parse_file(file_path: Path) -> Tuple[Dict[str, Any], str]:
        """Parse markdown file and extract frontmatter and content."""
        try:
            content = file_path.read_text(encoding='utf-8')
        except Exception as e:
            print(f"[WARN] Could not read {file_path}: {e}")
            return {}, ""

        # Extract frontmatter
        frontmatter = {}
        match = MarkdownParser.FRONTMATTER_REGEX.match(content)
        if match:
            fm_text = match.group(1)
            content = content[match.end():]

            # Simple YAML parsing (key: value pairs)
            for line in fm_text.split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    frontmatter[key.strip()] = value.strip().strip('"').strip("'")

        return frontmatter, content.strip()

    @staticmethod
    def extract_sections(content: str) -> List[Tuple[str, str, int]]:
        """
        Split content into sections by headings.
        Returns: List of (heading, content, level) tuples
        """
        sections = []
        matches = list(MarkdownParser.HEADING_REGEX.finditer(content))

        if not matches:
            # No headings, treat entire content as one section
            return [("Introduction", content, 1)]

        for i, match in enumerate(matches):
            heading_level = len(match.group(1))  # Count '#' characters
            heading_text = match.group(2).strip()
            start = match.end()
            end = matches[i + 1].start() if i + 1 < len(matches) else len(content)
            section_content = content[start:end].strip()

            if section_content:  # Only add non-empty sections
                sections.append((heading_text, section_content, heading_level))

        return sections


class ContentChunker:
    """Chunk content intelligently for embedding."""

    def __init__(self, max_tokens: int = 512, overlap_tokens: int = 50):
        self.max_tokens = max_tokens
        self.overlap_tokens = overlap_tokens
        self.encoding = tiktoken.encoding_for_model("gpt-4")

    def count_tokens(self, text: str) -> int:
        """Count tokens in text."""
        return len(self.encoding.encode(text))

    def chunk_section(
        self,
        heading: str,
        content: str,
        module: str,
        section: str,
        file_path: Path,
        chunk_start_index: int
    ) -> List[ContentChunk]:
        """
        Chunk a section into embeddings.
        If section is small, keep as one chunk. Otherwise split intelligently.
        """
        full_text = f"# {heading}\n\n{content}"
        tokens = self.count_tokens(full_text)

        chunks = []

        if tokens <= self.max_tokens:
            # Section fits in one chunk
            chunks.append(ContentChunk(
                text=full_text,
                module=module,
                section=section,
                heading=heading,
                tokens=tokens,
                chunk_index=chunk_start_index,
                source_id=f"Source {chunk_start_index + 1}",
                file_path=str(file_path),
                metadata={"heading_level": 1}
            ))
        else:
            # Split by paragraphs
            paragraphs = content.split('\n\n')
            current_chunk = f"# {heading}\n\n"
            current_tokens = self.count_tokens(current_chunk)

            for para in paragraphs:
                para_tokens = self.count_tokens(para)

                if current_tokens + para_tokens <= self.max_tokens:
                    current_chunk += para + "\n\n"
                    current_tokens += para_tokens
                else:
                    # Save current chunk
                    if len(current_chunk.strip()) > len(f"# {heading}"):
                        chunks.append(ContentChunk(
                            text=current_chunk.strip(),
                            module=module,
                            section=section,
                            heading=heading,
                            tokens=current_tokens,
                            chunk_index=chunk_start_index + len(chunks),
                            source_id=f"Source {chunk_start_index + len(chunks) + 1}",
                            file_path=str(file_path),
                            metadata={"heading_level": 1, "chunk_part": len(chunks) + 1}
                        ))

                    # Start new chunk with overlap (include heading)
                    current_chunk = f"# {heading}\n\n{para}\n\n"
                    current_tokens = self.count_tokens(current_chunk)

            # Save final chunk
            if len(current_chunk.strip()) > len(f"# {heading}"):
                chunks.append(ContentChunk(
                    text=current_chunk.strip(),
                    module=module,
                    section=section,
                    heading=heading,
                    tokens=current_tokens,
                    chunk_index=chunk_start_index + len(chunks),
                    source_id=f"Source {chunk_start_index + len(chunks) + 1}",
                    file_path=str(file_path),
                    metadata={"heading_level": 1, "chunk_part": len(chunks) + 1}
                ))

        return chunks


class ContentLoader:
    """Load course content into Qdrant vector database."""

    def __init__(
        self,
        docs_path: str = "docs",
        collection_name: str = "physical_ai_course",
        chunk_size: int = 512,
        overlap: int = 50
    ):
        self.docs_path = Path(docs_path)
        self.collection_name = collection_name
        self.chunker = ContentChunker(max_tokens=chunk_size, overlap_tokens=overlap)

        # Initialize Sentence-Transformers model (free, runs locally)
        print("[INFO] Loading sentence-transformers model (this may take a moment on first run)...")
        self.embedding_model = SentenceTransformer('all-MiniLM-L6-v2')  # 384-dim, fast and efficient
        print(f"[OK] Model loaded: all-MiniLM-L6-v2 (384 dimensions)")

        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_url == ":memory:" or (qdrant_url and not qdrant_url.startswith("http")):
            # Use local file-based storage
            self.qdrant_client = QdrantClient(path=qdrant_url if qdrant_url != ":memory:" else "./qdrant_storage")
            print(f"[OK] Connected to local Qdrant at {qdrant_url}")
        elif qdrant_api_key:
            self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
            print(f"[OK] Connected to Qdrant at: {qdrant_url}")
        else:
            self.qdrant_client = QdrantClient(url=qdrant_url)
            print(f"[OK] Connected to Qdrant at: {qdrant_url}")

    def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding using Sentence-Transformers (local, no API needed)."""
        try:
            embedding = self.embedding_model.encode(text, convert_to_tensor=False)
            return embedding.tolist()
        except Exception as e:
            print(f"[ERROR] Failed to generate embedding: {e}")
            raise

    def process_file(self, file_path: Path, chunk_counter: int) -> Tuple[List[ContentChunk], int]:
        """Process a markdown file and return chunks."""
        # Determine module and section from path
        relative_path = file_path.relative_to(self.docs_path)
        parts = relative_path.parts

        if len(parts) == 1:
            # Root file like intro.md
            module = "introduction"
            section = file_path.stem
        else:
            module = parts[0]  # e.g., "module-1-ros2"
            section = f"{module}/{file_path.stem}"

        # Parse file
        frontmatter, content = MarkdownParser.parse_file(file_path)

        if not content:
            return [], chunk_counter

        # Extract sections
        sections = MarkdownParser.extract_sections(content)

        # Chunk each section
        all_chunks = []
        for heading, section_content, level in sections:
            chunks = self.chunker.chunk_section(
                heading=heading,
                content=section_content,
                module=module,
                section=section,
                file_path=file_path,
                chunk_start_index=chunk_counter
            )
            all_chunks.extend(chunks)
            chunk_counter += len(chunks)

        return all_chunks, chunk_counter

    def load_content(self, dry_run: bool = False) -> None:
        """Load all course content into Qdrant."""
        print("\n" + "="*60)
        print("Course Content Loader - Starting")
        print("="*60)
        print(f"Docs path: {self.docs_path.absolute()}")
        print(f"Collection: {self.collection_name}")
        print(f"Chunk size: {self.chunker.max_tokens} tokens")
        print(f"Dry run: {dry_run}")
        print("="*60 + "\n")

        # Find all markdown files
        md_files = sorted(self.docs_path.rglob("*.md"))
        print(f"Found {len(md_files)} markdown files\n")

        if not md_files:
            print("[ERROR] No markdown files found in docs directory")
            sys.exit(1)

        # Process all files
        all_chunks = []
        chunk_counter = 0

        for file_path in md_files:
            print(f"Processing: {file_path.relative_to(self.docs_path)}")
            chunks, chunk_counter = self.process_file(file_path, chunk_counter)
            all_chunks.extend(chunks)
            print(f"  -> {len(chunks)} chunks ({sum(c.tokens for c in chunks)} tokens)")

        print(f"\n[OK] Processed {len(md_files)} files")
        print(f"[OK] Generated {len(all_chunks)} chunks")
        print(f"[OK] Total tokens: {sum(c.tokens for c in all_chunks)}")

        if dry_run:
            print("\n[INFO] Dry run mode - skipping upload to Qdrant")
            print("\nSample chunks:")
            for i, chunk in enumerate(all_chunks[:3]):
                print(f"\n--- Chunk {i+1} ---")
                print(f"Module: {chunk.module}")
                print(f"Section: {chunk.section}")
                print(f"Heading: {chunk.heading}")
                print(f"Tokens: {chunk.tokens}")
                print(f"Text preview: {chunk.text[:200]}...")
            return

        # Generate embeddings and upload
        print("\n" + "="*60)
        print("Generating Embeddings and Uploading to Qdrant")
        print("="*60 + "\n")

        points = []
        for i, chunk in enumerate(all_chunks):
            print(f"[{i+1}/{len(all_chunks)}] {chunk.source_id}: {chunk.heading[:50]}...", end="")

            try:
                # Generate embedding
                embedding = self.generate_embedding(chunk.text)

                # Create point
                point = PointStruct(
                    id=i + 1,
                    vector=embedding,
                    payload={
                        "source_id": chunk.source_id,
                        "module": chunk.module,
                        "section": chunk.section,
                        "heading": chunk.heading,
                        "text": chunk.text,
                        "tokens": chunk.tokens,
                        "chunk_index": chunk.chunk_index,
                        "file_path": chunk.file_path,
                        "metadata": chunk.metadata
                    }
                )
                points.append(point)
                print(" [OK]")

                # Upload in batches of 100
                if len(points) >= 100:
                    self.qdrant_client.upsert(
                        collection_name=self.collection_name,
                        points=points
                    )
                    points = []

                # Rate limiting
                time.sleep(0.1)

            except Exception as e:
                print(f" [ERROR]: {e}")
                continue

        # Upload remaining points
        if points:
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

        # Get final count
        collection_info = self.qdrant_client.get_collection(self.collection_name)

        print("\n" + "="*60)
        print("Content Loading Complete!")
        print("="*60)
        print(f"Collection: {self.collection_name}")
        print(f"Total points: {collection_info.points_count}")
        print(f"Vector size: {collection_info.config.params.vectors.size}")
        print("="*60 + "\n")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Load course content into Qdrant vector database"
    )
    parser.add_argument(
        "--docs-path",
        default="docs",
        help="Path to docs directory (default: docs)"
    )
    parser.add_argument(
        "--collection",
        default="physical_ai_course",
        help="Qdrant collection name (default: physical_ai_course)"
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=512,
        help="Maximum tokens per chunk (default: 512)"
    )
    parser.add_argument(
        "--overlap",
        type=int,
        default=50,
        help="Overlap tokens between chunks (default: 50)"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Process files without uploading to Qdrant"
    )

    args = parser.parse_args()

    try:
        loader = ContentLoader(
            docs_path=args.docs_path,
            collection_name=args.collection,
            chunk_size=args.chunk_size,
            overlap=args.overlap
        )
        loader.load_content(dry_run=args.dry_run)
    except KeyboardInterrupt:
        print("\n\n[WARN] Interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Content loading failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
