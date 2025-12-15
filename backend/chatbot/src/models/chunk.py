from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List
from uuid import UUID, uuid4


class ChunkMetadata(BaseModel):
    heading: str
    page_range: Optional[str] = None
    code_language: Optional[str] = None
    has_diagram: bool = False
    diagram_ref: Optional[str] = None


class Chunk(BaseModel):
    chunk_id: str = Field(default_factory=lambda: str(uuid4()))
    module_id: str
    section_id: Optional[str] = None
    text: str
    embedding: Optional[List[float]] = None  # Will be 1536-dimensional vector
    tokens: int = Field(..., le=750)
    chunk_index: int = 0
    metadata: ChunkMetadata