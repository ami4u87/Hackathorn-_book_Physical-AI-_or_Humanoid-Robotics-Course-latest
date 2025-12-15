from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from uuid import UUID, uuid4
from datetime import datetime


class QueryRequest(BaseModel):
    query_text: str = Field(..., min_length=1, max_length=500)
    student_id: Optional[str] = None
    module_filter: Optional[str] = None


class Citation(BaseModel):
    module: str
    section: str
    heading: str
    link: Optional[str] = None


class ChunkInfo(BaseModel):
    chunk_id: str
    similarity_score: float
    text_preview: str


class QueryResponse(BaseModel):
    query_id: str = Field(default_factory=lambda: str(uuid4()))
    response_text: str
    citations: List[Citation]
    retrieved_chunks: Optional[List[ChunkInfo]] = None
    response_time_ms: int


class FeedbackRequest(BaseModel):
    query_id: str
    rating: int = Field(..., ge=1, le=5)
    comment: Optional[str] = Field(None, max_length=500)


class ErrorResponse(BaseModel):
    error: str
    message: str
    details: Optional[Dict[str, Any]] = None