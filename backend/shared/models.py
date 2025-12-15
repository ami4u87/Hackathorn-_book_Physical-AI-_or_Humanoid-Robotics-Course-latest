from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from uuid import UUID, uuid4
from datetime import datetime


class Student(BaseModel):
    student_id: str
    jwt_secret: Optional[str] = None
    quota_daily: int = 100
    quota_used_today: int = 0
    last_reset: Optional[datetime] = None
    created_at: Optional[datetime] = None
    last_active: Optional[datetime] = None


class ChatbotQuery(BaseModel):
    query_id: str = Field(default_factory=lambda: str(uuid4()))
    student_id: Optional[str] = None
    query_text: str
    query_embedding: Optional[List[float]] = None  # 1536-dimensional vector
    retrieved_chunks: List[Dict[str, Any]]  # Array of chunk IDs and similarity scores
    response_text: str
    citations: List[Dict[str, str]]  # Array of module/section references
    feedback_rating: Optional[int] = Field(None, ge=1, le=5)
    feedback_text: Optional[str] = None
    response_time_ms: int
    timestamp: Optional[datetime] = None


class APIUsageLog(BaseModel):
    log_id: Optional[int] = None
    student_id: str
    endpoint: str
    service: str  # 'chatbot' or 'vla'
    request_timestamp: Optional[datetime] = None
    response_timestamp: Optional[datetime] = None
    request_tokens: int = 0
    response_tokens: int = 0
    cost_usd: float = 0.0
    quota_remaining: int = 0
    status_code: int = 200


class ExerciseSubmission(BaseModel):
    submission_id: str = Field(default_factory=lambda: str(uuid4()))
    student_id: str
    exercise_id: str
    validation_hash: str  # 64-character hex string
    timestamp: Optional[datetime] = None
    is_correct: Optional[bool] = None
    attempt_number: int = 1