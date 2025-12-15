from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any, Union
from uuid import UUID, uuid4
from datetime import datetime


class ChatCompletionRequest(BaseModel):
    model: str
    messages: List[Dict[str, str]]
    temperature: Optional[float] = 0.7
    max_tokens: Optional[int] = 1024


class ChatCompletionResponse(BaseModel):
    id: str
    object: str = "chat.completion"
    created: int
    model: str
    choices: List[Dict[str, Any]]
    usage: Dict[str, int]


class EmbeddingRequest(BaseModel):
    model: str
    input: Union[str, List[str]]


class EmbeddingResponse(BaseModel):
    object: str = "list"
    data: List[Dict[str, Any]]
    model: str
    usage: Dict[str, int]


class TokenRequest(BaseModel):
    student_id: str
    quota_daily: Optional[int] = 100


class TokenResponse(BaseModel):
    token: str
    expires_at: datetime


class UsageResponse(BaseModel):
    student_id: str
    quota_daily: int
    quota_used_today: int
    quota_remaining: int
    last_request: Optional[datetime] = None
    total_requests: int = 0
    total_cost_usd: float = 0.0


class ErrorResponse(BaseModel):
    error: str
    message: str
    details: Optional[Dict[str, Any]] = None