"""
FastAPI Chatbot Service for Physical AI Course
RAG-based chatbot with strict grounding to course content
"""
from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
import os
from datetime import datetime
import asyncio
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables from backend/.env
dotenv_path = Path(__file__).parent.parent.parent / ".env"
load_dotenv(dotenv_path=dotenv_path)

# Import local modules
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from shared.models import ChatbotQuery
from chatbot.src.models.chunk import Chunk, ChunkMetadata

# Embedding and database clients
from sentence_transformers import SentenceTransformer
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue

app = FastAPI(
    title="Physical AI Chatbot API",
    description="RAG-based chatbot with grounding to course content",
    version="1.0.0"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Update this for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize clients
embedding_model = None
gemini_model = None
qdrant_client = None

class QueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=500)
    student_id: Optional[str] = None
    context_module: Optional[str] = None  # Filter by module if provided

class QueryResponse(BaseModel):
    response: str
    citations: List[Dict[str, str]]
    retrieved_chunks: int
    response_time_ms: int

@app.on_event("startup")
async def startup_event():
    """Initialize database connections on startup"""
    global embedding_model, gemini_model, qdrant_client

    # Initialize Sentence-Transformers model (local, no API cost)
    print("Loading sentence-transformers model...")
    embedding_model = SentenceTransformer('all-MiniLM-L6-v2')  # 384-dim
    print("[OK] Embedding model loaded")

    # Initialize Gemini API
    api_key = os.getenv("GEMINI_API_KEY")
    if api_key:
        genai.configure(api_key=api_key)
        gemini_model = genai.GenerativeModel('models/gemini-2.5-flash')
        print("[OK] Using Google Gemini 2.5 Flash for response generation")
    else:
        print("[WARN] No GEMINI_API_KEY found in environment variables")
        print("   Add GEMINI_API_KEY to backend/.env")

    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if qdrant_url == ":memory:" or (qdrant_url and not qdrant_url.startswith("http")):
        # Use local file-based storage
        qdrant_client = QdrantClient(path=qdrant_url if qdrant_url != ":memory:" else "./qdrant_storage")
        print(f"[OK] Using local Qdrant at {qdrant_url}")
    elif qdrant_api_key:
        qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        print(f"[OK] Connected to Qdrant Cloud")
    else:
        qdrant_client = QdrantClient(url=qdrant_url)
        print(f"[OK] Connected to Qdrant at {qdrant_url}")

    print("[OK] Chatbot service initialized")

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "chatbot",
        "timestamp": datetime.utcnow().isoformat()
    }

@app.post("/query", response_model=QueryResponse)
async def query_chatbot(request: QueryRequest):
    """
    Process a chatbot query with RAG retrieval and grounded response generation
    """
    start_time = datetime.utcnow()

    try:
        # 1. Generate embedding for the query (local, no API cost)
        query_embedding = embedding_model.encode(request.query, convert_to_tensor=False).tolist()

        # 2. Retrieve relevant chunks from Qdrant
        search_results = qdrant_client.query_points(
            collection_name="physical_ai_course",
            query=query_embedding,
            limit=5,  # Top 5 most relevant chunks
            with_payload=True
        ).points

        # 3. Format retrieved context
        context_parts = []
        citations = []

        for idx, result in enumerate(search_results):
            chunk_text = result.payload.get("text", "")
            module_id = result.payload.get("module_id", "")
            section_id = result.payload.get("section_id", "")
            heading = result.payload.get("heading", "")

            context_parts.append(f"[Source {idx+1}] {heading}\n{chunk_text}")
            citations.append({
                "source_id": f"Source {idx+1}",
                "module": module_id,
                "section": section_id,
                "heading": heading,
                "similarity": str(round(result.score, 3))
            })

        context = "\n\n".join(context_parts)

        # 4. Generate grounded response using Gemini
        system_prompt = """You are an expert AI teaching assistant for the Physical AI & Humanoid Robotics course.

CRITICAL RULES:
1. ONLY answer questions using information from the provided course content below
2. If the answer is NOT in the provided content, respond: "I don't have information on that topic in this course. Try rephrasing or checking the Table of Contents."
3. ALWAYS cite sources using [Source X] notation
4. Be concise and educational
5. Do not use external knowledge or make assumptions

Course Content:
{context}"""

        user_prompt = f"Student Question: {request.query}\n\nPlease provide a clear, educational answer based ONLY on the course content provided above."

        # Use Gemini API
        if gemini_model:
            prompt = f"{system_prompt.format(context=context)}\n\n{user_prompt}"
            response = await asyncio.to_thread(gemini_model.generate_content, prompt)
            response_text = response.text
        else:
            response_text = "Error: No Gemini API key configured. Please add GEMINI_API_KEY to backend/.env"

        # 5. Calculate response time
        end_time = datetime.utcnow()
        response_time_ms = int((end_time - start_time).total_seconds() * 1000)

        return QueryResponse(
            response=response_text,
            citations=citations,
            retrieved_chunks=len(search_results),
            response_time_ms=response_time_ms
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Query processing failed: {str(e)}")

@app.get("/stats")
async def get_stats():
    """Get chatbot statistics"""
    try:
        # Get collection info from Qdrant
        collection_info = qdrant_client.get_collection("physical_ai_course")

        return {
            "total_chunks": collection_info.points_count,
            "vector_size": collection_info.config.params.vectors.size,
            "status": "operational"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to fetch stats: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)
