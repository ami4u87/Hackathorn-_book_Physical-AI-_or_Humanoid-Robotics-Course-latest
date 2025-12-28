"""
Mock Chatbot Server for Testing
No API keys needed - demonstrates the UI without OpenAI
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Dict
from datetime import datetime

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class QueryRequest(BaseModel):
    query: str

@app.get("/health")
async def health():
    return {
        "status": "healthy",
        "service": "mock-chatbot",
        "timestamp": datetime.utcnow().isoformat()
    }

@app.get("/stats")
async def stats():
    return {
        "total_chunks": 173,
        "vector_size": 384,
        "status": "operational (mock mode)"
    }

@app.post("/query")
async def query(request: QueryRequest):
    # Simulate a response based on the query
    query_lower = request.query.lower()

    if "ros" in query_lower or "publisher" in query_lower or "subscriber" in query_lower:
        response = """In ROS 2, the publisher-subscriber pattern is a fundamental communication mechanism. [Source 1]

A **publisher** sends messages on a topic, while a **subscriber** receives them. Here's a simple example:

```python
# Publisher
publisher = node.create_publisher(String, 'chatter', 10)
publisher.publish(String(data='Hello'))

# Subscriber
subscription = node.create_subscription(String, 'chatter', callback, 10)
```

The pattern is asynchronous and decoupled, allowing multiple publishers and subscribers on the same topic. [Source 2]"""

        citations = [
            {
                "source_id": "Source 1",
                "module": "module-1-ros2",
                "section": "2-pubsub",
                "heading": "Publisher-Subscriber Pattern",
                "similarity": 0.92
            },
            {
                "source_id": "Source 2",
                "module": "module-1-ros2",
                "section": "2-pubsub",
                "heading": "Understanding Topics",
                "similarity": 0.87
            }
        ]
    elif "install" in query_lower:
        response = """To install ROS 2, follow these steps: [Source 1]

1. Set up your sources.list
2. Add the ROS 2 GPG key
3. Install ROS 2 packages: `sudo apt install ros-humble-desktop`
4. Source the environment: `source /opt/ros/humble/setup.bash`

For detailed instructions, see the installation guide in Module 1. [Source 2]"""

        citations = [
            {
                "source_id": "Source 1",
                "module": "module-1-ros2",
                "section": "1-installation",
                "heading": "Installation",
                "similarity": 0.95
            },
            {
                "source_id": "Source 2",
                "module": "module-1-ros2",
                "section": "1-installation",
                "heading": "Prerequisites",
                "similarity": 0.89
            }
        ]
    else:
        response = f"""This is a mock response to your question: "{request.query}"

In production, the chatbot would:
1. Generate an embedding of your query using sentence-transformers (FREE, local)
2. Search Qdrant for the 5 most relevant course content chunks
3. Generate a grounded response using GPT-4 based ONLY on those chunks
4. Provide citations showing which course sections were used

The current setup uses:
- ✅ Local embeddings (all-MiniLM-L6-v2) - FREE
- ✅ Qdrant vector search - Already configured
- ⚠️ GPT-4 response generation - Needs valid API key or can use Ollama (local LLM)

173 chunks loaded from your course content! [Source 1]"""

        citations = [
            {
                "source_id": "Source 1",
                "module": "introduction",
                "section": "intro",
                "heading": "Physical AI & Humanoid Robotics",
                "similarity": 0.75
            }
        ]

    return {
        "response": response,
        "citations": citations,
        "retrieved_chunks": len(citations),
        "response_time_ms": 150
    }

if __name__ == "__main__":
    import uvicorn
    print("\n" + "="*60)
    print("Mock Chatbot Server Starting")
    print("="*60)
    print("Testing the chatbot UI without OpenAI API")
    print("Server: http://localhost:8000")
    print("="*60 + "\n")
    uvicorn.run(app, host="0.0.0.0", port=8000)
