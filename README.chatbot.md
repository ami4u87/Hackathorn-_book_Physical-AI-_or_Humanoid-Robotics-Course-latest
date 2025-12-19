# Chatbot Integration

This document explains the RAG-based chatbot implementation for the Physical AI course.

## Features

✅ **Grounded Responses**: Only answers from course content
✅ **Source Citations**: Shows which sections were used
✅ **Semantic Search**: Uses OpenAI embeddings + Qdrant vector DB
✅ **Modern UI**: Floating widget with smooth animations
✅ **Dark Mode**: Automatically adapts to Docusaurus theme
✅ **Mobile Responsive**: Works on all screen sizes

## Architecture

```
┌─────────────┐
│  User Query │
└──────┬──────┘
       │
       ▼
┌─────────────────────────────────┐
│  ChatbotWidget.tsx (Frontend)   │
│  - Captures user input          │
│  - Displays messages/citations  │
└──────┬──────────────────────────┘
       │ HTTP POST /query
       ▼
┌─────────────────────────────────┐
│  FastAPI Backend                │
│  - Generate query embedding     │
│  - Search Qdrant for chunks     │
│  - Generate grounded response   │
└──────┬──────────────────────────┘
       │
       ▼
┌─────────────────────────────────┐
│  Response with Citations        │
└─────────────────────────────────┘
```

## Files

### Frontend
- `src/components/ChatbotWidget.tsx` - Main chat UI component
- `src/components/ChatbotWidget.module.css` - Styles for the widget
- `src/theme/Root.tsx` - Docusaurus root wrapper (adds widget globally)

### Backend
- `backend/chatbot/src/main.py` - FastAPI chatbot service
- `backend/chatbot/src/models/` - Data models
- `backend/shared/` - Shared utilities

### Configuration
- `docusaurus.config.js` - Added `customFields.chatbotApiUrl`
- `.env.local.example` - Frontend environment variables
- `backend/.env.example` - Backend environment variables

## Local Development

### 1. Start Backend

```bash
# Navigate to backend
cd backend

# Create .env from example
cp .env.example .env

# Fill in your API keys in .env
# - OPENAI_API_KEY
# - QDRANT_URL
# - QDRANT_API_KEY (if using cloud)

# Initialize Qdrant
python scripts/init_qdrant.py

# Load course content
python scripts/load_content.py

# Start chatbot service
cd chatbot
pip install -r requirements.txt
uvicorn src.main:app --reload --port 8000
```

### 2. Start Frontend

```bash
# In project root
cp .env.local.example .env.local

# .env.local should have:
# CHATBOT_API_URL=http://localhost:8000

npm start
```

### 3. Test Chatbot

1. Visit http://localhost:3000
2. Click the 💬 button in bottom-right corner
3. Ask a question about the course
4. Verify you get a response with citations

## How It Works

### 1. Query Embedding
When a user asks a question, the backend generates an embedding using OpenAI's `text-embedding-3-large` model (1536 dimensions).

### 2. Vector Search
The query embedding is searched against the Qdrant collection (`book_chunks`) to find the top 5 most semantically similar course content chunks.

### 3. Context Assembly
The retrieved chunks are formatted into a context string with source tags:
```
[Source 1] Publisher-Subscriber Pattern
In ROS 2, publishers and subscribers...

[Source 2] Creating a Publisher Node
To create a publisher in Python...
```

### 4. Grounded Response Generation
The context is passed to GPT-4 with strict instructions:
- Only use information from the provided chunks
- Always cite sources using [Source X] notation
- If answer not in content, say "I don't have information on that topic"

### 5. Response with Citations
The frontend displays:
- The generated answer
- A list of cited sources with:
  - Module and section names
  - Similarity scores
  - Source tags

## Configuration

### Environment Variables

**Frontend (.env.local):**
```
CHATBOT_API_URL=http://localhost:8000  # or production URL
```

**Backend (backend/.env):**
```
OPENAI_API_KEY=sk-...
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=...  # optional for local
DATABASE_URL=postgresql://...
PORT=8000
```

### Customization

**Change chatbot appearance:**
Edit `src/components/ChatbotWidget.module.css`

**Change system prompt:**
Edit `backend/chatbot/src/main.py` line 130-140

**Change number of retrieved chunks:**
Edit `backend/chatbot/src/main.py` line 104 (`limit=5`)

**Change embedding model:**
Edit `backend/chatbot/src/main.py` line 95 (`model="text-embedding-3-large"`)

**Change GPT model:**
Edit `backend/chatbot/src/main.py` line 146 (`model="gpt-4-turbo-preview"`)

## API Endpoints

### POST /query
Query the chatbot

**Request:**
```json
{
  "query": "What is ROS 2?",
  "student_id": "optional-id",
  "context_module": "optional-module-filter"
}
```

**Response:**
```json
{
  "response": "ROS 2 is a framework for...",
  "citations": [
    {
      "source_id": "Source 1",
      "module": "module-1-ros2",
      "section": "introduction",
      "heading": "What is ROS 2?",
      "similarity": 0.89
    }
  ],
  "retrieved_chunks": 5,
  "response_time_ms": 1250
}
```

### GET /health
Health check

**Response:**
```json
{
  "status": "healthy",
  "service": "chatbot",
  "timestamp": "2024-01-15T12:00:00"
}
```

### GET /stats
Get collection statistics

**Response:**
```json
{
  "total_chunks": 1523,
  "vector_size": 1536,
  "status": "operational"
}
```

## Deployment

See [DEPLOYMENT.md](./DEPLOYMENT.md) for full deployment instructions.

Quick summary:
1. Deploy backend to Railway/Render/Fly.io
2. Set environment variables on backend
3. Update `CHATBOT_API_URL` in Vercel
4. Update CORS in `backend/chatbot/src/main.py`

## Troubleshooting

### Chatbot button not appearing
- Check browser console for errors
- Verify `src/theme/Root.tsx` is being used
- Clear Docusaurus cache: `npm run clear`

### No response from chatbot
- Check backend is running: `curl http://localhost:8000/health`
- Check browser Network tab for failed requests
- Verify CORS is configured correctly

### "I don't have information" responses
- Verify Qdrant has data: `curl http://localhost:8000/stats`
- Check if question is too specific or off-topic
- Try rephrasing the question

### Slow responses
- Check your OpenAI API rate limits
- Monitor backend response time in citations
- Consider caching common queries

## Future Enhancements

- [ ] Add feedback collection (thumbs up/down)
- [ ] Implement rate limiting per student
- [ ] Add conversation history (multi-turn)
- [ ] Add suggested questions
- [ ] Add "Ask about this section" button on each page
- [ ] Add analytics dashboard for instructors
- [ ] Support file upload (PDFs, code files)
- [ ] Add voice input/output
