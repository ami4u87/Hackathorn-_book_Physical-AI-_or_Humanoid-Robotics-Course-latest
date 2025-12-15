# Physical AI Chatbot Backend

RAG-based chatbot API with strict grounding to course content using OpenAI GPT-4, Qdrant vector database, and FastAPI.

## Architecture

```
┌─────────────────┐
│  Docusaurus     │
│  Frontend       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  API Proxy      │ (Optional: Auth, Rate Limiting)
│  Port: 8001     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────┐
│  Chatbot API    │─────▶│  OpenAI API  │
│  Port: 8000     │      │  GPT-4       │
└────────┬────────┘      └──────────────┘
         │
         ├─────────────────┐
         ▼                 ▼
┌──────────────┐    ┌─────────────┐
│  Qdrant      │    │  Neon       │
│  Vectors     │    │  Postgres   │
│  Port: 6333  │    │             │
└──────────────┘    └─────────────┘
```

## Prerequisites

- Python 3.11+
- Docker and Docker Compose (for local development)
- OpenAI API key
- Qdrant instance (local Docker or cloud)
- Neon Postgres database (or local PostgreSQL)

## Quick Start

### 1. Environment Setup

Copy the environment template:
```bash
cd backend
cp .env.example .env
```

Edit `.env` and add your API keys:
```env
OPENAI_API_KEY=sk-your-key-here
QDRANT_URL=http://localhost:6333
DATABASE_URL=postgresql://user:pass@localhost:5432/dbname
```

### 2. Local Development with Docker

Start all services:
```bash
docker-compose up -d
```

This starts:
- Qdrant vector database (port 6333)
- Chatbot API (port 8000)
- API Proxy (port 8001)

### 3. Initialize Qdrant Collection

```bash
python scripts/init_qdrant.py
```

### 4. Load Course Content

```bash
# TODO: Add content loading script
# python scripts/load_content.py --docs-path ../docs
```

### 5. Test the API

```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "student_id": "test-123"}'
```

Expected response:
```json
{
  "response": "ROS 2 (Robot Operating System 2) is...",
  "citations": [
    {
      "source_id": "Source 1",
      "module": "module-1-ros2",
      "section": "module-1-ros2/index",
      "heading": "Introduction to ROS 2",
      "similarity": 0.923
    }
  ],
  "retrieved_chunks": 5,
  "response_time_ms": 1240
}
```

## API Endpoints

### Chatbot Service (Port 8000)

#### `POST /query`
Process a chatbot query with RAG retrieval.

**Request:**
```json
{
  "query": "How do I create a ROS 2 publisher?",
  "student_id": "student-123",
  "context_module": "module-1-ros2"
}
```

**Response:**
```json
{
  "response": "To create a ROS 2 publisher, you need to...",
  "citations": [...],
  "retrieved_chunks": 5,
  "response_time_ms": 1200
}
```

#### `GET /health`
Health check endpoint.

#### `GET /stats`
Get chatbot statistics (total chunks, vector size, etc.).

## Production Deployment

### Vercel Serverless Functions

1. Create `vercel.json` in backend directory:
```json
{
  "functions": {
    "chatbot/src/main.py": {
      "runtime": "python3.11"
    }
  },
  "env": {
    "OPENAI_API_KEY": "@openai-api-key",
    "QDRANT_URL": "@qdrant-url",
    "QDRANT_API_KEY": "@qdrant-api-key"
  }
}
```

2. Deploy:
```bash
vercel
```

### Alternative: Railway/Render

Both support Python/FastAPI deployments:
- Railway: Connect GitHub repo, auto-deploy
- Render: Create Web Service, connect repo

## Database Setup

### Qdrant Cloud

1. Sign up at https://cloud.qdrant.io
2. Create a cluster
3. Get API key and cluster URL
4. Update `.env`:
```env
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key
```

### Neon Postgres

1. Sign up at https://neon.tech
2. Create a project
3. Copy connection string
4. Update `.env`:
```env
DATABASE_URL=postgresql://user:pass@ep-xxx.region.aws.neon.tech/dbname?sslmode=require
```

## Development

### Install Dependencies

```bash
cd backend/chatbot
pip install -r requirements.txt
```

### Run Locally

```bash
cd backend/chatbot
uvicorn src.main:app --reload --port 8000
```

### Testing

```bash
pytest
```

## Configuration

### Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `OPENAI_API_KEY` | OpenAI API key | Required |
| `QDRANT_URL` | Qdrant instance URL | `http://localhost:6333` |
| `QDRANT_API_KEY` | Qdrant API key (cloud only) | Optional |
| `DATABASE_URL` | PostgreSQL connection string | Required |
| `CHATBOT_PORT` | Chatbot service port | `8000` |
| `ALLOWED_ORIGINS` | CORS allowed origins | `*` |
| `LOG_LEVEL` | Logging level | `INFO` |

## Troubleshooting

### Connection Refused to Qdrant

- Ensure Qdrant is running: `docker ps | grep qdrant`
- Check URL in `.env` matches container: `http://qdrant:6333` (Docker) or `http://localhost:6333` (local)

### OpenAI API Errors

- Verify API key is valid
- Check quota/billing: https://platform.openai.com/usage
- Rate limits: Implement exponential backoff

### Slow Response Times

- Reduce number of retrieved chunks (default: 5)
- Use faster embedding model: `text-embedding-3-small`
- Cache frequent queries

## Monitoring

View logs:
```bash
docker-compose logs -f chatbot
```

Monitor Qdrant:
- Dashboard: http://localhost:6333/dashboard
- Metrics: http://localhost:6333/metrics

## Security

- **Never commit** `.env` files
- Use secrets management (Vercel Env Vars, Railway Secrets)
- Enable CORS only for trusted origins
- Implement rate limiting in production
- Use JWT authentication for student-specific features

## Next Steps

1. ✅ Set up environment variables
2. ✅ Start Docker services
3. ✅ Initialize Qdrant collection
4. ⏳ Load course content into vector database
5. ⏳ Test API endpoints
6. ⏳ Deploy to production
7. ⏳ Integrate with Docusaurus frontend

## Support

For issues, see:
- [Main Documentation](../README.md)
- [API Contracts](../specs/master/contracts/)
- [Architecture Docs](../ARCHITECTURE.md)
