# Deployment Guide

This guide covers deploying the Physical AI Course platform with the RAG chatbot.

## Architecture Overview

- **Frontend**: Docusaurus site (Static) → Vercel
- **Backend**: FastAPI chatbot service → Railway/Render/Fly.io
- **Databases**:
  - Qdrant (Vector DB) → Qdrant Cloud
  - PostgreSQL → Neon (already configured)

## Prerequisites

- [ ] OpenAI API key
- [ ] Qdrant Cloud account (or use local Qdrant in Docker)
- [ ] Neon PostgreSQL database (already configured in backend/.env.example)
- [ ] Backend hosting account (Railway, Render, or Fly.io)
- [ ] Vercel account (already set up)

## Step 1: Set Up Qdrant Cloud

### Option A: Qdrant Cloud (Recommended for Production)

1. Sign up at https://cloud.qdrant.io
2. Create a new cluster
3. Copy your cluster URL and API key
4. Update `backend/.env`:
   ```
   QDRANT_URL=https://your-cluster-id.us-east4-0.gcp.cloud.qdrant.io
   QDRANT_API_KEY=your-api-key-here
   ```

### Option B: Self-Hosted Qdrant (Docker)

For development or if you want to self-host:

```bash
cd backend
docker-compose up -d qdrant
```

## Step 2: Initialize and Load Data

1. **Create backend/.env** from backend/.env.example:
   ```bash
   cd backend
   cp .env.example .env
   ```

2. **Fill in your credentials** in backend/.env:
   - `OPENAI_API_KEY`: Your OpenAI API key
   - `QDRANT_URL`: Your Qdrant instance URL
   - `QDRANT_API_KEY`: Your Qdrant API key (if using cloud)
   - `DATABASE_URL`: Already configured for Neon

3. **Initialize Qdrant collection**:
   ```bash
   python scripts/init_qdrant.py
   ```

4. **Load course content** into Qdrant:
   ```bash
   python scripts/load_content.py
   ```

## Step 3: Test Backend Locally

1. **Install Python dependencies**:
   ```bash
   cd backend/chatbot
   pip install -r requirements.txt
   ```

2. **Run the chatbot service**:
   ```bash
   cd backend/chatbot
   uvicorn src.main:app --reload --port 8000
   ```

3. **Test the API**:
   ```bash
   curl http://localhost:8000/health
   ```

   ```bash
   curl -X POST http://localhost:8000/query \
     -H "Content-Type: application/json" \
     -d '{"query": "What is ROS 2?"}'
   ```

## Step 4: Deploy Backend

### Option A: Railway (Recommended)

1. **Install Railway CLI**:
   ```bash
   npm install -g @railway/cli
   ```

2. **Login to Railway**:
   ```bash
   railway login
   ```

3. **Create new project**:
   ```bash
   cd backend/chatbot
   railway init
   ```

4. **Add environment variables**:
   ```bash
   railway variables set OPENAI_API_KEY=your-key
   railway variables set QDRANT_URL=your-qdrant-url
   railway variables set QDRANT_API_KEY=your-qdrant-key
   railway variables set DATABASE_URL=your-neon-url
   railway variables set PORT=8000
   ```

5. **Deploy**:
   ```bash
   railway up
   ```

6. **Get your deployment URL**:
   ```bash
   railway domain
   ```

### Option B: Render

1. Go to https://render.com
2. Create new "Web Service"
3. Connect your GitHub repository
4. Configure:
   - **Root Directory**: `backend/chatbot`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables (same as Railway)
6. Deploy

### Option C: Fly.io

1. **Install Fly CLI**:
   ```bash
   curl -L https://fly.io/install.sh | sh
   ```

2. **Login**:
   ```bash
   fly auth login
   ```

3. **Create fly.toml** in `backend/chatbot/`:
   ```toml
   app = "physical-ai-chatbot"
   primary_region = "iad"

   [build]
     dockerfile = "Dockerfile"

   [env]
     PORT = "8000"

   [[services]]
     http_checks = []
     internal_port = 8000
     processes = ["app"]
     protocol = "tcp"

     [[services.ports]]
       handlers = ["http"]
       port = 80

     [[services.ports]]
       handlers = ["tls", "http"]
       port = 443
   ```

4. **Deploy**:
   ```bash
   cd backend/chatbot
   fly launch
   fly secrets set OPENAI_API_KEY=your-key
   fly secrets set QDRANT_URL=your-url
   fly secrets set QDRANT_API_KEY=your-key
   fly secrets set DATABASE_URL=your-neon-url
   fly deploy
   ```

## Step 5: Configure Frontend

1. **Create .env.local** in the root directory:
   ```
   CHATBOT_API_URL=https://your-backend-url.railway.app
   ```

2. **Update Vercel environment variables**:
   - Go to your Vercel project settings
   - Add environment variable:
     - Name: `CHATBOT_API_URL`
     - Value: Your deployed backend URL
   - Redeploy

## Step 6: Test End-to-End

1. **Visit your Vercel site**: https://hackathorn-book-physical-ai-or-huma-ebon.vercel.app
2. **Click the chatbot button** (💬) in the bottom-right
3. **Ask a question** about the course content
4. **Verify** you receive a response with citations

## Step 7: Configure CORS (Important!)

Update `backend/chatbot/src/main.py` line 33 to allow your Vercel domain:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "https://hackathorn-book-physical-ai-or-huma-ebon.vercel.app"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

Redeploy the backend after this change.

## Monitoring and Maintenance

### Check Backend Health

```bash
curl https://your-backend-url/health
```

### Check Qdrant Stats

```bash
curl https://your-backend-url/stats
```

### View Backend Logs

**Railway:**
```bash
railway logs
```

**Render:**
- Go to your service dashboard → Logs tab

**Fly.io:**
```bash
fly logs
```

## Troubleshooting

### Chatbot not responding

1. Check browser console for CORS errors
2. Verify backend is running: `curl https://your-backend-url/health`
3. Check backend logs for errors
4. Verify environment variables are set correctly

### "No information found" responses

1. Check if Qdrant has data:
   ```bash
   curl https://your-backend-url/stats
   ```
2. If `total_chunks: 0`, re-run `scripts/load_content.py`

### CORS errors

1. Update CORS configuration in `backend/chatbot/src/main.py`
2. Ensure your Vercel domain is in the `allow_origins` list
3. Redeploy backend

## Cost Estimates (Monthly)

- **Qdrant Cloud** (Free tier): 1GB storage, 0.5GB RAM → $0
- **Railway** (Hobby): 500 hours, 8GB RAM → $5
- **Vercel** (Hobby): 100GB bandwidth → $0
- **Neon** (Free tier): 0.5GB storage → $0
- **OpenAI API**: ~$10-50 (depending on usage)

**Total**: ~$15-55/month

## Security Checklist

- [ ] Environment variables secured (not in code)
- [ ] CORS configured for production domains only
- [ ] Rate limiting enabled (TODO: implement in API)
- [ ] API key rotation schedule set
- [ ] Monitoring and alerts configured

## Next Steps

1. Implement rate limiting per student
2. Add feedback collection endpoint
3. Set up monitoring (Sentry, LogRocket)
4. Configure CDN for faster responses
5. Add analytics tracking
