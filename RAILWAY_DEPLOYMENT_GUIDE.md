# Railway Deployment Guide - Physical AI Chatbot

## Why Railway?
- ✅ $5 free credit monthly (no card required for trial)
- ✅ Easy GitHub integration
- ✅ Automatic HTTPS
- ✅ Great free tier for testing
- ✅ Simple environment variable management

## Prerequisites
- [x] GitHub repository pushed
- [x] Qdrant Cloud data loaded (659 chunks)
- [x] Gemini API key ready

---

## OPTION 1: Railway CLI (Recommended - Fastest)

### Step 1: Install Railway CLI

**Windows (PowerShell):**
```powershell
iwr https://railway.app/install.ps1 | iex
```

**Or using npm:**
```bash
npm install -g @railway/cli
```

### Step 2: Login to Railway
```bash
railway login
```
This will open your browser - sign up with GitHub (free, no card needed for trial)

### Step 3: Initialize Project
```bash
cd "E:\Hackathorn _book_Physical AI _or_Humanoid Robotics Course\backend\chatbot"
railway init
```
- Select: **"Create new project"**
- Enter project name: **physical-ai-chatbot**

### Step 4: Add Environment Variables
```bash
railway variables set GEMINI_API_KEY="AIzaSyDcWPZcd510GQ9ZZR5wXZN553OTDFQlP24"
railway variables set QDRANT_URL="https://cfc7191e-395c-4a70-ac72-979ede526d04.us-east4-0.gcp.cloud.qdrant.io"
railway variables set QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.PtNgKDFuruWwVDasnl-xkrWShCwLnQgECdrY7Rc2n-0"
railway variables set PORT="8000"
```

### Step 5: Deploy
```bash
railway up
```

This will:
- Upload your code
- Install dependencies
- Start the service
- Give you a deployment URL

### Step 6: Generate Public URL
```bash
railway domain
```

This creates a public URL like: `https://physical-ai-chatbot-production.up.railway.app`

---

## OPTION 2: Railway Web Dashboard

### Step 1: Sign Up
1. Go to https://railway.app
2. Click **"Start a New Project"**
3. Click **"Login with GitHub"**
4. Authorize Railway

### Step 2: Create New Project
1. Click **"New Project"**
2. Select **"Deploy from GitHub repo"**
3. Choose: **`Hackathorn-_book_Physical-AI-_or_Humanoid-Robotics-Course-latest`**
4. Railway will detect it's a Python app

### Step 3: Configure Service
Railway should auto-detect settings, but verify:

- **Root Directory**: `backend/chatbot`
- **Build Command**: `pip install -r requirements.txt`
- **Start Command**: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`

### Step 4: Add Environment Variables
1. Click on your service
2. Go to **"Variables"** tab
3. Click **"New Variable"** for each:

```
GEMINI_API_KEY=AIzaSyDcWPZcd510GQ9ZZR5wXZN553OTDFQlP24
QDRANT_URL=https://cfc7191e-395c-4a70-ac72-979ede526d04.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.PtNgKDFuruWwVDasnl-xkrWShCwLnQgECdrY7Rc2n-0
PORT=8000
```

### Step 5: Deploy
1. Click **"Deploy"**
2. Wait 3-5 minutes
3. Check logs for: **"Chatbot service initialized"**

### Step 6: Generate Public Domain
1. Go to **"Settings"** tab
2. Click **"Generate Domain"**
3. Copy your URL

---

## OPTION 3: Other Free Alternatives (If Railway also requires card)

### A) PythonAnywhere (No card required)
- Free tier: 1 web app
- Manual deployment (no auto-deploy from GitHub)
- Good for simple testing

**Steps:**
1. Sign up at https://www.pythonanywhere.com
2. Go to "Web" tab → "Add a new web app"
3. Choose "Manual configuration" → Python 3.10
4. Upload code manually or clone from GitHub
5. Configure WSGI file for uvicorn

### B) Fly.io (May require card)
- Good free tier
- Similar to Railway
- Uses Docker (we have Dockerfile ready)

### C) Local Testing with Ngrok (Temporary)
If you just want to test quickly:

```bash
# Terminal 1: Start backend locally
cd backend/chatbot
uvicorn src.main:app --port 8000

# Terminal 2: Expose with ngrok
ngrok http 8000
```

This gives you a temporary public URL like: `https://abc123.ngrok.io`

---

## Testing Your Deployment

Once deployed on Railway, test with:

### 1. Health Check
```bash
curl https://your-app.up.railway.app/health
```

Expected:
```json
{"status": "healthy", "service": "chatbot", "timestamp": "..."}
```

### 2. Stats Check
```bash
curl https://your-app.up.railway.app/stats
```

Expected:
```json
{"total_chunks": 659, "vector_size": 384, "status": "operational"}
```

### 3. Query Test
```bash
curl -X POST https://your-app.up.railway.app/query \
  -H "Content-Type: application/json" \
  -d "{\"query\": \"What is ROS 2?\"}"
```

---

## Cost Comparison

| Platform | Free Tier | Card Required? | Best For |
|----------|-----------|----------------|----------|
| Railway | $5 credit/month | Not initially | Quick deployment |
| Render | 750 hours/month | Yes (verification) | Production |
| Fly.io | 3 VMs free | Sometimes | Global deployment |
| PythonAnywhere | 1 web app | No | Simple testing |
| Ngrok | Temporary URLs | No | Local testing |

---

## Recommended Path

1. **Try Railway CLI** (Option 1 above) - Fastest and easiest
2. **If Railway needs card**: Use PythonAnywhere or ngrok for testing
3. **For production**: Eventually add card to Railway/Render (they won't charge for free tier usage)

---

## Next Steps After Deployment

1. Get your deployment URL
2. Update Vercel environment variable:
   - `CHATBOT_API_URL=https://your-railway-url.app`
3. Test the chatbot on your live site

Let me know which option you'd like to try!
