# Render Deployment Guide - Physical AI Chatbot

## Overview
This guide will help you deploy the chatbot backend to Render with Qdrant Cloud.

## Prerequisites Checklist
- [x] GitHub repository with chatbot code pushed
- [x] Gemini API key configured
- [x] Qdrant Cloud credentials available
- [ ] Render account created

## STEP 1: Activate Qdrant Cloud

You already have Qdrant Cloud credentials in your `.env` file:
- **URL**: `https://cfc7191e-395c-4a70-ac72-979ede526d04.us-east4-0.gcp.cloud.qdrant.io`
- **API Key**: `eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.PtNgKDFuruWwVDasnl-xkrWShCwLnQgECdrY7Rc2n-0`

### 1.1 Verify Qdrant Cloud Access
1. Go to https://cloud.qdrant.io
2. Log in to your account
3. Verify your cluster is active (should show "Running")

## STEP 2: Create Render Web Service

### 2.1 Sign Up / Log In
1. Go to https://render.com
2. Click **"Get Started"** or **"Sign In"**
3. Sign up with GitHub (recommended) or email

### 2.2 Create New Web Service
1. From Render Dashboard, click **"New +"** button (top right)
2. Select **"Web Service"**
3. Click **"Connect GitHub"** if not already connected
4. Authorize Render to access your repositories

### 2.3 Select Repository
1. Find and select: **`Hackathorn-_book_Physical-AI-_or_Humanoid-Robotics-Course-latest`**
2. Click **"Connect"**

## STEP 3: Configure Web Service

### 3.1 Basic Settings
Fill in these fields exactly:

| Field | Value |
|-------|-------|
| **Name** | `physical-ai-chatbot` |
| **Region** | `Oregon (US West)` (or closest to you) |
| **Branch** | `master` |
| **Root Directory** | `backend/chatbot` |
| **Runtime** | `Python 3` |

### 3.2 Build & Deploy Commands

| Field | Value |
|-------|-------|
| **Build Command** | `pip install -r requirements.txt` |
| **Start Command** | `uvicorn src.main:app --host 0.0.0.0 --port $PORT` |

### 3.3 Instance Type
- For testing: Select **"Free"** ($0/month, goes to sleep after 15 min inactivity)
- For production: Select **"Starter"** ($7/month, always on)

> **Note**: Free tier is fine for testing but will have cold starts

## STEP 4: Configure Environment Variables

Click **"Advanced"** section, then **"Add Environment Variable"** for each:

### Required Variables:

1. **GEMINI_API_KEY**
   ```
   AIzaSyDcWPZcd510GQ9ZZR5wXZN553OTDFQlP24
   ```

2. **QDRANT_URL**
   ```
   https://cfc7191e-395c-4a70-ac72-979ede526d04.us-east4-0.gcp.cloud.qdrant.io
   ```

3. **QDRANT_API_KEY**
   ```
   eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.PtNgKDFuruWwVDasnl-xkrWShCwLnQgECdrY7Rc2n-0
   ```

4. **PORT** (Render provides this automatically, but you can add it explicitly)
   ```
   8000
   ```

### Optional Variables (for future use):

5. **DATABASE_URL** (if you plan to use PostgreSQL logging)
   ```
   postgresql://neondb_owner:npg_XFq6ebU3VYpz@ep-snowy-cell-adoozlvj-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require
   ```

6. **ALLOWED_ORIGINS** (CORS configuration)
   ```
   http://localhost:3000,https://hackathorn-book-physical-ai-or-huma-ebon.vercel.app
   ```

## STEP 5: Deploy

1. Click **"Create Web Service"** button at the bottom
2. Render will start the deployment process:
   - Cloning repository ✓
   - Installing dependencies ✓
   - Starting service ✓
3. Wait 3-5 minutes for initial deployment
4. Watch the logs for any errors

### Expected Log Output:
```
[INFO] Loading sentence-transformers model...
[OK] Model loaded: all-MiniLM-L6-v2 (384 dimensions)
[OK] Connected to Qdrant Cloud
[OK] Chatbot service initialized
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

## STEP 6: Get Your Deployment URL

Once deployed, Render will provide a URL like:
```
https://physical-ai-chatbot.onrender.com
```

Copy this URL - you'll need it for:
1. Testing the API
2. Configuring your frontend
3. Loading data into Qdrant Cloud

## STEP 7: Load Data into Qdrant Cloud

After deployment is successful, you need to load your course content into Qdrant Cloud.

### 7.1 Update Local .env for Qdrant Cloud
Run this command to initialize Qdrant Cloud collection:

```bash
# Set environment to use Qdrant Cloud
$env:QDRANT_URL="https://cfc7191e-395c-4a70-ac72-979ede526d04.us-east4-0.gcp.cloud.qdrant.io"
$env:QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.PtNgKDFuruWwVDasnl-xkrWShCwLnQgECdrY7Rc2n-0"

# Initialize collection
python scripts/init_qdrant.py

# Load course content
python scripts/load_content.py
```

This will take about 5-10 minutes to upload all 659 chunks to Qdrant Cloud.

## STEP 8: Test Your Deployment

### 8.1 Health Check
```bash
curl https://physical-ai-chatbot.onrender.com/health
```

Expected response:
```json
{
  "status": "healthy",
  "service": "chatbot",
  "timestamp": "2025-12-19T..."
}
```

### 8.2 Stats Check
```bash
curl https://physical-ai-chatbot.onrender.com/stats
```

Expected response:
```json
{
  "total_chunks": 659,
  "vector_size": 384,
  "status": "operational"
}
```

### 8.3 Query Test
```bash
curl -X POST https://physical-ai-chatbot.onrender.com/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

## STEP 9: Update Frontend Configuration

### 9.1 Update Vercel Environment Variable
1. Go to https://vercel.com/dashboard
2. Select your project: `hackathorn-book-physical-ai-or-huma-ebon`
3. Go to **Settings** → **Environment Variables**
4. Add or update:
   - **Name**: `CHATBOT_API_URL`
   - **Value**: `https://physical-ai-chatbot.onrender.com`
5. Click **"Save"**
6. Redeploy: Go to **Deployments** → click **"Redeploy"** on latest deployment

### 9.2 Update CORS in Backend
The backend is already configured to allow your Vercel domain. If you get CORS errors, verify line 40 in `backend/chatbot/src/main.py`:

```python
allow_origins=["*"],  # This allows all origins for testing
```

For production, update to:
```python
allow_origins=[
    "http://localhost:3000",
    "https://hackathorn-book-physical-ai-or-huma-ebon.vercel.app"
],
```

## Troubleshooting

### Deployment Failed
- Check Render logs for error messages
- Verify all environment variables are set correctly
- Ensure `requirements.txt` has all dependencies

### "No information found" in responses
- Verify Qdrant Cloud has data: check `/stats` endpoint
- Re-run `python scripts/load_content.py` if needed

### CORS Errors
- Verify `ALLOWED_ORIGINS` environment variable in Render
- Check browser console for specific CORS error messages

### Service Goes to Sleep (Free Tier)
- First request after 15 min will take 30-60 seconds (cold start)
- Consider upgrading to Starter plan ($7/month) for always-on

## Monitoring

### View Logs
1. Go to Render Dashboard
2. Click on your service
3. Click **"Logs"** tab
4. Monitor real-time logs

### Metrics
Render provides basic metrics on the service dashboard:
- CPU usage
- Memory usage
- Request count
- Response times

## Cost Summary

| Service | Plan | Cost |
|---------|------|------|
| Render | Free | $0 |
| Render | Starter | $7/month |
| Qdrant Cloud | Free Tier | $0 (1GB storage) |
| Vercel | Hobby | $0 |
| Google Gemini API | Pay-as-you-go | ~$5-20/month (usage-based) |

**Total for Free Tier**: ~$5-20/month (Gemini API only)
**Total for Production**: ~$12-27/month (Render Starter + Gemini)

## Next Steps

- [ ] Test chatbot on your Vercel site
- [ ] Monitor usage and costs
- [ ] Set up error tracking (optional: Sentry)
- [ ] Configure rate limiting for production
- [ ] Add user feedback collection

## Support

If you encounter issues:
1. Check Render logs first
2. Verify environment variables
3. Test health endpoint
4. Review Qdrant Cloud status

---

**Deployment Status**: Ready for production ✅
**Last Updated**: 2025-12-19
