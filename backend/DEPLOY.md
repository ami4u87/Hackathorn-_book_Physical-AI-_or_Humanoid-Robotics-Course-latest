# Deploying the Physical AI Chatbot Backend

## Option 1: Deploy to Render (Recommended - Free Tier)

### Prerequisites
- GitHub account
- Render account (https://render.com)
- Gemini API key

### Steps:

1. **Push code to GitHub** (if not already done)

2. **Create new Web Service on Render**:
   - Go to https://render.com/dashboard
   - Click "New +" → "Web Service"
   - Connect your GitHub repository
   - Select the repository: `Hackathorn-_book_Physical-AI-_or_Humanoid-Robotics-Course-latest`

3. **Configure the service**:
   - **Name**: `physical-ai-chatbot`
   - **Region**: Choose closest to your users
   - **Branch**: `master`
   - **Root Directory**: `backend/chatbot`
   - **Runtime**: `Python 3`
   - **Build Command**: `pip install -r requirements.txt && python -m nltk.downloader punkt`
   - **Start Command**: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`

4. **Add Environment Variables**:
   - `GEMINI_API_KEY`: Your Gemini API key
   - `QDRANT_URL`: `./qdrant_storage` (for file-based storage)
   - `PYTHON_VERSION`: `3.12.0`

5. **Deploy**:
   - Click "Create Web Service"
   - Wait for deployment (~5-10 minutes)
   - Note the service URL (e.g., `https://physical-ai-chatbot.onrender.com`)

6. **Initialize Qdrant and Load Content**:
   After first deployment, you'll need to run the init and load scripts once:
   - Go to Render Dashboard → Your Service → Shell
   - Run: `python ../../scripts/init_qdrant.py`
   - Run: `python ../../scripts/load_content.py`

## Option 2: Deploy to Railway

1. Go to https://railway.app
2. Click "New Project" → "Deploy from GitHub repo"
3. Select your repository
4. Configure:
   - Root directory: `backend/chatbot`
   - Start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables (same as above)
6. Deploy

## Update Frontend

After backend deployment, update the frontend:

1. **Update Vercel environment variable**:
   - Go to Vercel Dashboard → Your Project → Settings → Environment Variables
   - Add: `CHATBOT_API_URL` = `https://your-backend-url.onrender.com`

2. **Redeploy frontend**:
   - Vercel will auto-deploy on git push, or
   - Manual redeploy from Vercel dashboard

## Testing

Test the deployed backend:
```bash
curl -X POST "https://your-backend-url.onrender.com/query" \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

## Important Notes

- **Free tier limitations**:
  - Render free tier spins down after 15 min of inactivity
  - First request after spin-down takes ~30 seconds
  - 750 hours/month free

- **Qdrant storage**:
  - File-based storage (`./qdrant_storage`) persists on Render
  - For better performance, consider Qdrant Cloud (free tier available)

- **Gemini API**:
  - Free tier: 15 requests/minute, 1500 requests/day
  - Monitor usage at https://makersuite.google.com
