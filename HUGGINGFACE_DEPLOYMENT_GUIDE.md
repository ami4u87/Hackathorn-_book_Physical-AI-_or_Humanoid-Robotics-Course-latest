# Hugging Face Spaces Deployment Guide - Physical AI Chatbot

## Why Hugging Face Spaces?
- ✅ Free community tier (no card required initially)
- ✅ Perfect for AI/ML applications
- ✅ Built-in GPU support available
- ✅ Multiple deployment options (Gradio, Docker, Streamlit)
- ✅ Great for demos and prototypes
- ✅ Easy sharing and discoverability

## Prerequisites
- [x] Hugging Face account (free at https://huggingface.co)
- [x] Qdrant Cloud data loaded (659 chunks)
- [x] Gemini API key ready
- [x] GitHub repository with chatbot code

---

## Deployment Options

We'll cover TWO approaches:
1. **Option A: Gradio Space** - Interactive chat UI (easiest, best for demos)
2. **Option B: Docker Space** - Full FastAPI backend (most flexible)

---

# OPTION A: Gradio Space (Recommended for Quick Demo)

This creates an interactive chatbot UI directly on Hugging Face.

## Step 1: Create Gradio Space

### 1.1 Create New Space
1. Go to https://huggingface.co/spaces
2. Click **"Create new Space"**
3. Fill in:
   - **Owner**: Your username
   - **Space name**: `physical-ai-chatbot`
   - **License**: `MIT`
   - **Select SDK**: `Gradio`
   - **Space hardware**: `CPU basic` (free tier)
   - **Visibility**: `Public` (or Private)
4. Click **"Create Space"**

### 1.2 Clone Your Space Locally

```bash
# Install Git LFS (required for Hugging Face Spaces)
git lfs install

# Clone your new space
git clone https://huggingface.co/spaces/YOUR_USERNAME/physical-ai-chatbot
cd physical-ai-chatbot
```

## Step 2: Create Gradio App Files

### 2.1 Create `app.py`

Create this file in your Space directory:

```python
import gradio as gr
import google.generativeai as genai
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
import os

# Configuration
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Initialize
genai.configure(api_key=GEMINI_API_KEY)
model = genai.GenerativeModel('gemini-1.5-flash')
encoder = SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2')
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY
)

def query_chatbot(message, history):
    """
    Process user query and return chatbot response.

    Args:
        message: Current user message
        history: Chat history (list of [user, bot] message pairs)

    Returns:
        Bot response string
    """
    try:
        # Encode query
        query_vector = encoder.encode(message).tolist()

        # Search Qdrant
        search_results = qdrant_client.search(
            collection_name="physical_ai_course",
            query_vector=query_vector,
            limit=5
        )

        # Extract context
        context_chunks = [hit.payload.get("text", "") for hit in search_results]
        context = "\n\n".join(context_chunks)

        if not context.strip():
            return "I apologize, but I couldn't find relevant information in the course materials. Could you rephrase your question?"

        # Build prompt
        prompt = f"""You are a helpful teaching assistant for the Physical AI and Humanoid Robotics course.

Context from course materials:
{context}

Student question: {message}

Instructions:
- Provide a clear, educational response based on the context above
- If the context doesn't contain enough information, say so
- Use examples when helpful
- Be encouraging and supportive

Answer:"""

        # Generate response
        response = model.generate_content(prompt)
        return response.text

    except Exception as e:
        return f"Error processing your question: {str(e)}"

# Create Gradio interface
demo = gr.ChatInterface(
    query_chatbot,
    title="🤖 Physical AI Course Chatbot",
    description="""
    Ask me anything about ROS 2, robotics, simulation, perception, and humanoid robotics!

    **Topics covered:**
    - ROS 2 fundamentals (nodes, topics, services, actions)
    - Simulation (Gazebo, Unity, Isaac Sim)
    - Perception and computer vision
    - Vision-Language-Action (VLA) models
    - Control systems and path planning
    """,
    examples=[
        "What is ROS 2?",
        "How do I create a ROS 2 publisher?",
        "Explain TF2 coordinate frames",
        "What's the difference between Gazebo and Isaac Sim?",
        "How do I integrate GPT-4 Vision with robotics?"
    ],
    theme=gr.themes.Soft(),
    retry_btn="🔄 Retry",
    undo_btn="↩️ Undo",
    clear_btn="🗑️ Clear"
)

if __name__ == "__main__":
    demo.launch()
```

### 2.2 Create `requirements.txt`

```txt
gradio==4.44.0
google-generativeai==0.8.3
sentence-transformers==3.2.1
qdrant-client==1.12.1
torch==2.5.1
```

### 2.3 Create `README.md`

```markdown
---
title: Physical AI Course Chatbot
emoji: 🤖
colorFrom: blue
colorTo: green
sdk: gradio
sdk_version: 4.44.0
app_file: app.py
pinned: false
license: mit
---

# Physical AI Course Chatbot

An AI-powered chatbot trained on the complete Physical AI and Humanoid Robotics course content.

## Features
- RAG (Retrieval-Augmented Generation) with Qdrant vector database
- Powered by Google Gemini 1.5 Flash
- Trained on 659 course content chunks
- Covers ROS 2, simulation, perception, and VLA integration

## Try it!
Ask questions about robotics, ROS 2, simulation environments, and more!
```

## Step 3: Configure Environment Variables

### 3.1 Add Secrets in Hugging Face UI
1. Go to your Space settings: `https://huggingface.co/spaces/YOUR_USERNAME/physical-ai-chatbot/settings`
2. Scroll to **"Repository secrets"**
3. Add these secrets:

| Name | Value |
|------|-------|
| `GEMINI_API_KEY` | `AIzaSyDcWPZcd510GQ9ZZR5wXZN553OTDFQlP24` |
| `QDRANT_URL` | `https://cfc7191e-395c-4a70-ac72-979ede526d04.us-east4-0.gcp.cloud.qdrant.io` |
| `QDRANT_API_KEY` | `eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.PtNgKDFuruWwVDasnl-xkrWShCwLnQgECdrY7Rc2n-0` |

## Step 4: Deploy

```bash
# Add files
git add app.py requirements.txt README.md
git commit -m "feat: add Gradio chatbot interface"

# Push to Hugging Face
git push
```

Your Space will automatically build and deploy in 2-3 minutes!

## Step 5: Access Your Chatbot

Once deployed, visit:
```
https://huggingface.co/spaces/YOUR_USERNAME/physical-ai-chatbot
```

You'll see an interactive chat interface where users can ask questions!

---

# OPTION B: Docker Space (Full FastAPI Backend)

This deploys your existing FastAPI backend as a Docker container.

## Step 1: Create Docker Space

### 1.1 Create New Space
1. Go to https://huggingface.co/spaces
2. Click **"Create new Space"**
3. Fill in:
   - **Space name**: `physical-ai-chatbot-api`
   - **Select SDK**: `Docker`
   - **Space hardware**: `CPU basic`
4. Click **"Create Space"**

### 1.2 Clone Your Space

```bash
git clone https://huggingface.co/spaces/YOUR_USERNAME/physical-ai-chatbot-api
cd physical-ai-chatbot-api
```

## Step 2: Copy Backend Files

```bash
# Copy your entire backend/chatbot directory
cp -r "E:/Hackathorn _book_Physical AI _or_Humanoid Robotics Course/backend/chatbot/"* .
```

## Step 3: Create Dockerfile

Create `Dockerfile` in the Space root:

```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY src/ ./src/

# Expose port 7860 (Hugging Face Spaces requirement)
EXPOSE 7860

# Start command
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "7860"]
```

## Step 4: Update FastAPI Port

Edit `src/main.py` to ensure it uses port 7860 (Hugging Face requirement):

```python
# No changes needed - uvicorn command handles the port
```

## Step 5: Create README.md

```markdown
---
title: Physical AI Chatbot API
emoji: 🤖
colorFrom: blue
colorTo: green
sdk: docker
pinned: false
license: mit
---

# Physical AI Chatbot API

REST API backend for the Physical AI course chatbot.

## Endpoints

- `GET /health` - Health check
- `GET /stats` - Database statistics
- `POST /query` - Query the chatbot

## Example Usage

\`\`\`bash
curl -X POST https://YOUR_USERNAME-physical-ai-chatbot-api.hf.space/query \\
  -H "Content-Type: application/json" \\
  -d '{"query": "What is ROS 2?"}'
\`\`\`
```

## Step 6: Configure Environment Variables

Add secrets in Space settings:

| Name | Value |
|------|-------|
| `GEMINI_API_KEY` | Your Gemini API key |
| `QDRANT_URL` | Your Qdrant Cloud URL |
| `QDRANT_API_KEY` | Your Qdrant API key |

## Step 7: Deploy

```bash
git add .
git commit -m "feat: add FastAPI backend"
git push
```

Your API will be available at:
```
https://YOUR_USERNAME-physical-ai-chatbot-api.hf.space
```

---

# Testing Your Deployment

## For Gradio Space

1. Open your Space URL
2. Type a question in the chat box
3. See instant responses!

## For Docker/FastAPI Space

### Health Check
```bash
curl https://YOUR_USERNAME-physical-ai-chatbot-api.hf.space/health
```

### Stats Check
```bash
curl https://YOUR_USERNAME-physical-ai-chatbot-api.hf.space/stats
```

### Query Test
```bash
curl -X POST https://YOUR_USERNAME-physical-ai-chatbot-api.hf.space/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

---

# Updating Your Vercel Frontend

If you deployed the Docker API, update your Vercel environment variable:

1. Go to https://vercel.com/dashboard
2. Select your project
3. Go to **Settings** → **Environment Variables**
4. Update `CHATBOT_API_URL`:
   ```
   https://YOUR_USERNAME-physical-ai-chatbot-api.hf.space
   ```
5. Redeploy

---

# Upgrading to Persistent Space

Free community Spaces sleep after inactivity. To keep your Space always-on:

1. Go to Space settings
2. Under **"Space hardware"**, select:
   - **CPU Upgrade** - $0.60/day (~$18/month)
   - **GPU T4** - $0.60/hr (if you need GPU)
3. Click **"Update"**

**Persistent benefits:**
- No cold starts
- Faster responses
- Always available
- Better for production

---

# Cost Comparison

| Tier | Cost | Cold Starts | Best For |
|------|------|-------------|----------|
| **Community (Free)** | $0 | Yes (~30s) | Demos, testing |
| **CPU Upgrade** | ~$18/month | No | Production APIs |
| **GPU T4** | ~$432/month | No | GPU-intensive tasks |

---

# Troubleshooting

## Space Build Fails
- Check logs in the Space UI
- Verify all dependencies in `requirements.txt`
- Ensure Dockerfile syntax is correct

## "Application startup failed"
- Check environment variables are set
- Verify Qdrant Cloud is accessible
- Check Gemini API key is valid

## Slow Responses
- Upgrade to persistent Space (no cold starts)
- Check Qdrant Cloud performance
- Consider GPU upgrade if using large models

## CORS Errors (Docker Space)
Update `src/main.py` to allow Hugging Face domains:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://huggingface.co",
        "https://*.hf.space",
        "https://your-vercel-app.vercel.app"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

# Recommended Approach

**For your use case, I recommend:**

### 🥇 **Best: Gradio Space (Option A)**
- Fastest to deploy (5 minutes)
- Beautiful UI out of the box
- Perfect for showcasing your chatbot
- Free tier is sufficient
- Users can interact immediately

### 🥈 **Alternative: Docker Space + Vercel Frontend**
- If you want to keep your existing Vercel UI
- More control over backend
- Can serve as API for multiple frontends

### 🥉 **Hybrid: Both!**
- Deploy Gradio Space for public demo
- Deploy Docker Space for your Vercel frontend
- Best of both worlds

---

# Next Steps

1. Choose Gradio or Docker approach
2. Create Hugging Face account if needed
3. Follow steps above
4. Test your deployment
5. Share your Space URL!

---

**Deployment Status**: Ready to deploy ✅
**Estimated Time**: 10-15 minutes
**Difficulty**: Easy ⭐⭐☆☆☆

Let me know which option you'd like to proceed with!
