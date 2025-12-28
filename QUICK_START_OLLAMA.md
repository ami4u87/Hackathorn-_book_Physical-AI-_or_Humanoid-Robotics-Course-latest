# Quick Start - 100% Free Chatbot with Ollama

## Installation Steps (5 minutes)

### Step 1: Install Ollama

1. Download: https://ollama.com/download/windows
2. Run `OllamaSetup.exe`
3. Installation completes automatically

### Step 2: Pull a Model

Open PowerShell and run:

```powershell
# Recommended: Mistral (best quality/speed balance)
ollama pull mistral
```

This downloads ~4.1 GB. Takes 2-5 minutes depending on your internet speed.

### Step 3: Start the Chatbot

```powershell
# Option A: Use the automated script
.\start_chatbot_ollama.ps1

# Option B: Manual start
cd backend\chatbot
python -m uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
```

### Step 4: Test It!

1. Frontend is already running at: http://localhost:3000
2. Click the 💬 button in bottom-right
3. Ask: "What is ROS 2?"

## What You Get (All FREE!)

✅ **Local Embeddings** - sentence-transformers (no API)
✅ **Vector Search** - Qdrant with 173 course chunks
✅ **LLM Responses** - Ollama with Mistral/Llama2
✅ **No API Costs** - Everything runs locally
✅ **Privacy** - Your data never leaves your machine

## Expected Response Time

- First query: ~5-10 seconds (model loads into RAM)
- Subsequent queries: ~2-4 seconds
- After 5 minutes idle: Model unloads (to save RAM)

## Troubleshooting

### "Ollama not responding"
```powershell
# Check if running
ollama list

# If not, start it
ollama serve
```

### "No models found"
```powershell
# Pull Mistral
ollama pull mistral

# Or Llama2 (alternative)
ollama pull llama2
```

### "Backend won't start"
```powershell
# Check Python dependencies
cd backend\chatbot
pip install --user sentence-transformers httpx

# Try again
python -m uvicorn src.main:app --reload
```

### Model uses too much RAM
```powershell
# Use smaller model (Llama2 7B instead of 13B)
ollama pull llama2  # This is the 7B version by default
```

## System Requirements

- **RAM**: 8GB minimum, 16GB recommended
- **Disk**: 4-8 GB for model
- **GPU**: Optional (CPU works fine, just slower)

## Comparison: Ollama vs OpenAI

| Feature | Ollama | OpenAI |
|---------|--------|---------|
| Cost | **FREE** | ~$5-10/month |
| Privacy | **Local** | Cloud |
| Speed | Medium (2-4s) | Fast (1-2s) |
| Quality | Very Good (90%) | Excellent (95%) |
| Setup | 5 minutes | 1 minute |
| Internet | Download once | Always needed |

## Next Steps

Once running successfully:

1. Deploy to production (see DEPLOYMENT.md)
2. Customize the model (edit `ollama_model` in main.py)
3. Adjust response length (edit `num_predict` in main.py)
4. Try different models: `ollama pull codellama` for code questions

## Alternative Models

```powershell
# For code-heavy questions
ollama pull codellama

# For smaller footprint (2.6 GB)
ollama pull phi

# For highest quality (requires 16GB RAM)
ollama pull llama2:13b
```

## Success!

Your chatbot is now running 100% free with no API costs!

Test it at: http://localhost:3000 💬
