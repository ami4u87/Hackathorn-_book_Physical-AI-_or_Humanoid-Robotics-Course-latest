# Ollama Setup Guide - 100% Free Chatbot

This guide will help you set up Ollama to run the chatbot completely free with no API costs.

## Step 1: Install Ollama (Windows)

1. **Download Ollama**:
   - Visit: https://ollama.com/download/windows
   - Download `OllamaSetup.exe`
   - Run the installer

2. **Verify Installation**:
   ```powershell
   ollama --version
   ```

## Step 2: Pull a Language Model

Choose one of these models (I recommend Mistral for best quality/speed balance):

### Option A: Mistral 7B (Recommended - Best Balance)
```powershell
ollama pull mistral
```
- Size: ~4.1 GB
- Quality: Excellent
- Speed: Fast
- Best for: General use

### Option B: Llama 2 7B (Also Great)
```powershell
ollama pull llama2
```
- Size: ~3.8 GB
- Quality: Very Good
- Speed: Fast

### Option C: Llama 2 13B (Highest Quality, Slower)
```powershell
ollama pull llama2:13b
```
- Size: ~7.3 GB
- Quality: Excellent
- Speed: Medium
- Requires: More RAM (16GB+)

## Step 3: Test Ollama

```powershell
# Test the model
ollama run mistral "What is ROS 2?"
```

You should see a response. Press `Ctrl+D` or type `/bye` to exit.

## Step 4: Start Ollama Server

Ollama runs as a background service after installation, but you can verify:

```powershell
# Check if Ollama is running
curl http://localhost:11434
```

You should see: `Ollama is running`

## Step 5: Update Backend and Restart

Once Ollama is installed and running, I'll update the backend code to use it instead of OpenAI.

The backend will then:
1. Generate embeddings locally (sentence-transformers) ✅ Already done
2. Search Qdrant for relevant chunks ✅ Already configured
3. Generate responses with Ollama (local LLM) ⏳ Next step

## Troubleshooting

### Ollama not starting
```powershell
# Restart Ollama service
net stop "Ollama"
net start "Ollama"
```

### Port 11434 in use
Check if another process is using the port:
```powershell
netstat -ano | findstr :11434
```

### Model pull fails
- Check your internet connection
- Ensure you have enough disk space (4-8 GB depending on model)

## Next Steps

After installing Ollama and pulling a model, run:
```bash
# I'll update the backend automatically, then you restart:
cd backend/chatbot
python src/main.py
```

## Cost Comparison

| Component | OpenAI | Ollama |
|-----------|---------|---------|
| Embeddings | ~$0.13/1K tokens | FREE (local) |
| Responses | ~$0.01/query | FREE (local) |
| Total | ~$5-10/month | **$0** |
| Privacy | Cloud | **Local** |
| Speed | Fast | Medium |
| Quality | Excellent | Very Good |

## Recommended: Mistral Model

For the best experience, use Mistral:
```powershell
ollama pull mistral
```

It offers the best balance of quality, speed, and resource usage!
