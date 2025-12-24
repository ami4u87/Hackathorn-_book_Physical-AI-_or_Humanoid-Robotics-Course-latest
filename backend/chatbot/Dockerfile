FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first for better caching
COPY backend/chatbot/requirements.txt ./chatbot/
RUN pip install --no-cache-dir -r chatbot/requirements.txt

# Copy shared modules
COPY backend/shared/ ./shared/

# Copy chatbot application code
COPY backend/chatbot/ ./chatbot/

# Set Python path to /app so imports work
ENV PYTHONPATH=/app

# Expose port (Railway will set PORT env var)
EXPOSE 8000

# Run the application (Railway will inject $PORT)
CMD uvicorn chatbot.src.main:app --host 0.0.0.0 --port ${PORT:-8000}
