# Database Setup Guide

Complete guide for setting up Qdrant (vector database) and Neon Postgres for the Physical AI chatbot.

## Option A: Cloud Setup (Recommended for Production)

### 1. Qdrant Cloud Setup

#### Step 1: Sign Up
1. Go to https://cloud.qdrant.io
2. Click "Get Started" or "Sign Up"
3. Sign up with GitHub, Google, or email

#### Step 2: Create a Cluster
1. Click "Create Cluster"
2. Choose configuration:
   - **Region**: Select closest to your users (e.g., US East, EU West)
   - **Cluster Size**: Free tier (1GB) is sufficient for development
   - **Name**: `physical-ai-course`
3. Click "Create"
4. Wait 2-3 minutes for cluster provisioning

#### Step 3: Get Connection Details
1. Click on your cluster name
2. Copy the **Cluster URL** (e.g., `https://xxx-yyy-zzz.aws.cloud.qdrant.io:6333`)
3. Click "API Keys" → "Create API Key"
4. Copy the **API Key** (starts with `eyJ...`)
5. Save both securely

#### Step 4: Update Environment Variables
Add to your `.env` file:
```env
QDRANT_URL=https://your-cluster.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-api-key-here
```

### 2. Neon Postgres Setup

#### Step 1: Sign Up
1. Go to https://neon.tech
2. Click "Sign Up"
3. Sign up with GitHub (recommended) or email

#### Step 2: Create a Project
1. Click "Create Project"
2. Configure:
   - **Name**: `physical-ai-course`
   - **Region**: Same as Qdrant (for low latency)
   - **Postgres Version**: 16 (latest)
3. Click "Create Project"

#### Step 3: Get Connection String
1. On the project dashboard, find "Connection Details"
2. Copy the **Connection String**:
   ```
   postgresql://user:password@ep-xxx-xxx.region.aws.neon.tech/neondb?sslmode=require
   ```
3. Save it securely

#### Step 4: Update Environment Variables
Add to your `.env` file:
```env
DATABASE_URL=postgresql://user:password@ep-xxx.region.aws.neon.tech/neondb?sslmode=require
```

#### Step 5: Create Database Schema
Run the initialization script:
```bash
python scripts/init_database.py
```

---

## Option B: Local Development Setup

### 1. Local Qdrant with Docker

#### Step 1: Start Qdrant
```bash
docker run -p 6333:6333 -p 6334:6334 \
  -v $(pwd)/qdrant_storage:/qdrant/storage \
  qdrant/qdrant:v1.7.4
```

Or use docker-compose:
```bash
cd backend
docker-compose up qdrant -d
```

#### Step 2: Verify Running
Visit: http://localhost:6333/dashboard

#### Step 3: Update Environment Variables
```env
QDRANT_URL=http://localhost:6333
# QDRANT_API_KEY not needed for local
```

### 2. Local Postgres with Docker

#### Step 1: Start Postgres
```bash
docker run -d \
  --name physical-ai-postgres \
  -e POSTGRES_USER=physicalai \
  -e POSTGRES_PASSWORD=dev_password \
  -e POSTGRES_DB=physical_ai_course \
  -p 5432:5432 \
  postgres:16
```

#### Step 2: Update Environment Variables
```env
DATABASE_URL=postgresql://physicalai:dev_password@localhost:5432/physical_ai_course
```

#### Step 3: Create Schema
```bash
python scripts/init_database.py
```

---

## Verify Setup

### Test Qdrant Connection

**Cloud:**
```bash
curl https://your-cluster.aws.cloud.qdrant.io:6333/collections \
  -H "api-key: your-api-key"
```

**Local:**
```bash
curl http://localhost:6333/collections
```

Expected response:
```json
{
  "result": {
    "collections": []
  }
}
```

### Test Postgres Connection

```bash
python -c "
import psycopg2
import os
from dotenv import load_dotenv

load_dotenv()
conn = psycopg2.connect(os.getenv('DATABASE_URL'))
print('✅ Successfully connected to Postgres!')
conn.close()
"
```

---

## Environment Variables Summary

Create `backend/.env` with:

```env
# OpenAI
OPENAI_API_KEY=sk-your-key-here

# Qdrant (Cloud)
QDRANT_URL=https://your-cluster.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key

# OR Qdrant (Local)
# QDRANT_URL=http://localhost:6333

# Postgres (Neon)
DATABASE_URL=postgresql://user:pass@ep-xxx.region.aws.neon.tech/neondb?sslmode=require

# OR Postgres (Local)
# DATABASE_URL=postgresql://physicalai:dev_password@localhost:5432/physical_ai_course

# Ports
CHATBOT_PORT=8000
API_PROXY_PORT=8001
```

---

## Cost Estimation

### Free Tier Limits

**Qdrant Cloud:**
- ✅ 1GB storage free
- ✅ Unlimited requests
- ✅ Sufficient for ~50,000 chunks (course needs ~2,000)

**Neon Postgres:**
- ✅ 0.5GB storage free
- ✅ 1 compute unit
- ✅ Sufficient for metadata and logs

**Total Cost: $0/month** for development and small-scale production

### Paid Tiers (if needed later)

**Qdrant:**
- Starter: $25/month (5GB)
- Pro: $95/month (20GB)

**Neon:**
- Pro: $19/month (10GB storage, better compute)

---

## Troubleshooting

### Qdrant Connection Issues

**Error:** `Connection refused`
- Check cluster status in Qdrant dashboard
- Verify URL includes `:6333` port
- Check API key is correct

**Error:** `SSL/TLS error`
- Ensure URL starts with `https://` for cloud
- Use `http://` only for local

### Postgres Connection Issues

**Error:** `password authentication failed`
- Copy connection string exactly from Neon dashboard
- Check for special characters in password (may need URL encoding)

**Error:** `SSL connection required`
- Add `?sslmode=require` to connection string
- Neon requires SSL for security

---

## Next Steps

After database setup:

1. ✅ Databases running
2. ⏳ Initialize Qdrant collection (see `scripts/init_qdrant.py`)
3. ⏳ Create Postgres schema (see `scripts/init_database.py`)
4. ⏳ Load course content into vector database
5. ⏳ Deploy backend services
6. ⏳ Test chatbot queries

---

## Quick Setup Checklist

- [ ] Sign up for Qdrant Cloud OR start local Qdrant
- [ ] Sign up for Neon Postgres OR start local Postgres
- [ ] Copy connection details
- [ ] Create `backend/.env` file
- [ ] Add all environment variables
- [ ] Test connections
- [ ] Run initialization scripts

**Ready? Proceed to Step 2: Initialize Vector Database**
