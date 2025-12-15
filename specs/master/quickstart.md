# Quickstart: Physical AI & Humanoid Robotics Development

**Feature**: Physical AI & Humanoid Robotics Capstone Quarter
**Phase**: 1 (Developer Setup)
**Date**: 2025-12-15

## Overview

This guide helps developers set up the complete development environment for Phase 1 (Module 1 + Chatbot v1).

**Prerequisites**:
- Ubuntu 22.04 LTS (native, VM, or WSL2)
- Docker & Docker Compose installed
- Git configured
- Node.js 18+ and npm/yarn (for Docusaurus)
- Python 3.11+ (for backend services)

---

## Repository Clone & Structure

```bash
# Clone repository
git clone https://github.com/organization/physical-ai-book.git
cd physical-ai-book

# Repository structure
physical-ai-book/
├── docs/                   # Docusaurus book content
├── backend/                # FastAPI services
├── ros2_workspace/         # ROS 2 code examples
├── docker/                 # Docker Compose files
├── scripts/                # Utility scripts
└── specs/master/           # Planning artifacts
```

---

## Setup Steps

### 1. Environment Variables

```bash
# Copy template
cp .env.example .env

# Edit .env with your values
nano .env
```

**Required variables** (`.env`):
```bash
# OpenAI API
OPENAI_API_KEY=sk-proj-...

# Qdrant (use Docker default or cloud URL)
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=  # Leave empty for Docker

# Neon Postgres (get from https://neon.tech)
DATABASE_URL=postgresql://user:pass@ep-xyz-abc123.us-east-2.aws.neon.tech/dbname?sslmode=require

# JWT Secrets
JWT_SECRET=your-secret-key-here  # Generate with: openssl rand -hex 32
INSTRUCTOR_JWT_SECRET=instructor-secret-key  # Different from student secret

# API Proxy Config
DAILY_QUOTA=100
```

---

### 2. Docker Setup (Backend Services)

**Start all services** (Qdrant, Postgres proxy, chatbot, API proxy):

```bash
# Start services
cd docker
docker-compose up -d

# Check logs
docker-compose logs -f chatbot
docker-compose logs -f api-proxy

# Stop services
docker-compose down
```

**Services started**:
- **Qdrant**: `http://localhost:6333` (vector database UI at `/dashboard`)
- **Chatbot API**: `http://localhost:8000` (FastAPI docs at `/docs`)
- **API Proxy**: `http://localhost:8001` (FastAPI docs at `/docs`)
- **Validator API**: `http://localhost:8002` (optional; FastAPI docs at `/docs`)

---

### 3. Docusaurus Setup (Book Frontend)

```bash
# Install dependencies
npm install  # or: yarn install

# Start development server
npm run start

# Open browser: http://localhost:3000
```

**Build for production**:
```bash
npm run build
# Static files generated in build/

# Test production build locally
npm run serve
```

---

### 4. ROS 2 Workspace Setup

**Option A: Docker (Recommended)**

```bash
# Build ROS 2 Docker image
docker build -t ros2-dev:latest -f docker/Dockerfile.ros2 .

# Run interactive container
docker run -it --rm \
  -v $(pwd)/ros2_workspace:/ros2_ws \
  ros2-dev:latest

# Inside container: build workspace
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

# Test example
ros2 run module1_examples publisher_node
```

**Option B: Native Ubuntu 22.04**

```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-gz-ros2-control

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Build workspace
cd ros2_workspace
colcon build --symlink-install
source install/setup.bash
```

---

### 5. Initialize Databases

**Qdrant** (create collection):

```bash
# Run initialization script
python scripts/init_qdrant.py

# Or manually:
curl -X PUT 'http://localhost:6333/collections/book_chunks' \
  -H 'Content-Type: application/json' \
  -d '{
    "vectors": {
      "size": 1536,
      "distance": "Cosine"
    }
  }'
```

**Neon Postgres** (create tables):

```bash
# Run schema migration
python scripts/init_database.py

# Or manually execute SQL:
psql $DATABASE_URL < backend/shared/schema.sql
```

---

### 6. Ingest Book Content (RAG Chunking)

```bash
# Chunk Module 1 Markdown files and vectorize
python scripts/ingest_book_content.py --module module-1-ros2

# Expected output:
# Chunking docs/module-1-ros2/*.md
# Generated 45 chunks
# Embedding with OpenAI text-embedding-3-large...
# Inserted 45 chunks into Qdrant
# Done!
```

---

### 7. Verify Setup

**Test Chatbot API**:

```bash
curl -X POST 'http://localhost:8000/query' \
  -H 'Content-Type: application/json' \
  -d '{"query_text": "How do I create a ROS 2 publisher?", "module_filter": "module-1-ros2"}'

# Expected: JSON response with grounded answer + citations
```

**Test API Proxy** (issue JWT first):

```bash
# Generate student JWT (instructor endpoint)
curl -X POST 'http://localhost:8001/auth/token' \
  -H 'Authorization: Bearer <INSTRUCTOR_JWT>' \
  -H 'Content-Type: application/json' \
  -d '{"student_id": "student-test"}'

# Use JWT to make proxied OpenAI request
curl -X POST 'http://localhost:8001/v1/chat/completions' \
  -H 'Authorization: Bearer <STUDENT_JWT>' \
  -H 'Content-Type: application/json' \
  -d '{"model": "gpt-4o-mini", "messages": [{"role": "user", "content": "Hello"}]}'
```

**Test ROS 2 Example**:

```bash
# In ROS 2 workspace
cd ros2_workspace
source install/setup.bash

# Run publisher
ros2 run module1_examples publisher_node

# In another terminal
ros2 topic echo /example_topic
```

---

## Development Workflow

### Adding New Module Content

1. **Write Markdown**: Create files in `docs/module-1-ros2/`
2. **Add to Sidebar**: Edit `sidebars.js` to include new pages
3. **Re-chunk Content**: Run `python scripts/ingest_book_content.py --module module-1-ros2 --update`
4. **Test Chatbot**: Query chatbot to verify new content is retrievable
5. **Commit**: Git commit and push (CI/CD builds Docusaurus)

### Adding New Code Example

1. **Create ROS 2 Package**: `ros2 pkg create --build-type ament_python module1_examples`
2. **Write Code**: Implement node in `module1_examples/module1_examples/`
3. **Add Tests**: Create test file in `test/`
4. **Build**: `colcon build --packages-select module1_examples`
5. **Test**: `colcon test --packages-select module1_examples`
6. **Document**: Add explanation in Markdown with code snippets

### Updating API Contracts

1. **Edit OpenAPI**: Modify `specs/master/contracts/*.yaml`
2. **Generate Models**: Use `datamodel-codegen` to generate Pydantic models (optional)
3. **Update FastAPI**: Implement new endpoints in `backend/chatbot/src/api/routes.py`
4. **Test**: Use FastAPI `/docs` UI to test new endpoints

---

## Troubleshooting

### Qdrant Connection Error

```
Error: Cannot connect to Qdrant at http://localhost:6333
```

**Solution**:
```bash
# Check if Qdrant container is running
docker ps | grep qdrant

# Restart Docker Compose
cd docker
docker-compose restart qdrant
```

### OpenAI API Rate Limit

```
Error: Rate limit exceeded for API key
```

**Solution**:
- Check OpenAI dashboard for quota limits
- Use `gpt-4o-mini` instead of `gpt-4-turbo` for development (cheaper)
- Add rate limiting delays in `ingest_book_content.py` (batch embeddings)

### ROS 2 Package Not Found

```
Package 'module1_examples' not found
```

**Solution**:
```bash
# Re-source workspace
cd ros2_workspace
colcon build --symlink-install
source install/setup.bash

# Verify package exists
ros2 pkg list | grep module1_examples
```

### Docusaurus Build Fails

```
Error: Cannot find module '@docusaurus/core'
```

**Solution**:
```bash
# Clear node_modules and reinstall
rm -rf node_modules package-lock.json
npm install
```

---

## CI/CD Pipeline

**GitHub Actions** workflows automatically run on push/PR:

1. **`.github/workflows/build-book.yml`**: Build Docusaurus, deploy to GitHub Pages
2. **`.github/workflows/test-examples.yml`**: Run ROS 2 integration tests in Docker
3. **`.github/workflows/test-chatbot.yml`**: Run chatbot accuracy benchmarks

**Local CI Testing**:
```bash
# Test book build
npm run build

# Test ROS 2 examples
cd docker
docker-compose -f docker-compose.test.yml up --abort-on-container-exit

# Test chatbot accuracy
python scripts/test_chatbot_accuracy.py --ground-truth tests/module1_qa.json
```

---

## Next Steps

1. **Complete Module 1 Content**: Write all 7 sections + exercises
2. **Validate Code Examples**: Ensure all ROS 2 examples pass CI tests
3. **Generate Ground-Truth Q&A**: Create 20-question test set for Module 1
4. **Beta Test**: Deploy Phase 1 and test with 5 students
5. **Iterate**: Gather feedback, refine chatbot grounding, improve exercises

---

## Resources

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **Docusaurus Docs**: https://docusaurus.io/docs
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **OpenAI API Reference**: https://platform.openai.com/docs/api-reference
- **Project Constitution**: `.specify/memory/constitution.md`
- **Implementation Plan**: `specs/master/plan.md`

