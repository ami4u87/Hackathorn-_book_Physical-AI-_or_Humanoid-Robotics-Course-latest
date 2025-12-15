# Research: Physical AI & Humanoid Robotics – Technical Decisions

**Date**: 2025-12-15
**Feature**: Physical AI & Humanoid Robotics Capstone Quarter
**Phase**: 0 (Research & Technology Decisions)

## Overview

This document consolidates research findings for 8 key technical questions that inform the architectural design of Phase 1 (Module 1 + Chatbot v1).

---

## 1. Docusaurus Chatbot Embedding

**Decision**: Use lazy-loaded React component with intersection observer for chatbot iframe.

**Rationale**:
- Docusaurus supports custom React components via `src/components/`
- Lazy loading prevents blocking initial page render (performance)
- Intersection Observer API loads iframe only when user scrolls to chatbot widget
- Maintains WCAG 2.1 AA compliance with proper ARIA labels

**Implementation Approach**:
```tsx
// src/components/ChatbotEmbed.tsx
import React, { useEffect, useRef, useState } from 'react';

export default function ChatbotEmbed({ apiUrl }) {
  const [isVisible, setIsVisible] = useState(false);
  const containerRef = useRef(null);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsVisible(true);
          observer.disconnect();
        }
      },
      { threshold: 0.1 }
    );

    if (containerRef.current) {
      observer.observe(containerRef.current);
    }

    return () => observer.disconnect();
  }, []);

  return (
    <div ref={containerRef} style={{ minHeight: '400px' }}>
      {isVisible && (
        <iframe
          src={apiUrl}
          title="Course Chatbot"
          style={{ width: '100%', height: '400px', border: 'none' }}
          aria-label="Interactive course chatbot for questions"
        />
      )}
    </div>
  );
}
```

**Positioning**: Fixed bottom-right floating widget (mobile: full-screen modal on tap)

**Alternatives Considered**:
- **Docusaurus plugin**: No official chatbot plugin; custom component more flexible
- **Inline iframe**: Blocks page load; rejected for performance
- **Web component**: Less React-idiomatic for Docusaurus; component preferred

---

## 2. RAG Chunking for Technical Content

**Decision**: Semantic chunking on section boundaries with 750 tokens, 100-token overlap; preserve code blocks whole.

**Rationale**:
- Section-based chunking maintains semantic coherence (headings provide context)
- 750 tokens balances retrieval precision (smaller) vs context richness (larger)
- 100-token overlap ensures no information loss at boundaries
- Code blocks preserved intact to avoid breaking syntax

**Chunking Algorithm**:
1. Parse Markdown with frontmatter library (extract module/section metadata)
2. Split on `##` (section) or `###` (subsection) boundaries
3. For each section: count tokens with `tiktoken` (cl100k_base for GPT-4)
4. If section >750 tokens: split on paragraph boundaries, maintaining 100-token overlap
5. Extract code blocks separately: store as metadata field `code_snippet` but keep in chunk text
6. LaTeX equations: render as text in chunk (e.g., `E = mc^2`), no special handling
7. Images/diagrams: store image path in metadata `diagram_ref`; alt text included in chunk

**Metadata Schema**:
```python
{
  "chunk_id": "uuid",
  "module": "module-1-ros2",
  "section": "2-pubsub",
  "heading": "Publisher-Subscriber Pattern",
  "chunk_index": 0,  # chunk number within section
  "page_range": "12-13",  # estimated page in final PDF
  "code_language": "python",  # if chunk contains code
  "has_diagram": true,
  "diagram_ref": "/img/pubsub-diagram.png",
  "tokens": 745
}
```

**Retrieval Strategy**:
- Query embedding with same model (text-embedding-3-large)
- Cosine similarity search in Qdrant
- Top-5 chunks retrieved
- Filter by module if user on specific module page (metadata filter)
- Re-rank not needed for initial version (acceptable accuracy with top-5)

**Alternatives Considered**:
- **Fixed-size chunking**: Ignores semantic boundaries; rejected
- **Sentence-level chunking**: Too granular, loses context; rejected
- **Separate code vectorization**: Complexity not justified for initial version

---

## 3. OpenAI API Proxy Architecture

**Decision**: FastAPI proxy with python-jose (JWT), slowapi (rate limiting), async Postgres logging.

**Rationale**:
- **python-jose**: Standard JWT library for FastAPI; supports HS256/RS256
- **slowapi**: Integrates with FastAPI; stores rate limit state in memory or Redis
- **Async Postgres**: psycopg3 async driver prevents blocking on logging inserts
- **Daily quota reset**: UTC midnight reset simpler than rolling windows for educational use

**FastAPI Structure**:
```
api-proxy/
├── src/
│   ├── main.py              # FastAPI app with middleware
│   ├── auth.py              # JWT validation (decode token, verify student ID)
│   ├── rate_limiter.py      # slowapi configuration (100 req/day)
│   ├── usage_logger.py      # Async Postgres INSERT for usage logs
│   ├── proxy_handler.py     # Forward to OpenAI API (httpx.AsyncClient)
│   └── config.py            # Settings (JWT secret, OpenAI key, DB connection)
```

**Database Schema** (Postgres):
```sql
CREATE TABLE student_auth (
  student_id VARCHAR(50) PRIMARY KEY,
  jwt_secret VARCHAR(255),  -- per-student secret or global
  quota_daily INT DEFAULT 100,
  created_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE api_usage (
  id SERIAL PRIMARY KEY,
  student_id VARCHAR(50),
  endpoint VARCHAR(50),  -- 'chatbot' or 'vla'
  request_timestamp TIMESTAMP DEFAULT NOW(),
  quota_remaining INT,
  request_tokens INT,
  response_tokens INT,
  cost_usd DECIMAL(10,6)
);

CREATE INDEX idx_usage_student_date ON api_usage(student_id, DATE(request_timestamp));
```

**Sample Endpoint** (Proxy to OpenAI):
```python
@app.post("/v1/chat/completions")
@limiter.limit("100/day")
async def proxy_chat_completion(request: Request, body: ChatCompletionRequest):
    # 1. Validate JWT from Authorization header
    token = request.headers.get("Authorization").split("Bearer ")[1]
    student_id = validate_jwt(token)  # raises 401 if invalid

    # 2. Check rate limit (slowapi middleware handles this)
    # If exceeded, slowapi returns 429 automatically

    # 3. Forward to OpenAI
    async with httpx.AsyncClient() as client:
        response = await client.post(
            "https://api.openai.com/v1/chat/completions",
            json=body.dict(),
            headers={"Authorization": f"Bearer {OPENAI_API_KEY}"}
        )

    # 4. Log usage asynchronously
    await log_usage(student_id, "chatbot", response.json())

    return response.json()
```

**Error Handling**:
- 401: Invalid or expired JWT
- 429: Rate limit exceeded (header: `X-RateLimit-Reset: 1702800000`)
- 503: OpenAI API unavailable (retry with exponential backoff)

**Alternatives Considered**:
- **Redis for rate limiting**: More robust for distributed systems but overkill for single-instance; in-memory acceptable
- **OAuth instead of JWT**: Too complex for internal educational use; JWT simpler

---

## 4. ROS 2 CI/CD in Docker

**Decision**: Use `osrf/ros:humble-desktop-full` Docker image with Gazebo Harmonic, run tests with `colcon test` in GitHub Actions.

**Rationale**:
- **osrf/ros:humble-desktop-full**: Official ROS 2 image; includes rviz2, rqt tools
- **Gazebo Harmonic**: Installed via apt (`gz-harmonic`); headless mode with `LIBGL_ALWAYS_SOFTWARE=1`
- **colcon test**: Standard ROS 2 test runner; outputs junit XML for GitHub UI
- **Docker layer caching**: GitHub Actions caches Docker build layers to speed up CI

**Dockerfile**:
```dockerfile
FROM osrf/ros:humble-desktop-full

# Install Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    ros-humble-gz-ros2-control \
    ros-humble-ros-gz \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for tests
RUN pip3 install pytest pytest-ros

# Set up workspace
WORKDIR /ros2_ws
COPY ros2_workspace/src /ros2_ws/src

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Environment for headless Gazebo
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV QT_QPA_PLATFORM=offscreen

CMD ["/bin/bash"]
```

**GitHub Actions Workflow** (`.github/workflows/test-examples.yml`):
```yaml
name: Test ROS 2 Code Examples

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04

    steps:
      - uses: actions/checkout@v3

      - name: Build Docker image
        run: docker build -t ros2-test:latest -f docker/Dockerfile.test .

      - name: Run colcon tests
        run: |
          docker run --rm \
            -v ${{ github.workspace }}/ros2_workspace:/ros2_ws \
            ros2-test:latest \
            bash -c "source /opt/ros/humble/setup.sh && \
                     source /ros2_ws/install/setup.sh && \
                     colcon test --event-handlers console_direct+ && \
                     colcon test-result --verbose"

      - name: Upload test results
        if: always()
        uses: actions/upload-artifact@v3
        with:
          name: test-results
          path: ros2_workspace/build/*/test_results/**/*.xml
```

**Headless Gazebo Commands**:
```bash
# Launch Gazebo headless (no GUI)
gz sim -s world.sdf

# Run ROS 2 launch file with Gazebo
ros2 launch module1_examples publisher_example.launch.py headless:=true
```

**Caching Strategy**:
- Cache Docker layers with `actions/cache@v3` (Docker buildx cache)
- Cache colcon build artifacts (install/, build/) between runs (saves ~5 minutes)

**Timeout Settings**:
- Default test timeout: 60 seconds per test case
- Gazebo simulation tests: 120 seconds (allow time for physics to stabilize)

**Alternatives Considered**:
- **GitHub Actions ROS 2 action**: Exists but less flexible than Docker; Docker preferred for reproducibility
- **Source build Gazebo**: Too slow in CI; apt packages sufficient

---

## 5. Exercise Validation Security

**Decision**: Behavioral validation scripts (check outcomes, not exact outputs) + SHA-256 hash of (result + timestamp + nonce).

**Rationale**:
- **Behavioral checks**: Validate "subscriber received ≥10 messages" not exact message content (prevents brittleness, reduces gaming)
- **Hash with nonce**: Prevents students from generating fake hashes; nonce provided by instructor
- **Open-source scripts**: Transparency builds trust; obfuscation ineffective and frustrating for students
- **Local validation**: Faster feedback loop than server-side; acceptable security for formative assessment

**Validation Script Design**:
```python
# exercises/module1/ex1_pubsub/validate.py
import rclpy
import hashlib
import time
from datetime import datetime

def validate_pubsub():
    # 1. Launch student's nodes (subprocess)
    publisher = subprocess.Popen(['ros2', 'run', 'student_pkg', 'publisher'])
    time.sleep(1)  # Allow publisher to start
    subscriber = subprocess.Popen(['ros2', 'run', 'student_pkg', 'subscriber'])

    # 2. Monitor /rosout for subscriber output (behavioral check)
    received_count = 0
    timeout = time.time() + 10
    while time.time() < timeout:
        # Check if subscriber logged messages (via ros2 topic echo or log file)
        received_count = count_messages_in_log()
        if received_count >= 10:
            break
        time.sleep(0.1)

    # 3. Cleanup
    publisher.terminate()
    subscriber.terminate()

    # 4. Validation result
    if received_count >= 10:
        result = "PASS"
        feedback = f"Subscriber received {received_count} messages (≥10 required)"
    else:
        result = "FAIL"
        feedback = f"Subscriber received {received_count} messages (<10 required)"

    # 5. Generate hash
    nonce = get_instructor_nonce()  # Fetched from server or config file
    timestamp = datetime.utcnow().isoformat()
    hash_input = f"{result}|{received_count}|{timestamp}|{nonce}"
    validation_hash = hashlib.sha256(hash_input.encode()).hexdigest()

    print(f"{result}: {feedback}")
    print(f"Validation Hash: {validation_hash}")
    print(f"Timestamp: {timestamp}")

    return result, validation_hash

if __name__ == "__main__":
    rclpy.init()
    validate_pubsub()
    rclpy.shutdown()
```

**Hash Generation**:
- **Data to hash**: `PASS|10|2025-12-15T10:30:00|{instructor_nonce}`
- **Nonce distribution**: Instructor provides per-exercise nonce via LMS or config file (students don't know format)
- **Timestamp**: Prevents replay attacks (instructor checks timestamp is recent)

**Preventing Gaming**:
- Scripts check **behavior**, not exact outputs (hard to fake without doing the exercise)
- Nonce unknown to students (can't pre-generate hashes)
- Instructor cross-checks: hash submitted matches expected hash for that exercise
- Open-source scripts: Students can read logic but can't bypass behavioral checks without actually implementing solution

**LMS Integration** (Canvas API):
```python
# Student submits hash via Canvas API
import requests

def submit_to_canvas(hash, timestamp):
    response = requests.post(
        f"https://canvas.university.edu/api/v1/courses/{course_id}/assignments/{assignment_id}/submissions",
        headers={"Authorization": f"Bearer {student_token}"},
        json={"submission": {"body": f"Hash: {hash}\nTimestamp: {timestamp}"}}
    )
    print("Submitted to Canvas:", response.status_code)
```

**Alternatives Considered**:
- **Server-side validation**: More secure but slower feedback; reserved for summative assessments
- **Obfuscated scripts**: Frustrates students; breakable anyway; transparency preferred
- **Exact output matching**: Too brittle (fails on minor formatting differences)

---

## 6. Vector Database Comparison

**Decision**: Use Qdrant (Docker self-hosted) for Phase 1; evaluate migration to pgvector if scaling issues arise.

**Rationale**:
- **Qdrant**: Open-source, excellent Python client, metadata filtering, Docker-friendly, free for self-hosted
- **Performance**: Sub-50ms query latency for top-5 retrieval (1,500 vectors, 1536 dims)
- **Cost**: Zero cost for Docker self-hosted (vs Pinecone $70/month, Weaviate cloud pricing)
- **Metadata filtering**: Native support for filtering by module/section during retrieval
- **Scalability**: Handles 10k+ vectors easily; sufficient for 5-module book (~1,500 chunks)

**Comparison Table**:

| Feature | Qdrant | Pinecone | Weaviate | pgvector |
|---------|--------|----------|----------|----------|
| **Cost (1.5k vectors)** | Free (self-hosted) | $70/month (Starter) | Free (self-hosted) | Free (with Postgres) |
| **Query Latency** | <50ms | <100ms | <50ms | ~100ms (depends on index) |
| **Deployment** | Docker (easy) | Cloud-only | Docker or cloud | Postgres extension |
| **Python Client** | Excellent | Excellent | Good | psycopg2 + SQL |
| **Metadata Filtering** | Native | Native | Native | SQL WHERE clause |
| **Scalability** | 10M+ vectors | 100M+ vectors | 10M+ vectors | 1M vectors (practical) |
| **Backup/Export** | JSON export | API export | JSON export | pg_dump |
| **Open-Source** | ✅ Yes | ❌ No | ✅ Yes | ✅ Yes (Postgres) |

**Recommendation**: **Qdrant (Docker self-hosted)** for Phase 1.

**Migration Path**: If Postgres already in use (Neon), pgvector is attractive for consolidation; migrate by exporting Qdrant JSON and importing to pgvector table.

**Example Qdrant Code**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

client = QdrantClient(url="http://localhost:6333")

# Create collection
client.recreate_collection(
    collection_name="book_chunks",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
)

# Insert chunk
client.upsert(
    collection_name="book_chunks",
    points=[
        PointStruct(
            id=uuid.uuid4(),
            vector=embedding,  # 1536-dim from OpenAI
            payload={"module": "module-1-ros2", "section": "2-pubsub", "text": chunk_text}
        )
    ]
)

# Query with metadata filter
results = client.search(
    collection_name="book_chunks",
    query_vector=query_embedding,
    limit=5,
    query_filter={"must": [{"key": "module", "match": {"value": "module-1-ros2"}}]}
)
```

**Alternatives Considered**:
- **Pinecone**: Best performance but paid; not justified for educational budget
- **Weaviate**: Good but more complex setup than Qdrant; similar capabilities
- **pgvector**: Attractive for consolidation but query performance lags dedicated vector DBs

---

## 7. Neon Postgres Serverless Limits

**Decision**: Use Neon free tier for Phase 1 (sufficient for 50 students); upgrade to Pro ($19/month) if exceeding 10GB storage.

**Rationale**:
- **Free tier limits**: 0.5GB storage, 100 compute hours/month, 20 concurrent connections
- **Expected usage**: ~100MB for 2k queries + usage logs; well within 0.5GB
- **Compute hours**: Serverless auto-scales to zero; 100 hours sufficient for intermittent development + student queries
- **Connections**: 20 concurrent connections adequate for FastAPI with 10 workers (2 connections per worker)
- **Cold start**: ~1-2 seconds; acceptable for infrequent chatbot queries

**Connection Pooling**:
- Neon handles connection pooling automatically (no pgbouncer needed)
- FastAPI: use `asyncpg` with connection pool (max 10 connections)

```python
import asyncpg

pool = await asyncpg.create_pool(
    dsn=NEON_DATABASE_URL,
    min_size=2,
    max_size=10
)
```

**Schema Recommendations**:
```sql
-- Chatbot query logs
CREATE TABLE chatbot_queries (
  id SERIAL PRIMARY KEY,
  student_id VARCHAR(50),
  query_text TEXT,
  retrieved_chunks JSONB,  -- Store chunk IDs and scores
  response_text TEXT,
  feedback INT,  -- Optional: 1-5 rating
  timestamp TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_queries_student ON chatbot_queries(student_id);
CREATE INDEX idx_queries_timestamp ON chatbot_queries(timestamp);

-- API proxy usage (from decision #3)
-- (schema already defined above)
```

**Performance**:
- **INSERT latency**: ~10-20ms (async non-blocking)
- **SELECT queries**: ~20-50ms for simple lookups
- **Cold start**: 1-2 seconds (first query after idle period)

**Backup Strategy**:
- Neon auto-backups daily (retained 7 days on free tier)
- Manual export via `pg_dump` for archival

**Upgrade Path**:
- **Pro tier ($19/month)**: 10GB storage, 300 compute hours, 100 connections
- Trigger: exceeds 0.5GB storage or 100 compute hours/month

**Alternatives Considered**:
- **Supabase**: Similar serverless Postgres; comparable free tier; Neon chosen for familiarity
- **University Postgres**: If available, eliminates external dependency; check with IT

---

## 8. Robotis OP3 ROS 2 Support

**Decision**: Use Robotis OP3 with **community-maintained ROS 2 port**; official ROS 1 URDF requires conversion.

**Rationale**:
- **Official repo** (ROBOTIS-GIT/ROBOTIS-OP3): ROS 1 only; not actively maintained for ROS 2
- **Community ROS 2 port**: Exists on GitHub (search "robotis op3 ros2 humble"); URDF converted, basic Gazebo support
- **URDF availability**: Official ROS 1 URDF can be adapted (minor URDF changes for Gazebo Harmonic)
- **MoveIt 2 config**: Not officially available; must be generated using MoveIt Setup Assistant
- **Gazebo compatibility**: Works with Gazebo Classic (ROS 2 bridge); Harmonic support requires URDF updates

**Assessment**:
- **ROS 2 Humble readiness**: Community port is **beta quality**; needs testing and documentation improvements
- **Work required**:
  1. Fork community ROS 2 port of OP3
  2. Validate URDF in Gazebo Harmonic (update collision/inertia if needed)
  3. Generate MoveIt 2 config using Setup Assistant
  4. Create ros2_control config (joint_state_broadcaster, position_controller)
  5. Document setup process in Module 2

**Backup Recommendation**:
If OP3 proves too unstable, use **TIAGo (PAL Robotics)** or **PR2 (Willow Garage)**:
- **TIAGo**: Better ROS 2 support; mobile manipulator (not full humanoid but good for manipulation)
- **PR2**: Well-documented ROS 2 port; 2-armed mobile robot (humanoid-like manipulation)

**Steps to Prepare OP3**:
```bash
# 1. Clone community ROS 2 port
cd ~/ros2_ws/src
git clone https://github.com/community/robotis_op3_ros2.git

# 2. Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# 3. Build
cd ~/ros2_ws
colcon build --packages-select robotis_op3_description robotis_op3_gazebo

# 4. Test in Gazebo
ros2 launch robotis_op3_gazebo op3_gazebo.launch.py

# 5. Generate MoveIt 2 config (if not included)
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

**Alternatives Considered**:
- **Custom URDF**: Too much work; prefer adapting existing model
- **NASA Valkyrie**: Overly complex (54 DOF) for educational use; OP3 simpler (20 DOF)
- **TIAGo/PR2**: Good fallback but OP3 chosen first for humanoid focus

---

## Summary of Decisions

| Question | Decision | Key Rationale |
|----------|----------|---------------|
| **1. Chatbot Embedding** | React component with lazy-load iframe | Performance (non-blocking), Docusaurus-native |
| **2. RAG Chunking** | 750 tokens, section boundaries, code preserved | Semantic coherence, optimal retrieval |
| **3. API Proxy** | FastAPI + JWT + slowapi + async Postgres | Standard stack, simple rate limiting |
| **4. ROS 2 CI/CD** | Docker + osrf/ros:humble + GitHub Actions | Official image, headless Gazebo, reproducible |
| **5. Exercise Validation** | Behavioral checks + hash(result+nonce) | Security vs usability balance, prevents gaming |
| **6. Vector DB** | Qdrant (Docker self-hosted) | Free, performant, excellent Python client |
| **7. Neon Postgres** | Free tier (upgrade to Pro if needed) | Sufficient for Phase 1, serverless convenience |
| **8. Robotis OP3** | Community ROS 2 port (with validation) | Open-source humanoid, 20 DOF, adaptable |

---

## Next Steps (Phase 1)

1. Create `data-model.md` defining entities (Module, Chunk, Query, Exercise, Student)
2. Generate API contracts (OpenAPI specs for chatbot, proxy, validator)
3. Create `quickstart.md` with developer setup instructions
4. Update agent context with new technologies (Qdrant, Neon, slowapi)
5. Proceed to `/sp.tasks` for implementation breakdown

