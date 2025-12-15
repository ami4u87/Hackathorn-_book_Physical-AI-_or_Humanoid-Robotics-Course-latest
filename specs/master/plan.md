# Implementation Plan: Physical AI & Humanoid Robotics – Capstone Quarter

**Branch**: `master` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/master/spec.md`

## Summary

Develop a comprehensive technical book (40k-60k words) on Physical AI and Humanoid Robotics with an embedded RAG chatbot, deployed as a Docusaurus site on GitHub Pages. The project follows a **progressive staged deployment** strategy: Phase 1 delivers Module 1 (ROS 2 Fundamentals) with a basic chatbot trained only on that module, enabling early validation before scaling to remaining modules (Simulation, Perception/Control, VLA, Capstone).

**Primary Requirement**: Students learn embodied AI through hands-on ROS 2 development, simulation (Gazebo/Unity/Isaac), perception pipelines, VLA integration, and a voice-commanded autonomous humanoid manipulation task.

**Technical Approach**:
- **Book**: Docusaurus with MDX for interactive components; Mermaid/draw.io diagrams; reproducible code examples with Docker environments
- **RAG Chatbot**: FastAPI backend with OpenAI GPT-4 + text-embedding-3-large; Qdrant vector DB (750-token chunks, 100-token overlap); Neon Postgres for logging; strict grounding with citation validation
- **API Proxy**: Instructor-managed FastAPI proxy for OpenAI API with JWT auth and rate limiting (100 req/day per student)
- **Robot Model**: Robotis OP3 (20 DOF) for consistency across all modules
- **Deployment**: GitHub Pages (frontend), containerized FastAPI services, CI/CD with GitHub Actions

## Technical Context

**Language/Version**:
- Python 3.11+ (backend, robotics code, validation scripts)
- TypeScript/JavaScript (Docusaurus customizations, chatbot UI)
- Markdown/MDX (book content)

**Primary Dependencies**:
- **Frontend**: Docusaurus 3.x, React 18, @docusaurus/theme-classic
- **Backend**: FastAPI 0.104+, uvicorn, pydantic 2.x
- **RAG Stack**: openai 1.x, qdrant-client 1.7+, psycopg2-binary, python-jose (JWT), slowapi (rate limiting)
- **Robotics**: ROS 2 Humble, rclpy, geometry_msgs, sensor_msgs, tf2_ros, moveit_py
- **Testing**: pytest, pytest-asyncio, httpx (API tests)
- **Build/Deploy**: Docker, Docker Compose, GitHub Actions

**Storage**:
- **Vector DB**: Qdrant (cloud-hosted or self-hosted Docker) for book content embeddings
- **Relational DB**: Neon Postgres (serverless) for chatbot query logs, API proxy usage analytics, student authentication
- **Static Assets**: GitHub repository (Markdown files, images, diagrams); GitHub Pages CDN (deployed HTML/CSS/JS)

**Testing**:
- **Book Content**: Manual review + automated link/image validation
- **Code Examples**: pytest integration tests in Docker containers (ROS 2 Humble); CI/CD runs all 50+ examples
- **Chatbot Accuracy**: Ground-truth test set (100 Q&A pairs); F1 score ≥0.8; zero hallucination threshold
- **API Proxy**: pytest for auth, rate limiting, quota enforcement
- **Exercise Validation**: Automated scripts (Python/Bash) students run locally; hash-based grading validation

**Target Platform**:
- **Book Readers**: Modern browsers (Chrome, Firefox, Safari, Edge); mobile-responsive
- **Development Environment**: Ubuntu 22.04 LTS (native, VM, or Docker); Windows/macOS via Docker only
- **Backend Services**: Linux containers (Docker); deployable to cloud (AWS, GCP, Azure) or university servers

**Project Type**: Multi-component web application
- **Frontend**: Static site generator (Docusaurus) with embedded chatbot iframe
- **Backend**: Microservices (RAG chatbot API, OpenAI proxy, exercise validator endpoint)
- **Documentation**: Markdown-based book with code repositories

**Performance Goals**:
- **Chatbot Response Time**: p95 ≤3 seconds (10 concurrent users)
- **Page Load Time**: p95 ≤2 seconds for book pages (excluding chatbot iframe)
- **API Proxy Latency**: <100ms overhead vs. direct OpenAI API calls
- **Search**: Docusaurus search returns results in <500ms

**Constraints**:
- **Book Size**: 40,000–60,000 words total; 8,000–12,000 per module
- **Chatbot Accuracy**: ≥90% correctness on 100-question test set; zero hallucinations in 20-response sample per module
- **API Quota**: 100 requests/day per student (VLA + chatbot combined)
- **Hardware**: Code examples must run on mid-range hardware (16GB RAM, 4-core CPU, GTX 1660); Isaac Sim optional (RTX GPU)
- **Dependencies**: All version-pinned; Docker images for reproducibility
- **Accessibility**: WCAG 2.1 AA (alt text, semantic HTML, keyboard navigation)

**Scale/Scope**:
- **Content**: 5 modules, ≥50 code examples, ≥30 diagrams, ≥40 citations (≥50% peer-reviewed)
- **Users**: Designed for cohorts of 20-50 students per semester
- **Chatbot Usage**: ~1,000-2,000 queries per semester (estimated 40 queries/student)
- **Repository**: ~500-1,000 files (Markdown, Python, URDF, launch files, diagrams)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Technical Accuracy Above All
- ✅ **PASS**: All code examples will be tested in simulation (ROS 2 Humble + Gazebo/Unity/Isaac) before publication
- ✅ **PASS**: Citations required (≥40 sources, ≥50% peer-reviewed, APA 7 format)
- ✅ **PASS**: Chatbot grounding enforced (answers only from book content; zero hallucination tolerance)
- ⚠️ **ACTION REQUIRED**: Verify ROS 2 API accuracy during Phase 1 (cross-reference official docs)

### II. Reproducibility First
- ✅ **PASS**: Docker Compose files provided for environment consistency
- ✅ **PASS**: All dependencies version-pinned in requirements.txt, package.xml
- ✅ **PASS**: CI/CD tests all code examples on Ubuntu 22.04 + ROS 2 Humble
- ✅ **PASS**: Expected outputs documented for each code example

### III. Safety-First Robotics Guidance
- ✅ **PASS**: Safety warnings included in book (simulation-first testing; emergency stop procedures)
- ✅ **PASS**: No unsafe code patterns (e.g., unchecked joint velocity commands, missing collision detection)
- ⚠️ **ACTION REQUIRED**: Review all manipulation examples for safety compliance during Phase 1

### IV. Grounded RAG Chatbot (Zero Hallucination)
- ✅ **PASS**: RAG retrieval from Qdrant only; no external knowledge
- ✅ **PASS**: Citation validation (every answer must reference Module/Section)
- ✅ **PASS**: Accuracy testing (≥90% on 100-question ground-truth set)
- ✅ **PASS**: Human review sampling (20 responses per module for hallucination detection)

### V. Test-Driven Development for All Code
- ✅ **PASS**: pytest for Python code; launch_testing for ROS 2 integration tests
- ✅ **PASS**: Exercise validation scripts with pass/fail feedback
- ⚠️ **ACTION REQUIRED**: Define test coverage targets during Phase 1 (e.g., ≥80% for backend services)

### VI. Clear Engineering-Level Explanations
- ✅ **PASS**: Target audience is senior CS/AI students (programming background assumed)
- ✅ **PASS**: Terminology glossary maintained; consistent naming per ROS 2 conventions
- ✅ **PASS**: Diagrams required (≥30 total; Mermaid/draw.io for editability)

### VII. Modular Structure with Progressive Complexity
- ✅ **PASS**: 5-module structure with explicit dependencies (Module 1 → 2 → 3 → 4 → 5)
- ✅ **PASS**: Progressive staged deployment (Module 1 + Chatbot v1 first)

**Overall Gate Status**: ✅ **PASS** (with 3 action items to address in Phase 1)

## Project Structure

### Documentation (this feature)

```text
specs/master/
├── spec.md              # Feature specification
├── plan.md              # This file (architectural plan)
├── research.md          # Phase 0: Technology research and decisions
├── data-model.md        # Phase 1: Entities and relationships
├── quickstart.md        # Phase 1: Setup instructions for developers
├── contracts/           # Phase 1: API schemas
│   ├── chatbot-api.yaml        # OpenAPI spec for RAG chatbot
│   ├── api-proxy.yaml          # OpenAPI spec for OpenAI proxy
│   └── validator-api.yaml      # OpenAPI spec for exercise validator
└── tasks.md             # Phase 2: Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root)

```text
# Repository structure (multi-component web application)

# Book content (Docusaurus)
docs/
├── intro.md                     # Course introduction
├── module-1-ros2/              # Module 1: ROS 2 Fundamentals
│   ├── index.md
│   ├── 1-installation.md
│   ├── 2-pubsub.md
│   ├── 3-services.md
│   ├── 4-actions.md
│   ├── 5-parameters.md
│   ├── 6-launch.md
│   ├── 7-tf2.md
│   └── exercises.md
├── module-2-simulation/        # Module 2: Gazebo/Unity/Isaac
├── module-3-perception/        # Module 3: Perception & Control
├── module-4-vla/               # Module 4: Vision-Language-Action
├── module-5-capstone/          # Module 5: Capstone Project
└── glossary.md

docusaurus.config.js            # Docusaurus configuration
sidebars.js                     # Navigation structure
src/
├── components/                 # Custom React components
│   └── ChatbotEmbed.tsx       # Chatbot iframe wrapper
├── css/                        # Custom styles
└── pages/                      # Static pages (about, contact)

static/
├── img/                        # Images (robot diagrams, screenshots)
└── diagrams/                   # Mermaid/draw.io source files

# Backend services
backend/
├── chatbot/                    # RAG chatbot service
│   ├── src/
│   │   ├── main.py            # FastAPI app entry point
│   │   ├── models/
│   │   │   ├── query.py       # Pydantic models for requests/responses
│   │   │   └── chunk.py       # RAG chunk metadata
│   │   ├── services/
│   │   │   ├── embedder.py    # OpenAI embeddings wrapper
│   │   │   ├── retriever.py   # Qdrant retrieval logic
│   │   │   └── grounding.py   # Citation validation and grounding enforcement
│   │   ├── api/
│   │   │   └── routes.py      # POST /query endpoint
│   │   └── config.py          # Environment variables, settings
│   ├── tests/
│   │   ├── test_retriever.py
│   │   ├── test_grounding.py
│   │   └── test_accuracy.py   # Ground-truth Q&A test suite
│   ├── requirements.txt
│   └── Dockerfile
├── api-proxy/                  # OpenAI API proxy with rate limiting
│   ├── src/
│   │   ├── main.py
│   │   ├── auth.py            # JWT authentication
│   │   ├── rate_limiter.py    # Quota enforcement (slowapi)
│   │   ├── usage_logger.py    # Log to Neon Postgres
│   │   └── config.py
│   ├── tests/
│   ├── requirements.txt
│   └── Dockerfile
└── shared/                     # Shared utilities
    ├── db.py                   # Neon Postgres connection
    └── logging_config.py

# ROS 2 code examples
ros2_workspace/
├── src/
│   ├── module1_examples/       # Module 1 code
│   │   ├── publisher_example/
│   │   │   ├── setup.py
│   │   │   ├── package.xml
│   │   │   └── publisher_example/
│   │   │       └── publisher_node.py
│   │   ├── subscriber_example/
│   │   ├── service_example/
│   │   ├── action_example/
│   │   └── param_example/
│   ├── module2_examples/       # Simulation examples
│   ├── module3_examples/       # Perception examples
│   ├── module4_examples/       # VLA examples
│   └── module5_capstone/       # Capstone starter code
├── install/                    # Built packages
└── build/

# Robot models
models/
├── robotis_op3/
│   ├── urdf/
│   │   └── robotis_op3.urdf
│   ├── meshes/
│   └── config/
│       ├── controllers.yaml   # ros2_control config
│       └── moveit2/           # MoveIt 2 configuration

# Exercise validation scripts
exercises/
├── module1/
│   ├── ex1_pubsub/
│   │   ├── validate.py        # Automated validation script
│   │   └── solution_hash.txt  # Expected hash for grading
│   ├── ex2_service/
│   └── ex3_params/
├── module2/
├── module3/
├── module4/
└── module5/

# Deployment and infrastructure
docker/
├── docker-compose.yml         # All services (chatbot, proxy, Qdrant, Postgres)
├── docker-compose.dev.yml     # Development overrides
└── nginx/
    └── nginx.conf             # Reverse proxy config (if needed)

scripts/
├── setup_dev_env.sh           # Developer setup script
├── ingest_book_content.py     # Chunk and vectorize Markdown to Qdrant
├── generate_ground_truth.py   # Create Q&A test set from book
└── validate_all_examples.sh   # CI/CD script to run all code examples

.github/
└── workflows/
    ├── build-book.yml         # Deploy Docusaurus to GitHub Pages
    ├── test-examples.yml      # Run ROS 2 code examples in Docker
    └── test-chatbot.yml       # Run chatbot accuracy benchmarks

# Configuration
.env.example                   # Template for API keys, DB credentials
requirements.txt               # Top-level Python dependencies (for local dev)
package.json                   # Docusaurus dependencies
```

**Structure Decision**: Multi-component web application chosen due to:
1. **Frontend (Docusaurus)**: Static site generator for book content; deployed to GitHub Pages
2. **Backend Services**: FastAPI microservices (chatbot, API proxy) require separate containerized deployment
3. **ROS 2 Workspace**: Standard colcon workspace structure for robotics code examples
4. **Exercise Validation**: Standalone scripts students run locally (not web-based)

Separation enables independent deployment of book (static site) and backend services (containerized APIs).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

*No violations requiring justification. All constitution principles are met.*

## Phase 0: Research & Technology Decisions

### Research Questions

1. **Docusaurus Chatbot Embedding**: What is the best practice for embedding a chatbot iframe in Docusaurus without blocking page load?
2. **RAG Chunking for Technical Content**: How should technical documentation with code blocks be chunked for optimal retrieval accuracy?
3. **OpenAI API Proxy Architecture**: What is the reference architecture for rate-limited API proxies with JWT auth and usage logging?
4. **ROS 2 CI/CD in Docker**: How to run ROS 2 integration tests in GitHub Actions with Gazebo headless mode?
5. **Exercise Validation Pattern**: What is a secure pattern for automated exercise validation that prevents solution leakage?
6. **Qdrant vs. Alternatives**: Is Qdrant the best choice for this use case, or should we consider Pinecone, Weaviate, or pgvector?
7. **Neon Postgres Serverless Limits**: What are the connection pooling and query performance characteristics for Neon's serverless tier?
8. **Robotis OP3 URDF Availability**: Is the OP3 URDF officially maintained and compatible with ROS 2 Humble + MoveIt 2?

### Research Agents

I'll now dispatch research agents to resolve these questions.

