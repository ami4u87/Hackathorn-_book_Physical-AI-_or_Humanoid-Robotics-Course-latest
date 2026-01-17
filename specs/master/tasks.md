# Tasks: Physical AI & Humanoid Robotics – Capstone Quarter

**Input**: Design documents from `/specs/master/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅, quickstart.md ✅

**Tests**: Tests are OPTIONAL unless explicitly requested. This task list focuses on implementation.

**Organization**: Tasks are organized by user story to enable independent implementation and testing. User stories are ordered by priority (P1 → P2).

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Exact file paths included in descriptions

## Path Conventions

Based on plan.md project structure:
- **Book content**: `docs/` (Docusaurus)
- **Frontend components**: `src/components/`, `src/css/`, `src/pages/`
- **Backend chatbot**: `backend/chatbot/src/`
- **Backend API proxy**: `backend/api-proxy/src/`
- **Shared utilities**: `backend/shared/`
- **ROS 2 workspace**: `ros2_workspace/src/`
- **Exercise validation**: `exercises/`
- **Robot models**: `models/`
- **Infrastructure**: `docker/`, `scripts/`, `.github/workflows/`

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Initialize repository structure, dependencies, and configuration

- [ ] T001 Create repository root structure per plan.md (docs/, backend/, ros2_workspace/, docker/, scripts/, exercises/, models/)
- [ ] T002 [P] Initialize Docusaurus project with React 18 in repository root (package.json, docusaurus.config.js, sidebars.js)
- [ ] T003 [P] Create Python virtual environment and requirements.txt for backend services
- [ ] T004 [P] Create .env.example with required environment variables (OPENAI_API_KEY, QDRANT_URL, DATABASE_URL, JWT_SECRET)
- [ ] T005 [P] Create .gitignore for Python, Node.js, ROS 2 workspace artifacts
- [ ] T006 Initialize docker/docker-compose.yml with Qdrant, chatbot, api-proxy, validator services
- [ ] T007 [P] Create docker/Dockerfile.ros2 for ROS 2 Humble development environment
- [ ] T008 [P] Create docker/Dockerfile.chatbot for chatbot FastAPI service
- [ ] T009 [P] Create docker/Dockerfile.api-proxy for API proxy FastAPI service

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story implementation

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database & Storage Infrastructure

- [ ] T010 Create backend/shared/schema.sql with Postgres tables (students, chatbot_queries, api_usage_logs, exercise_submissions)
- [ ] T011 [P] Create backend/shared/db.py with asyncpg connection pool for Neon Postgres
- [ ] T012 [P] Create scripts/init_database.py to run schema migrations
- [ ] T013 [P] Create scripts/init_qdrant.py to initialize book_chunks collection (1536-dim vectors, cosine distance)

### Backend Framework Setup

- [ ] T014 [P] Create backend/chatbot/src/config.py with Pydantic Settings for environment configuration
- [ ] T015 [P] Create backend/api-proxy/src/config.py with Pydantic Settings for proxy configuration
- [ ] T016 [P] Create backend/shared/logging_config.py with structured logging configuration
- [ ] T017 Create backend/chatbot/src/main.py with FastAPI app skeleton, CORS middleware, and health endpoint
- [ ] T018 Create backend/api-proxy/src/main.py with FastAPI app skeleton, CORS middleware, and health endpoint

### Pydantic Models (from data-model.md and contracts/)

- [ ] T019 [P] Create backend/chatbot/src/models/query.py with QueryRequest, QueryResponse, Citation, ChunkInfo, ErrorResponse schemas
- [ ] T020 [P] Create backend/chatbot/src/models/chunk.py with Chunk, ChunkMetadata schemas for RAG retrieval
- [ ] T021 [P] Create backend/chatbot/src/models/feedback.py with FeedbackRequest schema
- [ ] T022 [P] Create backend/api-proxy/src/models/auth.py with TokenRequest, TokenResponse, UsageResponse schemas
- [ ] T023 [P] Create backend/api-proxy/src/models/proxy.py with ChatCompletionRequest, EmbeddingRequest schemas

### Authentication & Rate Limiting

- [ ] T024 Create backend/api-proxy/src/auth.py with JWT token validation and generation using python-jose
- [ ] T025 Create backend/api-proxy/src/rate_limiter.py with slowapi configuration (100 requests/day per student)
- [ ] T026 Create backend/api-proxy/src/usage_logger.py with async Postgres INSERT for API usage logging

### CI/CD Infrastructure

- [ ] T027 [P] Create .github/workflows/build-book.yml for Docusaurus deployment to GitHub Pages
- [ ] T028 [P] Create .github/workflows/test-examples.yml for ROS 2 code example testing in Docker
- [ ] T029 [P] Create .github/workflows/test-chatbot.yml for chatbot accuracy benchmarks

**Checkpoint**: Foundation ready - user story implementation can begin

---

## Phase 3: User Story 8 - Reproducible Code Examples (Priority: P1) 🎯 MVP

**Goal**: Students can clone repository, follow setup instructions, and run all code examples without modification

**Independent Test**: Student runs `docker-compose up`, follows README, executes test script, and receives 100% pass rate on all examples

**Why First**: Reproducibility is the foundation for all other user stories; students cannot learn if examples don't work

### Implementation for User Story 8

- [ ] T030 [US8] Create README.md with project overview, prerequisites, and quick start instructions
- [ ] T031 [P] [US8] Create docker/docker-compose.dev.yml with development overrides for local testing
- [ ] T032 [P] [US8] Create scripts/setup_dev_env.sh for automated developer environment setup
- [ ] T033 [US8] Create ros2_workspace/src/.gitkeep and initialize colcon workspace structure
- [ ] T034 [US8] Create scripts/validate_all_examples.sh to run all ROS 2 code examples and report pass/fail

**Checkpoint**: Repository cloneable, Docker environment works, basic ROS 2 workspace ready

---

## Phase 4: User Story 1 - ROS 2 Fundamentals Module (Priority: P1) 🎯 MVP

**Goal**: Student completes Module 1 and understands ROS 2 core concepts (pub/sub, services, actions, parameters, launch files, TF2)

**Independent Test**: Student installs ROS 2 Humble, executes provided code examples, observes expected outputs, and passes module exercises

### Module 1 Content (docs/module-1-ros2/)

- [ ] T035 [US1] Create docs/module-1-ros2/index.md with module overview and learning objectives
- [ ] T036 [P] [US1] Create docs/module-1-ros2/1-installation.md with ROS 2 Humble installation guide for Ubuntu 22.04
- [ ] T037 [P] [US1] Create docs/module-1-ros2/2-pubsub.md with publisher-subscriber theory, diagrams, and code walkthrough
- [ ] T038 [P] [US1] Create docs/module-1-ros2/3-services.md with service server/client theory and implementation
- [ ] T039 [P] [US1] Create docs/module-1-ros2/4-actions.md with action server/client theory and implementation
- [ ] T040 [P] [US1] Create docs/module-1-ros2/5-parameters.md with parameter server theory and dynamic reconfiguration
- [ ] T041 [P] [US1] Create docs/module-1-ros2/6-launch.md with launch file syntax (Python launch API) and composition
- [ ] T042 [P] [US1] Create docs/module-1-ros2/7-tf2.md with coordinate frame transformations and TF2 usage
- [ ] T043 [US1] Create docs/module-1-ros2/exercises.md with exercise prompts, acceptance criteria, and validation instructions

### ROS 2 Code Examples (ros2_workspace/src/module1_examples/)

- [ ] T044 [P] [US1] Create ros2_workspace/src/module1_examples/publisher_example/ package with publisher_node.py
- [ ] T045 [P] [US1] Create ros2_workspace/src/module1_examples/subscriber_example/ package with subscriber_node.py
- [ ] T046 [P] [US1] Create ros2_workspace/src/module1_examples/service_example/ package with service_server.py and service_client.py
- [ ] T047 [P] [US1] Create ros2_workspace/src/module1_examples/action_example/ package with action_server.py and action_client.py
- [ ] T048 [P] [US1] Create ros2_workspace/src/module1_examples/param_example/ package with param_node.py demonstrating parameters
- [ ] T049 [US1] Create ros2_workspace/src/module1_examples/launch_example/ package with multi-node launch file

### Diagrams (static/diagrams/)

- [ ] T050 [P] [US1] Create static/diagrams/ros2-pubsub-architecture.drawio with publisher-subscriber flow diagram
- [ ] T051 [P] [US1] Create static/diagrams/ros2-service-pattern.drawio with service call sequence diagram
- [ ] T052 [P] [US1] Create static/diagrams/ros2-action-pattern.drawio with action goal/feedback/result flow
- [ ] T053 [P] [US1] Create static/diagrams/ros2-tf2-tree.drawio with coordinate frame tree example

### Exercise Validation Scripts (exercises/module1/)

- [ ] T054 [P] [US1] Create exercises/module1/ex1_pubsub/validate.py with behavioral check for publisher-subscriber exercise
- [ ] T055 [P] [US1] Create exercises/module1/ex2_service/validate.py with behavioral check for service exercise
- [ ] T056 [P] [US1] Create exercises/module1/ex3_params/validate.py with behavioral check for parameter exercise
- [ ] T057 [US1] Create exercises/module1/solution_hashes.json with expected SHA-256 hashes for all exercises (instructor only)

### Docusaurus Configuration for Module 1

- [ ] T058 [US1] Update sidebars.js to include Module 1 navigation structure
- [ ] T059 [US1] Update docusaurus.config.js with site metadata, theme configuration, and search plugin

**Checkpoint**: Module 1 complete - student can work through ROS 2 fundamentals independently

---

## Phase 5: User Story 7 - RAG Chatbot for Student Support (Priority: P2)

**Goal**: Instructor or student queries embedded chatbot for technical explanations, receives grounded answers with citations

**Independent Test**: User asks "How do I configure a ROS 2 service in Python?", receives answer citing Module 1 Section 3 with clickable link

### RAG Chunking & Ingestion Pipeline

- [ ] T060 [US7] Create scripts/ingest_book_content.py to chunk Markdown files (750 tokens, 100 overlap, section boundaries)
- [ ] T061 [US7] Add tiktoken token counting and section boundary detection to ingest script
- [ ] T062 [US7] Add OpenAI text-embedding-3-large embedding generation to ingest script
- [ ] T063 [US7] Add Qdrant upsert with metadata (module, section, heading, page_range, code_language) to ingest script

### RAG Retrieval Service

- [ ] T064 [US7] Create backend/chatbot/src/services/embedder.py with OpenAI embeddings wrapper (query vectorization)
- [ ] T065 [US7] Create backend/chatbot/src/services/retriever.py with Qdrant search (top-5 chunks, metadata filtering)
- [ ] T066 [US7] Create backend/chatbot/src/services/grounding.py with citation validation and grounding enforcement

### Chatbot API Endpoints (from contracts/chatbot-api.yaml)

- [ ] T067 [US7] Create backend/chatbot/src/api/routes.py with POST /query endpoint (QueryRequest → QueryResponse)
- [ ] T068 [US7] Add POST /feedback endpoint to routes.py (FeedbackRequest → success response)
- [ ] T069 [US7] Integrate rate limiting middleware with chatbot API (100 requests/day per student)
- [ ] T070 [US7] Add chatbot query logging to Neon Postgres (chatbot_queries table)

### Chatbot UI Component

- [ ] T071 [US7] Create src/components/ChatbotEmbed.tsx with lazy-loaded iframe and intersection observer
- [ ] T072 [US7] Create src/css/chatbot.css with floating widget styling and mobile modal view
- [ ] T073 [US7] Update docs/module-1-ros2/*.md files to include ChatbotEmbed component on each page

### Ground-Truth Test Set for Module 1

- [ ] T074 [US7] Create scripts/generate_ground_truth.py to create Q&A test pairs from book content
- [ ] T075 [US7] Create tests/module1_qa.json with 20+ ground-truth Q&A pairs for Module 1
- [ ] T076 [US7] Create scripts/test_chatbot_accuracy.py to run accuracy benchmarks against ground-truth

**Checkpoint**: Chatbot functional - answers Module 1 questions with citations, ≥90% accuracy on test set

---

## Phase 6: User Story 2 - Robot Simulation (Priority: P1)

**Goal**: Student sets up Gazebo/Unity/Isaac simulation environments and validates physics interactions with humanoid robot

**Independent Test**: Student launches URDF in Gazebo, teleoperates joints via ROS 2 topics, observes realistic physics

### Robot Model (models/robotis_op3/)

- [ ] T077 [P] [US2] Create models/robotis_op3/urdf/robotis_op3.urdf adapted from community ROS 2 port
- [ ] T078 [P] [US2] Create models/robotis_op3/meshes/ directory with collision and visual mesh files
- [ ] T079 [P] [US2] Create models/robotis_op3/config/controllers.yaml for ros2_control joint controllers
- [ ] T080 [US2] Validate URDF in Gazebo Harmonic (gravity, collision, joint limits)

### Module 2 Content (docs/module-2-simulation/)

- [ ] T081 [US2] Create docs/module-2-simulation/index.md with module overview and simulator comparison
- [ ] T082 [P] [US2] Create docs/module-2-simulation/1-gazebo.md with Gazebo Harmonic setup and ROS 2 integration
- [ ] T083 [P] [US2] Create docs/module-2-simulation/2-unity.md with Unity ROS-TCP-Connector setup (optional track)
- [ ] T084 [P] [US2] Create docs/module-2-simulation/3-isaac.md with NVIDIA Isaac Sim setup (optional, RTX required)
- [ ] T085 [US2] Create docs/module-2-simulation/4-comparison.md with simulator tradeoffs and selection guide
- [ ] T086 [US2] Create docs/module-2-simulation/exercises.md with simulation exercises and validation

### ROS 2 Simulation Examples (ros2_workspace/src/module2_examples/)

- [ ] T087 [P] [US2] Create ros2_workspace/src/module2_examples/op3_gazebo/ package with Gazebo launch file
- [ ] T088 [P] [US2] Create ros2_workspace/src/module2_examples/op3_teleop/ package with joint teleoperation node
- [ ] T089 [US2] Create ros2_workspace/src/module2_examples/op3_description/ package with robot_state_publisher

### Diagrams for Module 2

- [ ] T090 [P] [US2] Create static/diagrams/simulator-architecture.drawio comparing Gazebo/Unity/Isaac
- [ ] T091 [P] [US2] Create static/diagrams/ros2-gazebo-bridge.drawio showing ROS 2 ↔ Gazebo communication

### Docusaurus Updates

- [ ] T092 [US2] Update sidebars.js to include Module 2 navigation
- [ ] T093 [US2] Re-run scripts/ingest_book_content.py --module module-2-simulation for chatbot

**Checkpoint**: Module 2 complete - student can simulate humanoid in Gazebo

---

## Phase 7: User Story 3 & 4 - Perception & Motion Planning (Priority: P2)

**Goal**: Student implements perception pipeline (object detection, pose estimation) and motion planning (MoveIt 2)

**Independent Test**: Student runs perception node on simulated camera, publishes poses to TF; plans collision-free trajectory with MoveIt 2

### Module 3 Content (docs/module-3-perception/)

- [ ] T094 [US3] Create docs/module-3-perception/index.md with perception pipeline overview
- [ ] T095 [P] [US3] Create docs/module-3-perception/1-rgbd-cameras.md with simulated camera setup
- [ ] T096 [P] [US3] Create docs/module-3-perception/2-object-detection.md with YOLO integration
- [ ] T097 [P] [US3] Create docs/module-3-perception/3-pose-estimation.md with depth fusion and TF publishing
- [ ] T098 [US3] Create docs/module-3-perception/4-moveit2.md with MoveIt 2 configuration for OP3
- [ ] T099 [US3] Create docs/module-3-perception/exercises.md with perception and planning exercises

### ROS 2 Perception Examples (ros2_workspace/src/module3_examples/)

- [ ] T100 [P] [US3] Create ros2_workspace/src/module3_examples/perception_pipeline/ package with YOLO detection node
- [ ] T101 [P] [US3] Create ros2_workspace/src/module3_examples/pose_publisher/ package with TF2 pose broadcaster
- [ ] T102 [US3] Create models/robotis_op3/config/moveit2/ directory with MoveIt 2 configuration

### Diagrams for Module 3

- [ ] T103 [P] [US3] Create static/diagrams/perception-pipeline.drawio showing RGB-D → detection → TF flow
- [ ] T104 [P] [US3] Create static/diagrams/moveit2-architecture.drawio showing planning scene and execution

### Docusaurus Updates

- [ ] T105 [US3] Update sidebars.js to include Module 3 navigation
- [ ] T106 [US3] Re-run scripts/ingest_book_content.py --module module-3-perception for chatbot

**Checkpoint**: Modules 3-4 complete - student can run perception and plan motions

---

## Phase 8: User Story 5 - VLA Integration (Priority: P2)

**Goal**: Student connects VLA model (GPT-4V) to interpret natural language commands and generate action plans

**Independent Test**: Student sends "pick up the red mug" command, receives structured JSON action plan

### API Proxy Service (from contracts/api-proxy.yaml)

- [ ] T107 [US5] Create backend/api-proxy/src/proxy_handler.py with httpx async client for OpenAI forwarding
- [ ] T108 [US5] Implement POST /v1/chat/completions endpoint in backend/api-proxy/src/api/routes.py
- [ ] T109 [US5] Implement POST /v1/embeddings endpoint in backend/api-proxy/src/api/routes.py
- [ ] T110 [US5] Implement POST /auth/token endpoint for JWT issuance (instructor auth)
- [ ] T111 [US5] Implement GET /usage/{student_id} endpoint for usage statistics

### Module 4 Content (docs/module-4-vla/)

- [ ] T112 [US5] Create docs/module-4-vla/index.md with VLA concepts and architecture overview
- [ ] T113 [P] [US5] Create docs/module-4-vla/1-speech-to-text.md with Whisper API integration
- [ ] T114 [P] [US5] Create docs/module-4-vla/2-vision-language.md with GPT-4V API usage patterns
- [ ] T115 [US5] Create docs/module-4-vla/3-action-planning.md with structured action plan generation
- [ ] T116 [US5] Create docs/module-4-vla/4-ros2-integration.md with action plan → ROS 2 execution bridge
- [ ] T117 [US5] Create docs/module-4-vla/exercises.md with VLA integration exercises

### ROS 2 VLA Examples (ros2_workspace/src/module4_examples/)

- [ ] T118 [P] [US5] Create ros2_workspace/src/module4_examples/speech_node/ package with Whisper transcription
- [ ] T119 [P] [US5] Create ros2_workspace/src/module4_examples/vla_planner/ package with GPT-4V action planning
- [ ] T120 [US5] Create ros2_workspace/src/module4_examples/action_executor/ package with plan → motion execution

### Diagrams for Module 4

- [ ] T121 [P] [US5] Create static/diagrams/vla-architecture.drawio showing speech → VLA → action flow
- [ ] T122 [P] [US5] Create static/diagrams/api-proxy-architecture.drawio showing JWT auth and rate limiting

### Docusaurus Updates

- [ ] T123 [US5] Update sidebars.js to include Module 4 navigation
- [ ] T124 [US5] Re-run scripts/ingest_book_content.py --module module-4-vla for chatbot

**Checkpoint**: Module 4 complete - student can use VLA for action planning

---

## Phase 9: User Story 6 - Capstone Integration (Priority: P1)

**Goal**: Student integrates all modules into voice-commanded autonomous manipulation task

**Independent Test**: Student speaks "place the blue block in the bin", robot completes task in simulation with video proof

### Module 5 Content (docs/module-5-capstone/)

- [ ] T125 [US6] Create docs/module-5-capstone/index.md with capstone project overview and requirements
- [ ] T126 [US6] Create docs/module-5-capstone/1-system-architecture.md with full pipeline integration diagram
- [ ] T127 [US6] Create docs/module-5-capstone/2-starter-code.md with provided capstone starter package overview
- [ ] T128 [US6] Create docs/module-5-capstone/3-integration-guide.md with step-by-step integration instructions
- [ ] T129 [US6] Create docs/module-5-capstone/4-evaluation.md with success criteria and grading rubric

### ROS 2 Capstone Starter Code (ros2_workspace/src/module5_capstone/)

- [ ] T130 [US6] Create ros2_workspace/src/module5_capstone/capstone_bringup/ package with full system launch file
- [ ] T131 [US6] Create ros2_workspace/src/module5_capstone/capstone_perception/ package with object detection starter
- [ ] T132 [US6] Create ros2_workspace/src/module5_capstone/capstone_planner/ package with VLA integration starter
- [ ] T133 [US6] Create ros2_workspace/src/module5_capstone/capstone_control/ package with MoveIt 2 execution starter
- [ ] T134 [US6] Create ros2_workspace/src/module5_capstone/worlds/ directory with capstone Gazebo world (table, blocks, bin)

### Diagrams for Module 5

- [ ] T135 [P] [US6] Create static/diagrams/capstone-architecture.drawio with full system integration diagram
- [ ] T136 [P] [US6] Create static/diagrams/capstone-message-flow.drawio showing ROS 2 topic/service/action flow

### Docusaurus Updates

- [ ] T137 [US6] Update sidebars.js to include Module 5 navigation
- [ ] T138 [US6] Re-run scripts/ingest_book_content.py --module module-5-capstone for chatbot

**Checkpoint**: Capstone complete - student can run voice-commanded autonomous task

---

## Phase 10: Exercise Validator Service (Priority: P2)

**Goal**: Server-side exercise validation with LMS integration

**Independent Test**: Student submits validation hash, receives is_correct response and grade recorded

### Validator API (from contracts/validator-api.yaml)

- [ ] T139 Create backend/validator/src/main.py with FastAPI app for exercise validation
- [ ] T140 Create backend/validator/src/api/routes.py with POST /validate/submit and GET /validate/status endpoints
- [ ] T141 Create backend/validator/src/models/validation.py with ValidationSubmission, ValidationResponse schemas
- [ ] T142 Add exercise_submissions table operations to backend/shared/db.py
- [ ] T143 Create docker/Dockerfile.validator for validator service container

**Checkpoint**: Validator service deployed - exercise grading functional

---

## Phase 11: Polish & Cross-Cutting Concerns

**Purpose**: Documentation, accessibility, and final validation

### Documentation & Accessibility

- [ ] T144 [P] Create docs/intro.md with course introduction and learning path
- [ ] T145 [P] Create docs/glossary.md with terminology definitions per constitution
- [ ] T146 Add alt text to all images in docs/ for WCAG 2.1 AA compliance
- [ ] T147 Verify keyboard navigation for chatbot widget (ARIA labels, focus management)

### Content Completeness

- [ ] T148 [P] Compile bibliography (≥40 sources, ≥50% peer-reviewed) in docs/references.md
- [ ] T149 Verify word count targets: 8,000-12,000 words per module, 40,000-60,000 total
- [ ] T150 Run automated link validation across all Markdown files

### Chatbot Validation

- [ ] T151 Create ground-truth Q&A test sets for Modules 2-5 (20 questions each)
- [ ] T152 Run chatbot accuracy benchmark on full 100-question test set (target: ≥90%)
- [ ] T153 Human review 20 random responses for hallucination detection

### CI/CD Validation

- [ ] T154 Run full CI/CD pipeline: build book, test ROS 2 examples, benchmark chatbot
- [ ] T155 Validate quickstart.md instructions with clean Docker environment
- [ ] T156 Security review: no hardcoded secrets, proper JWT rotation

### Beta Testing Preparation

- [ ] T157 Recruit 5 beta testers (CS/AI students or faculty)
- [ ] T158 Create beta feedback survey (exercise difficulty, chatbot helpfulness)
- [ ] T159 Deploy Phase 1 (Module 1 + Chatbot v1) to GitHub Pages for beta testing

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 - BLOCKS all user stories
- **Phase 3 (US8: Reproducibility)**: Depends on Phase 2 - enables subsequent story work
- **Phase 4 (US1: ROS 2 Module)**: Depends on Phase 2, 3 - first content module
- **Phase 5 (US7: Chatbot)**: Depends on Phase 2, 4 - needs Module 1 content to ingest
- **Phase 6 (US2: Simulation)**: Depends on Phase 4 - builds on ROS 2 fundamentals
- **Phase 7 (US3/4: Perception)**: Depends on Phase 6 - uses simulation environment
- **Phase 8 (US5: VLA)**: Depends on Phase 5 - uses API proxy infrastructure
- **Phase 9 (US6: Capstone)**: Depends on Phase 6, 7, 8 - integrates all modules
- **Phase 10 (Validator)**: Can proceed in parallel with Phases 6-9
- **Phase 11 (Polish)**: Depends on all user stories complete

### User Story Dependencies

```
                    ┌─────────────────┐
                    │  Phase 1: Setup │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │Phase 2: Foundation│
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
     ┌────────▼───────┐ ┌────▼─────┐ ┌──────▼─────┐
     │US8: Reproducible│ │US7: Chat │ │US10: Valid.│
     │ (Phase 3)       │ │(Phase 5) │ │(Phase 10)  │
     └────────┬───────┘ └────┬─────┘ └────────────┘
              │              │
     ┌────────▼───────┐      │ (needs content)
     │US1: ROS 2 Mod1 │──────┘
     │ (Phase 4)      │
     └────────┬───────┘
              │
     ┌────────▼───────┐
     │US2: Simulation │
     │ (Phase 6)      │
     └────────┬───────┘
              │
     ┌────────▼───────┐
     │US3/4: Perception│
     │ (Phase 7)      │
     └───┬────────┬───┘
         │        │
         │  ┌─────▼─────┐
         │  │US5: VLA   │
         │  │(Phase 8)  │
         │  └─────┬─────┘
         │        │
     ┌───▼────────▼───┐
     │US6: Capstone   │
     │ (Phase 9)      │
     └────────────────┘
```

### Parallel Opportunities

**Within Setup (Phase 1)**:
- T002, T003, T004, T005 can run in parallel
- T007, T008, T009 can run in parallel

**Within Foundational (Phase 2)**:
- T011, T012, T013 can run in parallel
- T014, T015, T016 can run in parallel
- T019, T020, T021, T022, T023 can run in parallel
- T027, T028, T029 can run in parallel

**Within User Story 1 (Phase 4)**:
- T036, T037, T038, T039, T040, T041, T042 (content) can run in parallel
- T044, T045, T046, T047, T048 (ROS 2 packages) can run in parallel
- T050, T051, T052, T053 (diagrams) can run in parallel
- T054, T055, T056 (validation scripts) can run in parallel

**Cross-Phase Parallelism**:
- Phase 10 (Validator) can proceed in parallel with Phases 6-9
- Once Phase 4 completes, US7 (Chatbot) can start while US2 (Simulation) proceeds

---

## Parallel Execution Examples

### Example 1: Launch Phase 2 Infrastructure in Parallel

```bash
# All database tasks in parallel:
Task: "Create backend/shared/db.py with asyncpg connection pool"
Task: "Create scripts/init_database.py to run schema migrations"
Task: "Create scripts/init_qdrant.py to initialize book_chunks collection"
```

### Example 2: Launch Module 1 Content in Parallel

```bash
# All Module 1 sections in parallel:
Task: "Create docs/module-1-ros2/1-installation.md"
Task: "Create docs/module-1-ros2/2-pubsub.md"
Task: "Create docs/module-1-ros2/3-services.md"
Task: "Create docs/module-1-ros2/4-actions.md"
Task: "Create docs/module-1-ros2/5-parameters.md"
Task: "Create docs/module-1-ros2/6-launch.md"
Task: "Create docs/module-1-ros2/7-tf2.md"
```

### Example 3: Launch ROS 2 Packages in Parallel

```bash
# All Module 1 ROS 2 packages in parallel:
Task: "Create ros2_workspace/src/module1_examples/publisher_example/"
Task: "Create ros2_workspace/src/module1_examples/subscriber_example/"
Task: "Create ros2_workspace/src/module1_examples/service_example/"
Task: "Create ros2_workspace/src/module1_examples/action_example/"
Task: "Create ros2_workspace/src/module1_examples/param_example/"
```

---

## Implementation Strategy

### MVP First (Phase 1-5 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: US8 (Reproducibility)
4. Complete Phase 4: US1 (Module 1: ROS 2 Fundamentals)
5. Complete Phase 5: US7 (Chatbot trained on Module 1)
6. **STOP and VALIDATE**: Deploy to GitHub Pages, beta test with 5 students

**MVP Scope**: ~76 tasks (T001-T076)

### Incremental Delivery After MVP

1. Phase 6: US2 (Module 2: Simulation) → Deploy
2. Phase 7: US3/4 (Module 3: Perception & Control) → Deploy
3. Phase 8: US5 (Module 4: VLA Integration) → Deploy
4. Phase 9: US6 (Module 5: Capstone) → Deploy
5. Phase 10: Validator Service (parallel with above)
6. Phase 11: Polish & Cross-Cutting Concerns → Final Release

### Parallel Team Strategy

With 3 developers after MVP validation:
- **Developer A**: Modules 2-3 content (US2, US3/4)
- **Developer B**: Modules 4-5 content (US5, US6)
- **Developer C**: Chatbot enhancements + Validator (US7, US10)

---

## Summary

| Metric | Count |
|--------|-------|
| **Total Tasks** | 159 |
| **Setup Tasks** | 9 |
| **Foundational Tasks** | 20 |
| **US1 (ROS 2 Module) Tasks** | 25 |
| **US2 (Simulation) Tasks** | 17 |
| **US3/4 (Perception) Tasks** | 13 |
| **US5 (VLA) Tasks** | 18 |
| **US6 (Capstone) Tasks** | 14 |
| **US7 (Chatbot) Tasks** | 17 |
| **US8 (Reproducibility) Tasks** | 5 |
| **Validator Tasks** | 5 |
| **Polish Tasks** | 16 |
| **MVP Tasks (Phases 1-5)** | 76 |
| **Parallel Opportunities** | 68 tasks marked [P] |

---

## Notes

- [P] tasks = different files, no dependencies - run in parallel
- [Story] label maps task to user story for traceability
- Each user story independently testable per spec.md acceptance scenarios
- Commit after each task or logical group
- Stop at any phase checkpoint to validate independently
- Chatbot accuracy validation required after each module ingestion
- All code examples must pass CI/CD before module can be marked complete
