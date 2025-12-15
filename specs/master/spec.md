# Feature Specification: Physical AI & Humanoid Robotics – Capstone Quarter

**Feature Branch**: `master`
**Created**: 2025-12-15
**Status**: Draft
**Input**: User description: "Physical AI & Humanoid Robotics – Capstone Quarter

Target audience:
Senior CS/AI students learning embodied intelligence and humanoid robotics.

Focus:
Designing, simulating, and controlling humanoid robots using ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems.

Success criteria:
- Students understand Physical AI and embodied intelligence
- Each module delivers working, simulation-based outcomes
- Students can integrate perception, planning, and action
- Capstone humanoid completes a voice-driven autonomous task

Constraints:
- Format: Markdown (Docusaurus)
- Embedded RAG chatbot (grounded, zero hallucination)
- 40k–60k words total; 5 modules
- All code examples tested and reproducible
- ≥40 sources; ≥50% peer-reviewed (APA 7)
- Modules: ROS 2 → Simulation → Perception/Control → VLA → Capstone
- Stack: ROS 2 Humble, Gazebo/Unity/Isaac, Python 3.11+, FastAPI, OpenAI Agents, Neon Postgres, Qdrant
- Deployment: GitHub Pages

Deliverables:
1. Docusaurus book (5 modules, each with theory/diagrams/code/exercises)
2. Embedded RAG chatbot (answers only from book content, citations required)
3. Capstone: Humanoid robot simulation executing voice-commanded autonomous task (perception → planning → manipulation)
4. Test suite validating chatbot ≥90% accuracy on ground-truth Q&A
5. All code examples runnable as-is with clear setup instructions"

## Clarifications

### Session 2025-12-15

- Q: What is the implementation and deployment sequencing strategy for modules and chatbot? → A: Progressive staged deployment: Module 1 → Chatbot v1 (trained on M1 only) → Modules 2-5 → Full chatbot
- Q: Which specific humanoid robot model will be used throughout the course modules and capstone? → A: Robotis OP3 (open-source, 20 DOF, well-documented ROS 2 integration)
- Q: How will OpenAI API access and costs be managed for student use (VLA and chatbot)? → A: Instructor-managed proxy with rate limiting (students use shared endpoint with individual quotas; 100 requests/day per student)
- Q: What is the specific RAG chunking strategy (token size, overlap, boundary handling)? → A: 750-token chunks with 100-token overlap; split on section/subsection boundaries where possible to preserve semantic coherence
- Q: How do students validate exercise correctness and submit for grading? → A: Automated validation scripts provided; students run locally for immediate pass/fail feedback; optionally submit validation hash to LMS for grade tracking

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Completes ROS 2 Fundamentals Module (Priority: P1)

A senior CS/AI student with Python programming experience but no robotics background works through Module 1 to understand ROS 2 core concepts and write their first functioning robot nodes.

**Why this priority**: Foundation module enabling all subsequent work; students cannot progress without mastering pub/sub, services, actions, and parameters.

**Independent Test**: Student can install ROS 2 Humble, execute provided code examples, observe expected terminal outputs, and pass module exercises testing node creation, topic communication, and service calls.

**Acceptance Scenarios**:

1. **Given** ROS 2 Humble installed on Ubuntu 22.04, **When** student executes publisher example code, **Then** subscriber node receives and prints messages with correct topic names and data types
2. **Given** module theory sections read, **When** student writes a custom service server following the pattern, **Then** client can call service and receive expected response
3. **Given** exercise prompts provided, **When** student implements parameter-based node configuration, **Then** node behavior changes correctly when parameters are updated via command line

---

### User Story 2 - Student Simulates Robot in Gazebo/Unity/Isaac (Priority: P1)

Student sets up simulation environments for a humanoid robot and validates physics-based interactions (gravity, collisions, joint control).

**Why this priority**: Simulation is prerequisite for safe development; all code must be tested in simulation before hardware deployment per safety-first constitution.

**Independent Test**: Student launches provided URDF/USD robot model in each simulator, teleoperates joints via ROS 2 topics, and observes realistic physics behavior (falling, collision response, joint limits).

**Acceptance Scenarios**:

1. **Given** Gazebo Harmonic installed with ROS 2 bridge, **When** student launches humanoid URDF with `ros2 launch`, **Then** robot spawns, remains upright under gravity, and joint commands move limbs smoothly
2. **Given** Unity with ROS-TCP-Connector configured, **When** student imports robot model and runs simulation, **Then** sensor data (cameras, IMU) publishes to ROS 2 topics at expected rates
3. **Given** NVIDIA Isaac Sim with Omniverse, **When** student loads USD humanoid and applies torque commands, **Then** Isaac physics engine produces collision responses and robot maintains balance control

---

### User Story 3 - Student Integrates Perception Pipeline (Priority: P2)

Student implements vision and depth perception for object detection and pose estimation, feeding into robot planning systems.

**Why this priority**: Perception is critical for autonomous behavior; required for capstone but builds on simulation foundation.

**Independent Test**: Student runs perception node on simulated RGB-D camera stream, detects objects in scene, publishes poses to TF tree, and visualizes in RViz2.

**Acceptance Scenarios**:

1. **Given** RGB-D camera in Gazebo publishing images and depth, **When** student runs YOLO-based detection node, **Then** bounding boxes and class labels output for objects in scene
2. **Given** detected object centroids in image space, **When** depth data fused with camera intrinsics, **Then** 3D poses published to `/tf` with correct transformation from camera frame to object frame
3. **Given** multiple objects in scene, **When** perception pipeline filters by class and distance, **Then** only relevant objects (e.g., graspable items within 1m) forwarded to planner

---

### User Story 4 - Student Implements Motion Planning and Control (Priority: P2)

Student uses MoveIt 2 or custom planners to generate collision-free trajectories for humanoid manipulation tasks.

**Why this priority**: Motion planning connects perception to action; necessary for capstone task execution.

**Independent Test**: Student commands robot to reach target pose, planner generates trajectory avoiding obstacles, and robot executes motion in simulation without collisions.

**Acceptance Scenarios**:

1. **Given** MoveIt 2 configured with humanoid arm kinematics, **When** student requests end-effector pose, **Then** planner computes joint trajectory and robot arm reaches target within 5cm tolerance
2. **Given** obstacle in workspace (simulated table), **When** trajectory planned to grasp object on table, **Then** path avoids collision and grasp pose achieves stable grip
3. **Given** planned trajectory, **When** ros2_control executes joint commands, **Then** simulated robot follows path smoothly with position error <2 degrees per joint

---

### User Story 5 - Student Integrates Vision-Language-Action (VLA) Model (Priority: P2)

Student connects GPT-4 Vision or similar VLA model to interpret natural language commands and visual scenes, outputting high-level action plans.

**Why this priority**: VLA enables voice-driven autonomy for capstone; represents cutting-edge Physical AI integration.

**Independent Test**: Student sends voice command via API, VLA model processes camera image and text, returns structured action sequence (e.g., "pick red cup, place on shelf"), and student verifies action feasibility.

**Acceptance Scenarios**:

1. **Given** user voice command "pick up the red mug", **When** speech-to-text transcribes and VLA model receives transcription + camera RGB image, **Then** model outputs JSON action plan: `[{"action": "grasp", "object": "red_mug", "pose": [x, y, z]}]`
2. **Given** VLA action plan with grasp pose, **When** planner validates pose reachability, **Then** motion plan generated or fallback error returned if unreachable
3. **Given** ambiguous command "bring me that", **When** VLA processes with no clear object in view, **Then** model requests clarification: "Which object? I see a cup and a book."

---

### User Story 6 - Student Completes Capstone: Voice-Commanded Autonomous Task (Priority: P1)

Student integrates all modules into a capstone demonstration where a simulated humanoid responds to voice commands, perceives environment, plans motion, and executes manipulation tasks autonomously.

**Why this priority**: Validates cumulative learning and demonstrates end-to-end Physical AI system; primary success criterion for course.

**Independent Test**: Student runs full system (speech → VLA → perception → planning → control), issues command "place the blue block in the bin", and robot successfully completes task in simulation with video recording as proof.

**Acceptance Scenarios**:

1. **Given** capstone simulation environment with objects (blocks, bin), **When** student speaks command "put the blue block in the bin", **Then** system executes full pipeline: speech-to-text → VLA plan → object detection → grasp planning → trajectory execution → placement, with block ending in bin
2. **Given** unexpected obstacle introduced (e.g., another object blocking path), **When** capstone system re-plans, **Then** robot avoids new obstacle and completes task or reports infeasibility
3. **Given** task completion, **When** student reviews ROS 2 logs and simulation recording, **Then** all subsystems (perception, planning, control) show expected message flows and no errors

---

### User Story 7 - Instructor Uses RAG Chatbot for Student Support (Priority: P2)

Instructor or student queries embedded chatbot for technical explanations, code clarifications, or exercise hints directly within the Docusaurus book.

**Why this priority**: Enhances learning experience and reduces support burden; differentiates this course material from static textbooks.

**Independent Test**: User asks "How do I configure a ROS 2 service in Python?" in chatbot, receives grounded answer with citation to Module 1 Section 3, and can click link to navigate to source section.

**Acceptance Scenarios**:

1. **Given** chatbot embedded on Module 2 page, **When** user asks "What is the difference between Gazebo Classic and Gazebo Harmonic?", **Then** chatbot retrieves relevant chunks from Module 2, answers with citations, and provides no information outside book content
2. **Given** user asks "How do I train a neural network for object detection?" (out of scope), **When** chatbot searches vector database, **Then** returns "I don't have information on training object detection models in this book. This book focuses on integrating pre-trained models."
3. **Given** chatbot deployed, **When** tested on 100 ground-truth Q&A pairs, **Then** achieves ≥90% correctness (F1 ≥ 0.8) with zero hallucinations detected in random 20-sample human review

---

### User Story 8 - Student Runs All Code Examples Reproducibly (Priority: P1)

Student follows setup instructions, installs dependencies, and executes every code example in the book without modification, observing documented outputs.

**Why this priority**: Reproducibility is core constitution principle; broken examples undermine trust and learning.

**Independent Test**: Student clones companion repository, follows README setup (Docker or native), runs test script validating all 50+ examples, and receives 100% pass rate.

**Acceptance Scenarios**:

1. **Given** Docker Compose file provided, **When** student runs `docker-compose up`, **Then** ROS 2 Humble, Gazebo, and dependencies install correctly and container is ready for examples
2. **Given** Module 3 perception example script, **When** student runs `python3 detect_objects.py --config config.yaml`, **Then** script outputs bounding boxes matching expected test image results documented in module
3. **Given** all code examples in repository, **When** CI/CD pipeline runs automated tests, **Then** all integration tests pass on Ubuntu 22.04 with ROS 2 Humble

---

### Edge Cases

- What happens when a student's local ROS 2 installation is Foxy (older LTS) instead of Humble? → Book must specify Humble requirement prominently; chatbot should detect version mismatches in common error messages and suggest upgrade.
- How does the system handle VLA API rate limits or failures? → Code examples must include retry logic and fallback messages; exercises should teach error handling patterns.
- What if a student's GPU cannot run Isaac Sim? → Book provides alternative paths: Gazebo/Unity for students without RTX GPUs; Isaac content marked as optional advanced track.
- How does chatbot respond to questions in languages other than English? → Chatbot system prompt enforces English-only responses; non-English input triggers: "Please ask in English for accurate book citations."
- What if simulation physics diverges from real-world behavior? → Each module includes "Sim-to-Real Considerations" section documenting known gaps (friction models, sensor noise) and mitigation strategies.
- How are exercise solutions provided without enabling copy-paste? → Solutions in separate locked repository requiring instructor access code; students get validation scripts to check correctness without seeing full implementation.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Book MUST provide complete installation instructions for ROS 2 Humble on Ubuntu 22.04, including workspace setup, colcon build process, and environment sourcing
- **FR-002**: Each of 5 modules MUST include: theory sections (2000-3000 words), architecture diagrams, code examples with line-by-line explanations, and exercises with test cases
- **FR-003**: Module 1 MUST cover ROS 2 pub/sub, services, actions, parameters, launch files, and basic TF2 transformations with working Python examples
- **FR-004**: Module 2 MUST demonstrate robot simulation in Gazebo Harmonic, Unity with ROS-TCP-Connector, and NVIDIA Isaac Sim with comparative analysis of each tool's strengths
- **FR-005**: Module 3 MUST implement perception pipelines for RGB-D cameras including object detection (YOLO or similar), pose estimation, and point cloud processing
- **FR-006**: Module 4 MUST integrate a Vision-Language-Action model (GPT-4V or equivalent) for interpreting natural language commands and generating action plans
- **FR-007**: Module 5 (Capstone) MUST provide starter code for a humanoid robot (URDF/USD model, ROS 2 control config) capable of manipulation tasks in simulation
- **FR-008**: Capstone MUST integrate speech-to-text (Whisper or similar), VLA planning, MoveIt 2 motion planning, and ros2_control execution into a cohesive autonomous system
- **FR-009**: All code examples MUST run without modification given documented environment setup; examples must include test commands showing expected output
- **FR-010**: Embedded RAG chatbot MUST answer questions using ONLY book content retrieved from Qdrant vector database, with zero tolerance for hallucinated information
- **FR-011**: Chatbot MUST provide source citations for every factual answer in format: "According to [Module X, Section Y]" with clickable link to source section
- **FR-012**: Chatbot MUST refuse to answer questions outside book scope with template: "I don't have information on [topic] in this book. Try rephrasing or check the Table of Contents."
- **FR-013**: RAG system MUST use OpenAI text-embedding-3-large for vectorization with 750-token chunks and 100-token overlap (split on section boundaries); retrieve top-5 relevant chunks, and use GPT-4 Turbo with strict grounding prompt
- **FR-014**: Book MUST include ≥40 authoritative sources with ≥50% peer-reviewed papers, cited in APA 7 format
- **FR-015**: Book MUST maintain consistent terminology per glossary (e.g., "end effector" not "gripper" unless specifically referring to gripper mechanism)
- **FR-016**: Every exercise MUST include acceptance criteria enabling students to validate correctness (e.g., "subscriber receives 10 messages with correct timestamps")
- **FR-017**: Book MUST include safety warnings for physical robot operations (e.g., "Never deploy untested code to hardware; always test in simulation first")
- **FR-018**: Docusaurus site MUST deploy to GitHub Pages with functional navigation, search, and embedded chatbot iframe on content pages
- **FR-019**: Chatbot backend (FastAPI) MUST handle concurrent requests from multiple students without degradation, with 3-second p95 response time target
- **FR-020**: System MUST log all chatbot queries and responses to Neon Postgres for analytics (anonymized student IDs) and continuous accuracy monitoring
- **FR-021**: Instructor-managed OpenAI API proxy MUST enforce rate limiting (100 requests/day per student) for VLA and chatbot access; proxy tracks usage per student ID and returns quota status in response headers
- **FR-022**: Each exercise MUST include automated validation script (Python or Bash) that students run locally; script outputs pass/fail with specific feedback (e.g., "PASS: Subscriber received 10 messages" or "FAIL: Expected 10 messages, received 7")
- **FR-023**: Exercise validation scripts MUST generate SHA-256 hash of validation results; students optionally submit hash to LMS (Canvas, Moodle) for grade tracking; instructors verify hash matches expected solution hash

### Non-Functional Requirements

- **NFR-001**: Book word count MUST be 40,000–60,000 words total (8,000–12,000 per module)
- **NFR-002**: All code examples MUST pass linting (ruff for Python, clang-tidy for C++) and formatting (black, clang-format) checks
- **NFR-003**: Chatbot accuracy MUST achieve ≥90% correctness on 100-question ground-truth test set (F1 ≥ 0.8 per answer)
- **NFR-004**: Chatbot MUST have zero detected hallucinations in random 20-response human review sample per module
- **NFR-005**: Code examples MUST include type hints for all Python function signatures and docstrings in Google style
- **NFR-006**: Diagrams MUST be created in draw.io or Mermaid for editability and version control; minimum 30 diagrams across all modules
- **NFR-007**: All external dependencies MUST be version-pinned in requirements.txt, package.xml, or Docker files
- **NFR-008**: Simulation examples MUST run on mid-range hardware (16GB RAM, 4-core CPU, GTX 1660 or equivalent); Isaac Sim examples may require RTX GPU but must be marked optional
- **NFR-009**: Book MUST be accessible (WCAG 2.1 AA): alt text for images, semantic HTML headings, keyboard navigation for chatbot
- **NFR-010**: Chatbot embedding MUST not block page load; iframe loads asynchronously with loading spinner

### Key Entities

- **Module**: Represents one of five instructional units (ROS 2 Fundamentals, Simulation, Perception/Control, VLA, Capstone); contains theory, diagrams, code, exercises
- **Code Example**: Standalone Python or C++ script/package demonstrating specific ROS 2 concept; includes setup instructions, expected output, and test validation
- **Exercise**: Learning activity with prompt, acceptance criteria, automated validation script, and hidden solution; students run validation script locally for pass/fail feedback and optionally submit hash for grade tracking
- **RAG Chunk**: Text segment (750 tokens with 100-token overlap) from book content, split on section boundaries to preserve semantic coherence; vectorized with OpenAI text-embedding-3-large and stored in Qdrant with metadata (module, section, heading, page range)
- **Chatbot Query**: User question submitted to RAG system; logged with timestamp, retrieved chunks, generated answer, and feedback (if provided)
- **Simulation Environment**: Gazebo, Unity, or Isaac Sim workspace with robot model (URDF/USD), world file, and ROS 2 launch configuration
- **VLA Action Plan**: Structured JSON output from Vision-Language-Action model specifying high-level actions (grasp, place, navigate) with target objects and poses
- **Humanoid Robot Model**: URDF or USD description of robot kinematics, dynamics, and sensors; includes meshes, joint limits, and control interfaces
- **Ground-Truth Q&A Pair**: Test question with verified correct answer used for chatbot accuracy evaluation; covers factual recall, conceptual understanding, and code interpretation

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All 5 modules published on Docusaurus GitHub Pages site with functional navigation, search, and embedded chatbot
- **SC-002**: Book total word count between 40,000–60,000 words, with each module 8,000–12,000 words
- **SC-003**: ≥50 code examples provided, 100% pass automated integration tests on Ubuntu 22.04 + ROS 2 Humble
- **SC-004**: Embedded RAG chatbot achieves ≥90% accuracy on 100-question ground-truth test set (minimum 90 correct with F1 ≥ 0.8)
- **SC-005**: Zero hallucinations detected in 100-response human review sample (20 random responses per module)
- **SC-006**: ≥40 authoritative sources cited, with ≥50% (20+) peer-reviewed papers, all in APA 7 format
- **SC-007**: Chatbot provides source citation with clickable link for 100% of factual answers
- **SC-008**: Students can complete capstone task (voice-commanded autonomous manipulation) in simulation with provided starter code and instructions
- **SC-009**: Chatbot p95 response time ≤3 seconds under load (10 concurrent users)
- **SC-010**: All diagrams (≥30 total) render correctly in Docusaurus with alt text for accessibility
- **SC-011**: Students complete setup (Docker or native) and run first code example within 30 minutes following README
- **SC-012**: Instructor validation: 5 beta testers (CS/AI students or faculty) confirm exercises are completable and chatbot is helpful (≥4/5 rating)

## Out of Scope

- Hardware procurement guides or robot kit recommendations (focus on simulation and software)
- Custom robot hardware design, CAD modeling, or 3D printing instructions
- Production deployment at scale (course material assumes single-user educational context)
- Real-world robot deployment (sim-to-real transfer discussed conceptually but not implemented)
- Non-ROS frameworks (Robot Operating System is required; other frameworks mentioned only for VLA integration)
- Mobile robot navigation (Nav2 mentioned but not primary focus; emphasis on manipulation)
- Advanced topics: multi-robot coordination, SLAM, reinforcement learning for control (brief mentions acceptable but no deep dives)
- Custom trained models (assumes pre-trained YOLO, VLA models available via API; training pipelines not covered)

## Technical Constraints

- **Platform**: Docusaurus (latest stable) for book; GitHub Pages for hosting
- **ROS Version**: ROS 2 Humble Hawksbill (LTS); examples must not require Foxy or earlier
- **OS**: Ubuntu 22.04 LTS (primary); Windows/macOS support via Docker only
- **Languages**: Python 3.11+ (primary), C++17 (optional performance-critical examples)
- **Simulators**: Gazebo Harmonic (primary), Unity 2022.3+ with ROS-TCP-Connector, NVIDIA Isaac Sim 2023.1+ (optional advanced)
- **Backend**: FastAPI (Python 3.11+), deployed containerized (Docker Compose or Kubernetes)
- **Database**: Neon Postgres (serverless) for metadata/logs; Qdrant (cloud or self-hosted) for vectors
- **Embeddings**: OpenAI text-embedding-3-large (1536 dimensions)
- **LLM**: GPT-4 Turbo or GPT-4o for chatbot responses (with strict grounding system prompt)
- **APIs**: OpenAI Agents API for orchestration, Whisper API for speech-to-text
- **Version Control**: Git with conventional commits; all code in public GitHub repository
- **CI/CD**: GitHub Actions for automated testing of code examples and chatbot accuracy benchmarks

## Dependencies

- ROS 2 Humble installation with core packages (rclpy, tf2, ros2_control)
- MoveIt 2 for motion planning (Humble distribution)
- Gazebo Harmonic with ROS 2 integration (gz_ros2_control)
- Unity 2022.3 LTS with Robotics packages (optional)
- NVIDIA Isaac Sim 2023.1+ (optional, requires RTX GPU)
- OpenAI API access (GPT-4, Whisper, Embeddings)
- Qdrant vector database (cloud or Docker self-hosted)
- Neon Postgres (serverless tier acceptable for development)
- Docker and Docker Compose for environment consistency
- Python packages: fastapi, uvicorn, openai, qdrant-client, psycopg2, pydantic, pytest, python-jose (JWT auth for API proxy), slowapi (rate limiting)
- ROS 2 Python packages: rclpy, geometry_msgs, sensor_msgs, tf2_ros, moveit_py
- Node.js 18+ and npm/yarn for Docusaurus build
- draw.io or mermaid-cli for diagram generation

## Assumptions

- Students have completed introductory CS courses (Python, data structures, algorithms)
- Students have access to Ubuntu 22.04 (native, VM, or Docker)
- Instructor deploys OpenAI API proxy (FastAPI-based) with rate limiting (100 requests/day per student); students authenticate with student ID; proxy logs usage and enforces quotas
- Students have internet access for downloading ROS 2 packages and accessing chatbot API
- Beta testing phase includes at least 5 representative students before public release
- Peer-reviewed sources accessible via university library subscriptions or open access
- Humanoid robot model is Robotis OP3 (20 DOF) with URDF available from official Robotis GitHub repository; USD conversion for Isaac Sim performed by authors if not available
- Chatbot usage analytics comply with FERPA (student data anonymized before logging)

## Risks

- **VLA API costs**: GPT-4V API calls may be expensive for large cohorts → Mitigation: Instructor-managed proxy with 100 requests/day quota per student (estimated $2-5/student/month); caching for repeated queries; offline mode with mock VLA responses for practice exercises
- **Simulator compatibility**: Gazebo/Unity/Isaac versioning issues across student environments → Mitigation: Provide Docker images pinning exact versions
- **Chatbot hallucinations**: Despite grounding, LLM may fabricate citations → Mitigation: Strict retrieval thresholds, post-generation citation validation, human review sampling
- **Code example fragility**: ROS 2 API changes in future releases may break examples → Mitigation: Pin to Humble LTS (supported until 2027), include version checks in setup scripts
- **Exercise difficulty calibration**: Exercises too hard/easy for target audience → Mitigation: Beta testing with students, provide hints system in chatbot
- **Performance on low-end hardware**: Students with 8GB RAM may struggle with Gazebo → Mitigation: Provide lightweight simulation alternatives, document minimum specs clearly
- **Scope creep**: Temptation to add advanced topics expands beyond 60k words → Mitigation: Ruthless prioritization per constitution, defer advanced content to "Further Reading" sections

## Implementation Strategy

**Deployment Sequencing** (Progressive Staged Approach):
1. **Phase 1 - Module 1 + Chatbot v1**: Complete Module 1 (ROS 2 Fundamentals) with all code examples, exercises, and diagrams; deploy to GitHub Pages; implement RAG chatbot trained only on Module 1 content; validate chatbot accuracy on Module 1 Q&A test set (≥90% threshold)
2. **Phase 2 - Modules 2-3**: Develop Module 2 (Simulation) and Module 3 (Perception/Control); incrementally retrain chatbot with new content; validate chatbot performance remains ≥90% across all modules
3. **Phase 3 - Modules 4-5 + Full Chatbot**: Complete Module 4 (VLA) and Module 5 (Capstone); integrate full chatbot with complete book corpus; run final accuracy benchmark on 100-question ground-truth test set
4. **Phase 4 - Beta Testing**: Conduct beta testing with 5 representative students; gather feedback on exercises, chatbot helpfulness, and code reproducibility; iterate based on feedback
5. **Phase 5 - Public Release**: Deploy final version to GitHub Pages; publish announcement and documentation

**Rationale**: Staged deployment enables early validation of book content quality and chatbot grounding mechanisms before scaling to full scope; reduces rework risk; allows student/instructor feedback to inform remaining modules.

## Next Steps

1. Run `/sp.clarify` to identify and resolve remaining specification ambiguities
2. Run `/sp.plan` to create detailed architectural plan for Phase 1 (Module 1 + Chatbot v1)
3. Run `/sp.tasks` to break down Phase 1 into testable tasks with acceptance criteria
4. Begin Module 1 content development following Spec-Kit Plus workflow
5. Set up Docusaurus repository structure and deploy skeleton site to GitHub Pages
6. Implement RAG chatbot backend (FastAPI + Qdrant + Neon) with grounding validation tests for Module 1
7. Develop ground-truth Q&A test set for Module 1 (20 questions minimum for initial validation)
