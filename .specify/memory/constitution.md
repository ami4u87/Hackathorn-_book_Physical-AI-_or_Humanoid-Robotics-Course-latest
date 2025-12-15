# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### I. Technical Accuracy Above All
All technical content must be verifiable, current, and correct:
- Zero tolerance for hallucinations or speculative information
- Every technical claim must be traceable to authoritative sources
- ROS 2, Gazebo, Unity, NVIDIA Isaac implementations must match official documentation
- Real-world robotics constraints (physics, safety, hardware limitations) must be explicitly addressed
- All robotics code examples must be tested in simulation or on hardware before publication

### II. Reproducibility First
Every code example, tutorial, and exercise must be fully reproducible:
- Complete environment setup instructions (versions, dependencies, configuration)
- All code must run as-is without modification or guesswork
- Step-by-step workflows with expected outputs at each stage
- Clear error messages and troubleshooting guidance
- Docker/container definitions where applicable for environment consistency
- ROS 2 workspaces, launch files, and package structures fully specified

### III. Safety-First Robotics Guidance
All robotics content must prioritize safety and responsible development:
- Explicit safety warnings for physical robot operations
- Emergency stop procedures clearly documented
- Workspace safety requirements specified
- Human-robot interaction safety principles enforced
- Simulation-first testing before hardware deployment
- No content that could lead to unsafe robot behavior without proper warnings

### IV. Grounded RAG Chatbot (Zero Hallucination)
The embedded RAG chatbot must operate under strict grounding constraints:
- Answers ONLY from book content or user-selected text passages
- Explicit "I don't know" responses when information is not in source material
- Source citations required for every answer (chapter, section, page reference)
- No external knowledge retrieval beyond the book corpus
- Vector similarity thresholds tuned for high precision over recall
- Regular accuracy testing: ≥90% correctness on ground-truth test sets

### V. Test-Driven Development for All Code
All code examples must follow rigorous testing standards:
- Unit tests for standalone functions and classes
- Integration tests for ROS 2 nodes and multi-component systems
- Simulation tests in Gazebo/Unity/Isaac before claiming functionality
- Test fixtures and mock data provided alongside code
- Expected outputs documented for verification
- CI/CD validation where applicable

### VI. Clear Engineering-Level Explanations
Technical depth balanced with clarity:
- Assume reader has programming background but not necessarily robotics expertise
- Mathematical formulations with intuitive explanations
- Diagrams and visualizations for spatial/temporal concepts
- Terminology defined on first use; glossary maintained
- Real-world analogies where helpful, but never at expense of accuracy
- Architecture diagrams showing system component interactions

### VII. Modular Structure with Progressive Complexity
Content organized for systematic skill building:
- Module 1: ROS 2 fundamentals (pub/sub, services, actions, parameters)
- Module 2: Simulation environments (Gazebo, Unity, Isaac Sim)
- Module 3: Perception and control systems
- Module 4: Vision-Language-Action (VLA) models
- Module 5: Capstone integration project
- Each module: Theory → Diagrams → Code → Exercises → Assessment
- Later modules build on earlier ones; dependencies explicit

## Technology Stack Standards

### Book Platform
- **Framework**: Docusaurus (latest stable version)
- **Deployment**: GitHub Pages
- **Structure**: Spec-Kit Plus compliant
- **Content**: Markdown with MDX for interactive components
- **Assets**: Diagrams (draw.io, Mermaid), videos (hosted externally), code blocks with syntax highlighting

### RAG Chatbot Stack
- **Orchestration**: OpenAI Agents API with ChatKit interface
- **Backend**: FastAPI (Python 3.11+)
- **Vector Database**: Qdrant (cloud or self-hosted)
- **Relational Database**: Neon Postgres (for metadata, user sessions, analytics)
- **Embeddings**: OpenAI text-embedding-3-small or text-embedding-3-large
- **LLM**: GPT-4 Turbo or GPT-4o (with strict grounding prompts)
- **Deployment**: Containerized (Docker) with environment variables for API keys

### Robotics Development Stack
- **Framework**: ROS 2 Humble or later LTS
- **Simulators**: Gazebo Classic/Fortress/Harmonic, Unity with ROS-TCP-Connector, NVIDIA Isaac Sim
- **Languages**: Python 3.11+ (primary), C++17 (performance-critical nodes)
- **Build System**: colcon with CMake/setuptools
- **Testing**: pytest (Python), gtest (C++), launch_testing (ROS 2 integration)
- **Hardware Interfaces**: ros2_control, MoveIt 2 (manipulation), Nav2 (navigation)

### Code Quality Standards
- **Style**: PEP 8 (Python), Google C++ Style Guide
- **Type Hints**: Required for all Python function signatures
- **Documentation**: Docstrings (Google style), inline comments for non-obvious logic
- **Linting**: ruff (Python), clang-tidy (C++)
- **Formatting**: black (Python), clang-format (C++)
- **Version Control**: Git with conventional commits

## Content Quality Requirements

### Word Count and Scope
- **Total**: 40,000–60,000 words
- **Modules**: 5 major modules (8,000–12,000 words each)
- **Exercises**: Minimum 3 per module with solutions
- **Code Examples**: Minimum 50 complete, runnable examples across all modules
- **Diagrams**: Minimum 30 technical diagrams

### Citation and Sources
- **Total Sources**: ≥40 authoritative sources
- **Peer-Reviewed**: ≥50% (20+ papers from journals/conferences)
- **Citation Style**: APA 7th edition
- **Source Types**: Research papers, official documentation (ROS 2, NVIDIA, Unity), textbooks, technical standards
- **Citation Frequency**: Key claims cited; implementation details reference official docs

### Terminology and Consistency
- **Glossary**: Comprehensive glossary of robotics, AI, and ML terms
- **Acronym Expansion**: First use expanded, then acronym consistently used
- **Naming Conventions**: ROS 2 conventions (snake_case for topics/services, PascalCase for nodes)
- **Units**: SI units throughout; conversions noted where relevant
- **Coordinate Frames**: Explicit frame names (base_link, odom, map, etc.)

## Embedded Chatbot Validation

### Functional Requirements
- **Embedding**: iframe or web component embedded in Docusaurus pages
- **Context Window**: Current page/section content automatically included in retrieval context
- **User Interaction**: Text input, streaming responses, source citations clickable to navigate book
- **Session Management**: Conversation history preserved within session
- **Error Handling**: Graceful fallback messages; API failures logged

### Accuracy Testing Protocol
- **Test Set**: 100 ground-truth Q&A pairs covering all modules
- **Evaluation Metrics**: Exact match, F1 score (token-level), semantic similarity (embedding cosine)
- **Threshold**: ≥90% correctness (at least 90/100 pass with F1 ≥ 0.8)
- **Hallucination Detection**: Human review of 20 random responses per module for unsupported claims
- **Bias Testing**: Check for factual consistency across rephrased questions

### Grounding Enforcement
- **System Prompt**: Explicit instructions to ONLY answer from provided context
- **Citation Format**: "According to [Chapter X, Section Y]..." or "The book states in [Section Z]..."
- **Refusal Template**: "I don't have information on that topic in this book. Try rephrasing or checking the Table of Contents."
- **Context Injection**: RAG retrieves top 5 relevant chunks; concatenated with explicit boundaries
- **No Web Search**: Chatbot has no internet access; only book vector database

## Development Workflow

### Feature Development
- **Specification**: /sp.specify to create specs/<feature>/spec.md
- **Planning**: /sp.plan to create specs/<feature>/plan.md
- **Task Breakdown**: /sp.tasks to create specs/<feature>/tasks.md
- **Implementation**: /sp.implement to execute task-by-task with tests
- **ADR Creation**: /sp.adr for architecturally significant decisions
- **Commit & PR**: /sp.git.commit_pr to commit and create pull request

### Quality Gates
- **Pre-Commit**: All code formatted, linted, type-checked
- **Pre-Merge**: All tests passing (unit, integration, simulation)
- **Pre-Publish**: Manual review of technical accuracy, reproducibility verification
- **Chatbot Validation**: Accuracy threshold met on test set before deployment

### Knowledge Capture
- **Prompt History Records (PHRs)**: Created automatically after every user interaction
  - Routed to `history/prompts/constitution/`, `history/prompts/<feature>/`, or `history/prompts/general/`
  - Full prompt preserved verbatim; response summarized
  - Metadata includes stage, files modified, tests run, next steps
- **Architecture Decision Records (ADRs)**: Suggested for significant decisions (technology choices, architectural patterns, API designs)
  - Created in `history/adr/` with context, decision, consequences, alternatives
  - Linked from relevant specs/plans

## Constraints and Non-Goals

### In Scope
- ROS 2 fundamentals and intermediate topics
- Simulation environments setup and usage
- Perception pipelines (cameras, LiDAR, depth sensors)
- Motion planning and control
- VLA model integration concepts
- Full-stack RAG chatbot implementation

### Out of Scope
- Hardware recommendations or purchasing guides (focus on software/simulation)
- Custom robot hardware design (assume standard platforms like TurtleBot, UR5)
- Production deployment at scale (educational focus)
- Non-ROS frameworks (unless for VLA integration)
- Mechanical engineering or CAD design

### Non-Negotiable Constraints
- No unsupported speculation about future robotics capabilities
- No code that skips safety checks or encourages unsafe practices
- No external dependencies without version pinning and license verification
- No content that cannot be verified against authoritative sources
- No chatbot answers derived from LLM parametric knowledge alone

## Success Metrics

### Book Completeness
- All 5 modules complete with theory, code, exercises
- All code examples tested and reproducible
- All diagrams render correctly in Docusaurus
- Citation count and peer-review ratio met
- Deployed successfully to GitHub Pages

### Chatbot Functionality
- Embedded in all main content pages
- Responds within 3 seconds for 95% of queries
- ≥90% accuracy on ground-truth test set
- Zero detected hallucinations in validation sample
- Source citations provided for 100% of factual answers

### User Validation (if applicable)
- Exercises completable by readers with specified prerequisites
- Code runs without errors on specified platforms (Ubuntu 22.04, ROS 2 Humble)
- Chatbot rated helpful by test users (if user testing conducted)

## Governance

### Amendment Process
- Constitution changes require documentation of rationale and impact
- Major principle changes require ADR
- Updates versioned and dated below

### Compliance Verification
- All PRs reviewed against constitution principles
- Periodic audits of code reproducibility and citation accuracy
- Chatbot accuracy testing before each major release

**Version**: 1.0.0
**Ratified**: 2025-12-15
**Last Amended**: 2025-12-15
