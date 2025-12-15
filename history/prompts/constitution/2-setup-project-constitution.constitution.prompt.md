---
id: 2
title: Setup Project Constitution
stage: constitution
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: User Name
command: /sp.constitution
labels: ["constitution", "setup", "project-initialization", "physical-ai", "humanoid-robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution

Project:
Technical book + embedded RAG chatbot on Physical AI & Humanoid Robotics using Spec-Kit Plus, Claude Code, Docusaurus, GitHub Pages, OpenAI Agents/ChatKit, FastAPI, Neon, and Qdrant.

Core Principles:
- High technical accuracy (ROS 2, Gazebo, Unity, NVIDIA Isaac)
- Clear engineering-level explanations
- Fully reproducible code/workflows
- Strict grounding; no hallucinations
- Safe robotics guidance only

Standards:
- Docusaurus book (Spec-Kit Plus structure)
- All code must run as-is (ROS 2, Gazebo, Unity, Isaac, FastAPI, Agents)
- Chatbot answers only from book or selected text
- ≥40 sources, ≥50% peer-reviewed, APA 7
- Consistent terminology and formatting

Constraints:
- 40k–60k words
- Modules: ROS 2 → Gazebo/Unity → Isaac → VLA → Capstone
- Each module: theory, diagrams, runnable code, exercises
- RAG stack: OpenAI Agents + FastAPI + Neon Postgres + Qdrant
- Zero hallucination tolerance

Success Criteria:
- Fully deployed Docusaurus book on GitHub Pages
- Embedded RAG chatbot fully functional and grounded
- Complete, accurate, reproducible modules
- Chatbot ≥90% correctness on content-based tests

## Response snapshot

Acknowledged project constitution for a technical book on Physical AI and Humanoid Robotics with strict accuracy requirements, reproducible code standards, and an embedded RAG chatbot system. Constitution needs to be formally written to .specify/memory/constitution.md with proper principles and governance structure.

## Outcome

- ✅ Impact: Project constitution recorded; formal constitution document needs creation
- 🧪 Tests: none
- 📁 Files: PHR created; constitution.md identified for update
- 🔁 Next prompts: Create formal constitution.md with robotics-specific principles, code standards, and testing requirements
- 🧠 Reflection: Constitution establishes zero-hallucination tolerance and grounding requirements critical for robotics safety

## Evaluation notes (flywheel)

- Failure modes observed: Constitution template contains generic placeholders; needs domain-specific adaptation for robotics, RAG systems, and technical accuracy requirements
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Populate constitution.md with robotics-specific principles (safety-first, reproducibility, grounding, test-driven development for physical systems)
