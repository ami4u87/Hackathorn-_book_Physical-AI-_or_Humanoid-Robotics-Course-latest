---
id: 3
title: Research Robotis OP3 ROS 2 Compatibility
stage: misc
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: master
branch: master
user: User
command: research
labels: ["research", "robotics", "ros2", "urdf", "compatibility"]
links:
  spec: specs/master/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/master/research-robotis-op3-ros2-compatibility.md
tests:
  - Manual verification checklist provided
---

## Prompt

Verify availability and ROS 2 Humble compatibility of Robotis OP3 humanoid robot model (URDF, MoveIt 2 config, Gazebo support).

Context: We selected Robotis OP3 (20 DOF open-source humanoid) for consistency across all course modules. Need URDF for Gazebo simulation, MoveIt 2 configuration for motion planning, and ros2_control for joint control.

Research questions:
1. Official GitHub repository: Is ROBOTIS-GIT/ROBOTIS-OP3 actively maintained?
2. ROS 2 support: Does official repo have ROS 2 Humble branches/packages or only ROS 1?
3. URDF availability: Is op3.urdf available and accurate (meshes, inertia, collision geometry)?
4. MoveIt 2 config: Is there an official MoveIt 2 configuration package for OP3?
5. Gazebo compatibility: Does OP3 URDF work with Gazebo Harmonic (latest) or only Classic?
6. ros2_control: Are there controller configs (joint_state_controller, position_controllers)?
7. Community support: Are there ROS 2 tutorials, examples, or community packages for OP3?
8. Alternatives if OP3 support is incomplete: Other open-source humanoids with better ROS 2 integration?

Provide:
- Links to official and community OP3 ROS 2 repositories
- Assessment of ROS 2 Humble readiness (production-ready vs needs porting)
- Steps needed to prepare OP3 for use (e.g., convert ROS 1 URDF, create MoveIt config)
- Backup humanoid robot recommendation if OP3 not viable

## Response snapshot

**CRITICAL FINDING**: Robotis OP3 official support is primarily ROS 1 (Kinetic/Melodic) with no official ROS 2 port confirmed as of January 2025. The ROBOTIS-GIT/ROBOTIS-OP3 repository has historically been maintained for ROS 1 environments and does not include native ROS 2 Humble packages, MoveIt 2 configurations, or ros2_control integration.

**KEY FINDINGS**:

1. **Repository Status**: ROS 1 only (low-moderate maintenance, primarily 2016-2019 era)
2. **ROS 2 Support**: ❌ NOT CONFIRMED - No official ROS 2 branches found
3. **URDF**: ✅ LIKELY AVAILABLE but needs modernization for ROS 2/Gazebo Harmonic
4. **MoveIt 2**: ❌ NOT AVAILABLE - Needs generation via Setup Assistant
5. **Gazebo Harmonic**: ⚠️ REQUIRES PORTING - Plugin updates needed
6. **ros2_control**: ❌ NOT AVAILABLE - Controller configs need creation
7. **Community Support**: Limited ROS 2 community activity found

**ESTIMATED PORTING EFFORT**: 4-6 weeks (one engineer, full-time)

**RECOMMENDED ALTERNATIVES**:
1. **NASA Valkyrie (R5)** - 44 DOF, official NASA repo with ROS 2 branches
2. **PAL Robotics TALOS** - 32 DOF, active ROS 2 development
3. **Custom Simplified Humanoid** - 10-15 DOF, built specifically for course
4. **Continue with OP3** - Accept 4-6 week porting timeline

**IMMEDIATE ACTION REQUIRED**: Manual verification of current repository status before proceeding with course development. Risk level: 🔴 HIGH - Selected robot may require substantial engineering work.

Full research report created at: `specs/master/research-robotis-op3-ros2-compatibility.md`

## Outcome

- ✅ Impact: Identified critical risk to course development timeline; OP3 not production-ready for ROS 2 Humble
- 🧪 Tests: Manual verification checklist provided (git clone, branch inspection, URDF testing)
- 📁 Files: Created comprehensive research report (12+ sections, 8 research questions answered)
- 🔁 Next prompts: Manual verification → platform decision → spec update → plan revision
- 🧠 Reflection: Web access limitation prevented real-time verification; provided knowledge-based assessment with clear verification steps for user to execute

## Evaluation notes (flywheel)

- Failure modes observed: No web access (WebSearch/WebFetch denied); relied on training data (January 2025 cutoff)
- Graders run and results (PASS/FAIL): N/A (research task, not code implementation)
- Prompt variant (if applicable): Standard research request with 8 specific questions
- Next experiment (smallest change to try): User runs manual verification checklist, reports findings for decision refinement
