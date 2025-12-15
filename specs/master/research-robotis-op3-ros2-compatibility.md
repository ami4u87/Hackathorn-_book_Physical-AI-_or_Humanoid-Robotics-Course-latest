# Research Report: Robotis OP3 ROS 2 Humble Compatibility Assessment

**Date**: 2025-12-15
**Researcher**: Claude Sonnet 4.5
**Spec**: [spec.md](./spec.md)
**Plan**: [plan.md](./plan.md)
**Status**: Draft - Requires Manual Verification

---

## Executive Summary

**CRITICAL FINDING**: Robotis OP3 official support is **primarily ROS 1 (Kinetic/Melodic)** with **no official ROS 2 port** as of my knowledge cutoff (January 2025). The ROBOTIS-GIT/ROBOTIS-OP3 repository has historically been maintained for ROS 1 environments and does not include native ROS 2 Humble packages, MoveIt 2 configurations, or ros2_control integration.

**RECOMMENDATION**:
1. **Immediate Action Required**: Manually verify current repository status (see verification checklist below)
2. **If OP3 lacks ROS 2 support**: Consider alternative humanoid platforms with proven ROS 2 Humble integration, or plan for significant porting effort
3. **Backup Options**: Evaluate PAL Robotics TALOS, NASA Valkyrie (open-source), or simplified custom URDF humanoid

**Risk Level**: 🔴 **HIGH** - Selected robot may require substantial engineering work to meet course requirements

---

## Research Questions & Findings

### 1. Official GitHub Repository Maintenance Status

**Repository**: `ROBOTIS-GIT/ROBOTIS-OP3`

**Last Known Status** (as of January 2025):
- **Primary Branch**: ROS 1 packages (Kinetic/Melodic era)
- **Activity Level**: Low to moderate maintenance (primarily bug fixes, not feature development)
- **ROS 2 Branches**: No official ROS 2 branches confirmed in my knowledge base
- **Last Major Update**: Repository primarily active 2016-2019 timeframe

**Related Repositories**:
- `ROBOTIS-GIT/ROBOTIS-OP3-Tools`: ROS 1 utilities (walking, balance, manipulation demos)
- `ROBOTIS-GIT/ROBOTIS-OP3-Demo`: ROS 1 demonstration packages
- `ROBOTIS-GIT/ROBOTIS-OP3-Common`: Shared libraries for OP3 control

**⚠️ VERIFICATION NEEDED**:
```bash
# Commands to run manually (requires internet access):
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-OP3.git
cd ROBOTIS-OP3
git branch -a  # Check for ros2, humble, or galactic branches
git log -n 10 --oneline  # Check recent commit activity
cat README.md | grep -i "ros 2\|humble\|foxy\|galactic"
```

---

### 2. ROS 2 Support Assessment

**Official Support**: ❌ **NOT CONFIRMED**

**Evidence from Knowledge Base**:
- Robotis historically focused on ROS 1 for OP3 (released ~2016, peak ROS 1 era)
- OP3's successor models (OP3-P) also primarily documented for ROS 1
- No official migration announcements found in my training data

**Community Ports**: 🔍 **UNKNOWN** (requires verification)
- Possible community efforts on GitHub/GitLab (not in my knowledge base)
- University research groups may have private ROS 2 ports

**Expected Gaps if No Official Port**:
- Package structure needs conversion (catkin → colcon)
- Launch files need rewriting (XML → Python)
- Message/service dependencies need updating (std_msgs, geometry_msgs API changes)
- ros2_control integration required (replaces ros_control)
- Gazebo Classic → Gazebo Harmonic bridge updates

**⚠️ VERIFICATION NEEDED**:
```bash
# Search for community ROS 2 ports:
# 1. GitHub: Search "robotis op3 ros2" or "robotis op3 humble"
# 2. ROS Discourse: https://discourse.ros.org (search "OP3 ROS 2")
# 3. ROS Index: https://index.ros.org/packages/ (search "op3")
```

---

### 3. URDF Availability and Accuracy

**URDF Status**: ✅ **LIKELY AVAILABLE** (for ROS 1)

**Expected URDF Location**:
- `ROBOTIS-OP3/robotis_op3_description/urdf/robotis_op3.urdf` (or .xacro)

**URDF Components** (typical for OP3):
- **DOF**: 20 joints (head: 2, arms: 6, legs: 12)
- **Meshes**: STL or Collada (.dae) files for visual/collision geometry
- **Sensors**: IMU (MPU-9250), LIDAR (not standard), cameras
- **Materials**: Defined textures and colors

**Potential Issues**:
1. **ROS 1 URDF Compatibility**: ROS 1 URDFs generally work in ROS 2, but may need:
   - Updated `<gazebo>` plugin tags for Gazebo Harmonic
   - ros2_control `<ros2_control>` tags added (replacing `<transmission>` tags)
   - Updated sensor plugin names (e.g., `libgazebo_ros_camera.so` → `libgazebo_ros2_camera.so`)

2. **Inertia Properties**: Older URDFs may have placeholder inertia values (not physics-accurate)
   - Critical for simulation fidelity
   - May need recalculation from CAD models or experimental tuning

3. **Collision Geometry**: May use simplified convex hulls; verify accuracy for manipulation tasks

**⚠️ VERIFICATION NEEDED**:
```bash
# After cloning ROBOTIS-OP3 repository:
find . -name "*.urdf" -o -name "*.xacro"
# Check for meshes:
find . -name "*.stl" -o -name "*.dae"
# Inspect URDF for ros2_control tags:
grep -r "ros2_control" .
grep -r "transmission" .  # Old ROS 1 pattern
```

---

### 4. MoveIt 2 Configuration Availability

**MoveIt 2 Config Status**: ❌ **LIKELY NOT AVAILABLE**

**Expected for MoveIt 2**:
- `moveit_config` package with:
  - SRDF (Semantic Robot Description Format): defines planning groups, end effectors, collision matrices
  - `kinematics.yaml`: IK solver configuration (KDL, TRAC-IK, etc.)
  - `joint_limits.yaml`: velocity/acceleration limits
  - `controllers.yaml`: ros2_control controller mappings
  - Planning scene configuration

**Conversion Effort** (if starting from ROS 1 MoveIt config):
- **MoveIt 1 → MoveIt 2**: Moderate effort (1-2 weeks)
  - SRDF mostly compatible (minor syntax updates)
  - Controller interface needs rewriting for ros2_control
  - Planning pipeline config format changed
  - RViz plugins updated (MoveIt 2 uses different plugins)

**Alternatives if No MoveIt 2 Config**:
1. **Use MoveIt Setup Assistant**:
   - Run `ros2 launch moveit_setup_assistant setup_assistant.launch.py`
   - Load OP3 URDF and generate config interactively
   - Manually define planning groups (arms, legs, full_body)
   - Configure collision matrices

2. **Use Generic IK Solvers**:
   - Direct IK solving without MoveIt (e.g., TracIK Python bindings)
   - Less robust (no collision checking, path planning)

**⚠️ VERIFICATION NEEDED**:
```bash
# Search for existing MoveIt 2 configs:
# GitHub: "robotis op3 moveit2" or "robotis op3 moveit humble"
# Check ROBOTIS-OP3-Tools for MoveIt 1 config (basis for porting)
```

---

### 5. Gazebo Compatibility

**Gazebo Classic (11.x)**: ✅ **LIKELY COMPATIBLE** (if ROS 1 packages exist)
- OP3 was developed during Gazebo Classic era
- ROS 1 launch files likely spawn robot in Gazebo Classic

**Gazebo Harmonic (Latest)**: ⚠️ **REQUIRES PORTING**

**Key Differences**:
- **Plugin Names**: `libgazebo_ros_*.so` → `libgz_ros2_*.so`
- **SDF Format**: Gazebo Harmonic prefers SDF (not URDF), but URDF still supported via `gz-urdf` parser
- **ros2_control Integration**: Requires `gz_ros2_control` package (different from `gazebo_ros_control`)
- **Sensor Plugins**: Camera, IMU, LIDAR plugins have new APIs

**Migration Path**:
1. Test OP3 URDF in Gazebo Harmonic using `ros2 launch gazebo_ros spawn_entity.py`
2. Update `<gazebo>` plugin tags to Harmonic-compatible versions
3. Integrate `gz_ros2_control` for joint control
4. Validate physics behavior (joint damping, friction, gravity response)

**Alternative**: Use Gazebo Classic (11.x) for initial modules, migrate to Harmonic in later phase
- **Pros**: Faster initial setup, proven compatibility
- **Cons**: Gazebo Classic end-of-life approaching, students learn outdated tools

**⚠️ VERIFICATION NEEDED**:
```bash
# Test OP3 URDF in Gazebo Harmonic (if URDF obtained):
ros2 launch gazebo_ros gazebo.launch.py
# In another terminal:
ros2 run gazebo_ros spawn_entity.py -entity op3 -file /path/to/robotis_op3.urdf
# Check for errors in Gazebo console and ROS 2 logs
```

---

### 6. ros2_control Configuration

**ros2_control Status**: ❌ **LIKELY NOT AVAILABLE**

**Required Components**:
1. **Hardware Interface**:
   - For simulation: `gazebo_ros2_control` plugin in URDF
   - For real hardware: Custom hardware interface (if deploying to physical OP3)

2. **Controller Configuration** (`controllers.yaml`):
   ```yaml
   controller_manager:
     ros__parameters:
       update_rate: 100  # Hz

       joint_state_broadcaster:
         type: joint_state_broadcaster/JointStateBroadcaster

       arm_controller:
         type: joint_trajectory_controller/JointTrajectoryController

       # Add position/velocity controllers for individual joints
   ```

3. **URDF Integration**:
   ```xml
   <ros2_control name="op3_control" type="system">
     <hardware>
       <plugin>gazebo_ros2_control/GazeboSystem</plugin>
     </hardware>
     <!-- Define joints and interfaces -->
   </ros2_control>
   ```

**Creation Effort**:
- **From Scratch**: 1-2 weeks (requires understanding OP3 joint configuration)
- **From ROS 1 ros_control**: 1 week (conversion patterns are established)

**⚠️ VERIFICATION NEEDED**:
```bash
# Check for ros2_control tags in URDF:
grep -A 20 "ros2_control" /path/to/robotis_op3.urdf
# Check for controller configs:
find . -name "controllers.yaml" -o -name "*controller*.yaml"
```

---

### 7. Community Support & Tutorials

**ROS 1 Community**: ✅ **STRONG** (historical)
- Numerous tutorials for OP3 walking, kicking, manipulation (ROS 1)
- Academic papers using OP3 as research platform
- ROS Answers/ROS Discourse threads for troubleshooting

**ROS 2 Community**: ⚠️ **UNCERTAIN**
- No major ROS 2 OP3 tutorials found in my training data
- Possible private/academic implementations not publicly documented

**Resources to Check**:
1. **Official Robotis E-Manual**: http://emanual.robotis.com/docs/en/platform/op3/introduction/
   - Check for ROS 2 documentation sections
2. **ROS Discourse**: Search "OP3" in https://discourse.ros.org
3. **GitHub Issues**: Check ROBOTIS-GIT/ROBOTIS-OP3 issues for ROS 2 requests/discussions
4. **Research Papers**: Google Scholar search "Robotis OP3 ROS 2" (2023-2025 papers)

---

### 8. Alternative Humanoid Robots (Backup Options)

Given the ROS 2 uncertainty with OP3, here are alternative open-source humanoid platforms:

#### Option A: **NASA Valkyrie (R5)** 🌟 RECOMMENDED ALTERNATIVE
- **DOF**: 44 (more complex than OP3, but well-documented)
- **ROS 2 Support**: ✅ Official NASA repo has ROS 2 branches
- **URDF**: ✅ Available with accurate inertia, collision meshes
- **MoveIt Support**: ✅ MoveIt configs available (may need MoveIt 2 porting)
- **Gazebo**: ✅ Works with Gazebo Classic, Harmonic compatibility likely
- **Repository**: https://github.com/NASA-JSC/valkyrie (verify ROS 2 status)
- **Pros**: Professional-grade model, active NASA maintenance, impressive for students
- **Cons**: Higher complexity (44 DOF), larger simulation footprint

#### Option B: **PAL Robotics TALOS**
- **DOF**: 32
- **ROS 2 Support**: ✅ PAL Robotics actively developing ROS 2 for TALOS
- **URDF**: ✅ High-quality URDF with accurate dynamics
- **MoveIt Support**: ✅ MoveIt 2 configs may be available
- **Gazebo**: ✅ Gazebo integration maintained
- **Repository**: https://github.com/pal-robotics/talos_robot (check for ros2 branch)
- **Pros**: Industrial-quality, well-maintained, good documentation
- **Cons**: May have licensing restrictions (check for educational use)

#### Option C: **TIAGo++ (Mobile Manipulator, Not Pure Humanoid)**
- **DOF**: 2 x 7-DOF arms + mobile base
- **ROS 2 Support**: ✅ Excellent ROS 2 Humble support
- **MoveIt 2**: ✅ Official MoveIt 2 configs
- **Gazebo**: ✅ Full Gazebo Harmonic support
- **Repository**: https://github.com/pal-robotics/tiago_robot
- **Pros**: Best ROS 2 support, easier than full humanoid, still does manipulation
- **Cons**: Not bipedal humanoid (may not fit "humanoid robotics" course theme)

#### Option D: **Custom Simplified Humanoid URDF**
- **DOF**: 10-15 (simplified design)
- **ROS 2 Support**: ✅ Build from scratch for ROS 2
- **Effort**: 2-3 weeks to design, model, and validate
- **Pros**: Full control, minimal complexity, guaranteed ROS 2 compatibility
- **Cons**: Not based on real robot, less impressive visually

#### Option E: **Continue with OP3 (Port to ROS 2)**
- **Effort**: 4-6 weeks for full port (URDF updates, ros2_control, MoveIt 2 config, Gazebo Harmonic)
- **Pros**: Matches original course plan, good learning opportunity
- **Cons**: Significant engineering burden, risk of unforeseen issues

---

## Assessment Summary Table

| Criterion | OP3 Status | Required Action | Effort | Risk |
|-----------|------------|-----------------|--------|------|
| **Active Maintenance** | ⚠️ ROS 1 only | Verify repo activity | Low | Medium |
| **ROS 2 Packages** | ❌ Not available | Port or find community version | High | High |
| **URDF** | ✅ Likely exists | Test compatibility, update plugins | Medium | Low |
| **MoveIt 2 Config** | ❌ Not available | Generate with Setup Assistant | Medium | Medium |
| **Gazebo Harmonic** | ❌ Not ready | Update plugins, test physics | Medium | Medium |
| **ros2_control** | ❌ Not available | Write controller configs, integrate | High | Medium |
| **Community Support** | ⚠️ ROS 1 only | Search for ROS 2 examples | Low | Medium |

**Overall Readiness**: 🔴 **NOT PRODUCTION-READY** for ROS 2 Humble (as of January 2025 knowledge)

---

## Recommended Steps to Prepare OP3 for Course Use

### If Continuing with OP3:

#### Phase 1: Verification (1-2 days)
1. ✅ Clone all OP3 repositories and inspect file structure
2. ✅ Search GitHub/ROS Discourse for community ROS 2 ports
3. ✅ Test URDF loading in RViz2: `ros2 run rviz2 rviz2` (load URDF manually)
4. ✅ Check Robotis E-Manual for any ROS 2 migration guides

#### Phase 2: URDF Modernization (1 week)
1. Convert URDF to use ros2_control tags
2. Update Gazebo plugins for Harmonic compatibility
3. Validate joint limits, inertia properties from datasheets
4. Test spawning in Gazebo Harmonic with basic joint commands

#### Phase 3: ros2_control Integration (1 week)
1. Write `controllers.yaml` for joint_state_broadcaster and trajectory controllers
2. Create launch files for controller spawning
3. Test position/velocity control via ROS 2 topics
4. Validate with simple motion sequences (wave arm, step in place)

#### Phase 4: MoveIt 2 Configuration (1-2 weeks)
1. Run MoveIt Setup Assistant with updated URDF
2. Define planning groups (left_arm, right_arm, legs)
3. Configure self-collision matrices
4. Set up IK solvers (KDL or TRAC-IK)
5. Test motion planning in RViz2 with MoveIt 2 plugin

#### Phase 5: Validation & Documentation (1 week)
1. Create example launch files for common tasks
2. Write setup guide for students (dependencies, build process)
3. Test on fresh Ubuntu 22.04 install for reproducibility
4. Document known issues and workarounds

**Total Estimated Effort**: 4-6 weeks (one engineer, full-time)

---

## Decision Framework

### Choose OP3 If:
- ✅ Official ROS 2 port exists (verify first)
- ✅ Course has 4-6 weeks buffer for porting work
- ✅ Students will benefit from learning migration process
- ✅ OP3's 20 DOF complexity matches course difficulty target

### Choose Alternative If:
- ✅ Need immediate production-ready ROS 2 support
- ✅ Limited time for infrastructure work (focus on content)
- ✅ Want guaranteed MoveIt 2, Gazebo Harmonic compatibility
- ✅ Higher-quality dynamics simulation is priority

---

## Immediate Action Items

### Priority 1: Manual Verification (DO FIRST)
```bash
# 1. Check official repository status
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-OP3.git
cd ROBOTIS-OP3
git branch -a
git log -n 20 --oneline
cat README.md

# 2. Search for ROS 2 community ports
# Visit: https://github.com/search?q=robotis+op3+ros2&type=repositories
# Visit: https://discourse.ros.org/search?q=robotis%20op3%20ros2

# 3. Check Robotis E-Manual
# Visit: http://emanual.robotis.com/docs/en/platform/op3/
# Look for "ROS 2" sections

# 4. Test URDF in ROS 2 (if URDF found)
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat robotis_op3.urdf)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui
ros2 run rviz2 rviz2  # Add RobotModel display
```

### Priority 2: Decision Point
Based on verification results:
- **If ROS 2 support found**: Proceed with OP3, update course plan
- **If no ROS 2 support**: Evaluate alternatives (NASA Valkyrie recommended) or allocate porting effort

### Priority 3: Update Course Spec
- Document actual robot platform decision in `spec.md`
- Update Module 2 (Simulation) with confirmed robot model and setup instructions
- Adjust timeline if porting effort required

---

## Links & Resources

### Official Repositories (Verify Current Status)
- ROBOTIS-OP3 Main: https://github.com/ROBOTIS-GIT/ROBOTIS-OP3
- ROBOTIS-OP3-Tools: https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Tools
- ROBOTIS-OP3-Demo: https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Demo
- Robotis E-Manual: http://emanual.robotis.com/docs/en/platform/op3/

### ROS 2 Resources
- ros2_control Documentation: https://control.ros.org/
- MoveIt 2 Tutorials: https://moveit.picknik.ai/main/index.html
- Gazebo Harmonic + ROS 2: https://gazebosim.org/docs/harmonic/ros2_integration

### Alternative Robot Platforms
- NASA Valkyrie: https://github.com/NASA-JSC/valkyrie
- PAL TALOS: https://github.com/pal-robotics/talos_robot
- PAL TIAGo++: https://github.com/pal-robotics/tiago_robot

### Community Support
- ROS Discourse: https://discourse.ros.org
- ROS Answers: https://answers.ros.org
- Robotics Stack Exchange: https://robotics.stackexchange.com

---

## Conclusion

**CRITICAL RECOMMENDATION**: **Do NOT proceed with course development until OP3 ROS 2 compatibility is verified**. The risk of selecting an unsupported platform is too high given the course's reproducibility and student success requirements.

**Next Steps**:
1. ✅ Run manual verification checklist (Priority 1 above)
2. ✅ Make go/no-go decision on OP3 within 2-3 days
3. ✅ If OP3 not viable, immediately pivot to NASA Valkyrie or custom URDF
4. ✅ Update `spec.md` and `plan.md` with confirmed platform decision

**Fallback Plan**: If verification reveals OP3 is not ROS 2-ready and no suitable alternative exists:
- **Short-term**: Use a simple 7-DOF arm URDF (e.g., Panda, UR5) for Modules 1-3 to teach concepts
- **Long-term**: Custom design a simplified humanoid (10-15 DOF) specifically for course use
- **Benefit**: Full control, guaranteed compatibility, opportunity to open-source as educational resource

---

**Report Status**: 🔍 **Awaiting Manual Verification**

**Last Updated**: 2025-12-15
