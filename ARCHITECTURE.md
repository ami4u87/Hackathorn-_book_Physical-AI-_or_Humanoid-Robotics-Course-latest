# CI/CD Architecture Diagram

Visual reference for the ROS 2 Humble + Gazebo Harmonic CI/CD pipeline.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         GitHub Repository                            │
│  ┌────────────────┐  ┌──────────────┐  ┌─────────────────────────┐ │
│  │  Source Code   │  │  Dockerfile  │  │  GitHub Workflows       │ │
│  │  (50+ pkgs)    │  │  (ROS + Gz)  │  │  (ros2-ci.yml)          │ │
│  └────────────────┘  └──────────────┘  └─────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                │ git push
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      GitHub Actions Runner                           │
│                       (Ubuntu 22.04, 2-core, 7GB)                    │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  Step 1: Docker Build (30s with cache, 8-10min without)     │   │
│  │  ┌─────────────────────────────────────────────────────┐    │   │
│  │  │  Docker BuildKit + Layer Caching                    │    │   │
│  │  │  ┌───────────────────────────────────────────────┐  │    │   │
│  │  │  │  Base: osrf/ros:humble-desktop-full           │  │    │   │
│  │  │  │  + Gazebo Harmonic (gz-sim8)                  │  │    │   │
│  │  │  │  + ROS-Gazebo Bridge (ros-humble-ros-gz)      │  │    │   │
│  │  │  │  + Test Tools (pytest, colcon, launch_testing)│  │    │   │
│  │  │  │  + Xvfb (Virtual Display)                     │  │    │   │
│  │  │  │  + Mesa (Software Rendering)                  │  │    │   │
│  │  │  └───────────────────────────────────────────────┘  │    │   │
│  │  └─────────────────────────────────────────────────────┘    │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                │                                     │
│                                ▼                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  Step 2: Build Workspace (2-3 min with cache)              │   │
│  │  ┌─────────────────────────────────────────────────────┐    │   │
│  │  │  $ source /opt/ros/humble/setup.bash                │    │   │
│  │  │  $ colcon build --cmake-args -DCMAKE_BUILD_TYPE=... │    │   │
│  │  │                                                       │    │   │
│  │  │  Output: build/ install/ log/                       │    │   │
│  │  └─────────────────────────────────────────────────────┘    │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                │                                     │
│                                ▼                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  Step 3: Test Execution (3-5 min)                          │   │
│  │  ┌─────────────────────────────────────────────────────┐    │   │
│  │  │  Virtual Display Setup:                             │    │   │
│  │  │  $ Xvfb :99 -screen 0 1920x1080x24 &                │    │   │
│  │  │  $ export DISPLAY=:99                                │    │   │
│  │  │  $ export LIBGL_ALWAYS_SOFTWARE=1                    │    │   │
│  │  │                                                       │    │   │
│  │  │  Test Execution:                                     │    │   │
│  │  │  $ source install/setup.bash                         │    │   │
│  │  │  $ colcon test --pytest-args -v                      │    │   │
│  │  │                                                       │    │   │
│  │  │  Results: build/*/test_results/*.xml                 │    │   │
│  │  └─────────────────────────────────────────────────────┘    │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                │                                     │
│                                ▼                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  Step 4: Integration Tests (Matrix, 5-10 min)              │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │   │
│  │  │ basic-pub-sub│  │services-act. │  │gazebo-sim    │      │   │
│  │  │  (30s)       │  │  (45s)       │  │  (90s)       │      │   │
│  │  │              │  │              │  │              │      │   │
│  │  │  Parallel    │  │  Parallel    │  │  Parallel    │      │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘      │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                │                                     │
│                                ▼                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  Step 5: Publish Results                                    │   │
│  │  ┌─────────────────────────────────────────────────────┐    │   │
│  │  │  • JUnit XML → GitHub Test Results UI               │    │   │
│  │  │  • Test Logs → Artifacts (30 days)                  │    │   │
│  │  │  • Coverage → Codecov (if enabled)                  │    │   │
│  │  │  • Build Logs → Artifacts (on failure, 7 days)      │    │   │
│  │  └─────────────────────────────────────────────────────┘    │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Docker Image Layers

```
┌──────────────────────────────────────────────────────────┐
│  Layer 8: Entrypoint (docker-entrypoint.sh)              │  < 1 MB
├──────────────────────────────────────────────────────────┤
│  Layer 7: Colcon Mixins                                  │  < 1 MB
├──────────────────────────────────────────────────────────┤
│  Layer 6: Test Tools (pytest, coverage, etc.)            │  ~50 MB
├──────────────────────────────────────────────────────────┤
│  Layer 5: Virtual Display (Xvfb, Mesa)                   │  ~100 MB
├──────────────────────────────────────────────────────────┤
│  Layer 4: ROS-Gazebo Bridge (ros-humble-ros-gz)          │  ~200 MB
├──────────────────────────────────────────────────────────┤
│  Layer 3: Gazebo Harmonic (gz-sim8)                      │  ~500 MB
├──────────────────────────────────────────────────────────┤
│  Layer 2: OSRF Repository Setup                          │  < 1 MB
├──────────────────────────────────────────────────────────┤
│  Layer 1: Base (osrf/ros:humble-desktop-full)            │  ~2 GB
└──────────────────────────────────────────────────────────┘
Total Image Size: ~3 GB (compressed: ~1.2 GB)

Caching Strategy:
- Layers 1-2: Change rarely → Cache for weeks
- Layers 3-5: Change occasionally → Cache for days
- Layers 6-8: Change frequently → Rebuild each time
```

---

## Test Execution Flow

```
┌─────────────────────────────────────────────────────────────┐
│  colcon test                                                 │
└─────────────────────────────────────────────────────────────┘
                          │
                          ▼
        ┌─────────────────────────────────┐
        │  Discover Test Packages          │
        │  (packages with BUILD_TESTING)   │
        └─────────────────────────────────┘
                          │
         ┌────────────────┴────────────────┐
         │                                  │
         ▼                                  ▼
┌──────────────────┐            ┌──────────────────┐
│  Unit Tests      │            │  Integration     │
│  (pytest)        │            │  Tests           │
│  ✓ Fast (< 1s)   │            │  (launch_testing)│
│  ✓ No ROS        │            │  ✓ ROS nodes     │
│  ✓ Pure Python   │            │  ✓ Services      │
└──────────────────┘            │  ✓ Actions       │
         │                      └──────────────────┘
         │                                  │
         │                                  ▼
         │                      ┌──────────────────┐
         │                      │  Simulation      │
         │                      │  Tests           │
         │                      │  (Gazebo)        │
         │                      │  ✓ Headless      │
         │                      │  ✓ Physics       │
         │                      │  ✓ Sensors       │
         │                      └──────────────────┘
         │                                  │
         └────────────────┬─────────────────┘
                          │
                          ▼
        ┌─────────────────────────────────┐
        │  Generate JUnit XML Results      │
        │  build/*/test_results/*.xml      │
        └─────────────────────────────────┘
                          │
                          ▼
        ┌─────────────────────────────────┐
        │  colcon test-result --all        │
        │  • Count: pass/fail              │
        │  • Duration: total time          │
        │  • Summary: per package          │
        └─────────────────────────────────┘
```

---

## Caching Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  GitHub Actions Cache (10 GB limit per repo)                │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  Docker BuildKit Cache                               │   │
│  │  /tmp/.buildx-cache/                                 │   │
│  │  ┌─────────────────────────────────────────────┐     │   │
│  │  │  Key: buildx-{Dockerfile-hash}              │     │   │
│  │  │  Size: ~800 MB (compressed layers)          │     │   │
│  │  │  Hit Rate: 95% (if Dockerfile unchanged)    │     │   │
│  │  │  Speedup: 8-10 min → 30-60 sec              │     │   │
│  │  └─────────────────────────────────────────────┘     │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  APT Package Cache (inside Docker)                  │   │
│  │  /var/cache/apt/                                     │   │
│  │  ┌─────────────────────────────────────────────┐     │   │
│  │  │  Enabled via: --mount=type=cache            │     │   │
│  │  │  Size: ~300 MB                              │     │   │
│  │  │  Hit Rate: 80% (OS packages)                │     │   │
│  │  │  Speedup: 2-3 min → 10-20 sec               │     │   │
│  │  └─────────────────────────────────────────────┘     │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  colcon Build Cache (optional)                       │   │
│  │  install/                                             │   │
│  │  ┌─────────────────────────────────────────────┐     │   │
│  │  │  Key: colcon-{source-hash}                  │     │   │
│  │  │  Size: ~1 GB                                │     │   │
│  │  │  Hit Rate: 60% (incremental builds)         │     │   │
│  │  │  Speedup: 5-10 min → 30-60 sec              │     │   │
│  │  └─────────────────────────────────────────────┘     │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘

Total Speedup with All Caches:
  Without: 15-20 min
  With:    3-5 min
  Gain:    ~75% faster
```

---

## Parallel Execution (Matrix Strategy)

```
GitHub Actions Job: integration-tests
┌─────────────────────────────────────────────────────────┐
│  Matrix Strategy: 3 parallel jobs                       │
└─────────────────────────────────────────────────────────┘
                          │
         ┌────────────────┼────────────────┐
         │                │                │
         ▼                ▼                ▼
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│  Runner 1    │  │  Runner 2    │  │  Runner 3    │
│              │  │              │  │              │
│  Suite:      │  │  Suite:      │  │  Suite:      │
│  basic-pub-  │  │  services-   │  │  gazebo-     │
│  sub         │  │  actions     │  │  simulation  │
│              │  │              │  │              │
│  Timeout:    │  │  Timeout:    │  │  Timeout:    │
│  30s         │  │  45s         │  │  90s         │
│              │  │              │  │              │
│  Tests:      │  │  Tests:      │  │  Tests:      │
│  • talker    │  │  • add_ints  │  │  • spawn     │
│  • listener  │  │  • fibonacci │  │  • control   │
│              │  │              │  │  • sensors   │
└──────────────┘  └──────────────┘  └──────────────┘
         │                │                │
         └────────────────┼────────────────┘
                          │
                          ▼
        ┌─────────────────────────────────┐
        │  All jobs complete:              │
        │  • Upload artifacts              │
        │  • Publish results to GitHub     │
        │  • Report status                 │
        └─────────────────────────────────┘

Sequential: 30s + 45s + 90s = 165s (~3 min)
Parallel:   max(30s, 45s, 90s) = 90s (~1.5 min)
Speedup:    1.8x faster
```

---

## Data Flow

```
┌──────────────┐
│  Developer   │
└──────────────┘
       │
       │ git push
       ▼
┌──────────────┐     ┌─────────────────────────────────┐
│   GitHub     │────▶│  Trigger: ros2-ci.yml           │
│  Repository  │     └─────────────────────────────────┘
└──────────────┘                   │
                                   ▼
                    ┌──────────────────────────────────┐
                    │  GitHub Actions Runner           │
                    │  ┌────────────────────────────┐  │
                    │  │  1. Clone Repository       │  │
                    │  └────────────────────────────┘  │
                    │              │                    │
                    │              ▼                    │
                    │  ┌────────────────────────────┐  │
                    │  │  2. Build Docker Image     │  │
                    │  │     (Use cache if hit)     │  │
                    │  └────────────────────────────┘  │
                    │              │                    │
                    │              ▼                    │
                    │  ┌────────────────────────────┐  │
                    │  │  3. Run in Container:      │  │
                    │  │     • Mount source code    │  │
                    │  │     • colcon build         │  │
                    │  │     • Start Xvfb           │  │
                    │  │     • colcon test          │  │
                    │  └────────────────────────────┘  │
                    │              │                    │
                    │              ▼                    │
                    │  ┌────────────────────────────┐  │
                    │  │  4. Copy Results Out:      │  │
                    │  │     • test_results/*.xml   │  │
                    │  │     • coverage.xml         │  │
                    │  │     • build logs           │  │
                    │  └────────────────────────────┘  │
                    └──────────────────────────────────┘
                                   │
                                   ▼
                    ┌──────────────────────────────────┐
                    │  GitHub Services                 │
                    │  ┌────────────────────────────┐  │
                    │  │  • Artifact Storage        │  │
                    │  │  • Test Results UI         │  │
                    │  │  • Checks API              │  │
                    │  │  • Notifications           │  │
                    │  └────────────────────────────┘  │
                    └──────────────────────────────────┘
                                   │
                                   ▼
┌──────────────┐     ┌──────────────────────────────────┐
│  Developer   │◀────│  • Email notification            │
│  (Review)    │     │  • PR status update              │
└──────────────┘     │  • View results in GitHub UI     │
                     └──────────────────────────────────┘
```

---

## File System Layout (Inside Container)

```
/workspace/                         # Container working directory
├── src/                            # Source code (mounted from host)
│   └── humanoid_robotics/          # Your project
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── src/                    # C++ source
│       ├── scripts/                # Python scripts
│       ├── launch/                 # Launch files
│       └── test/                   # Test files
│           ├── unit/
│           ├── integration/
│           └── simulation/
│
├── build/                          # Build artifacts (auto-generated)
│   └── humanoid_robotics/
│       ├── test_results/           # JUnit XML files
│       │   ├── pytest.xml
│       │   └── gtest/*.xml
│       └── compile_commands.json
│
├── install/                        # Install artifacts (auto-generated)
│   └── humanoid_robotics/
│       ├── lib/
│       ├── share/
│       └── setup.bash              # Source this for ROS environment
│
├── log/                            # Log files (auto-generated)
│   └── latest_test/
│       └── humanoid_robotics/
│           ├── stdout.log
│           └── stderr.log
│
└── test_results/                   # Copied results (for artifact upload)
    ├── pytest.xml
    ├── gtest/
    └── coverage/

/opt/ros/humble/                    # ROS 2 installation (pre-installed)
├── setup.bash                      # Source for ROS environment
├── lib/
└── share/

/usr/share/gazebo/                  # Gazebo resources (pre-installed)
├── models/
└── worlds/

~/.gz/                              # Gazebo user data
└── log/
    └── gzserver.log
```

---

## Network Communication (ROS Topics)

```
┌────────────────────────────────────────────────────────────┐
│  ROS 2 DDS Network (inside container)                      │
│                                                              │
│  ┌──────────────┐    /chatter     ┌──────────────┐         │
│  │   Talker     │ ───────────────▶│   Listener   │         │
│  │   (pub)      │  (std_msgs/String)│   (sub)    │         │
│  └──────────────┘                 └──────────────┘         │
│                                                              │
│  ┌──────────────┐  /add_two_ints  ┌──────────────┐         │
│  │   Client     │◀────────────────│   Server     │         │
│  │              │  (AddTwoInts)   │              │         │
│  └──────────────┘                 └──────────────┘         │
│                                                              │
│  ┌──────────────┐    /cmd_vel     ┌──────────────┐         │
│  │  Controller  │ ───────────────▶│   Gazebo     │         │
│  │              │  (Twist)        │   Robot      │         │
│  └──────────────┘                 └──────────────┘         │
│                         │                 │                 │
│                         │    /odom        │                 │
│                         └─────────────────┘                 │
│                          (Odometry)                          │
│                                                              │
│  ┌──────────────┐    /clock       ┌──────────────┐         │
│  │   Gazebo     │ ───────────────▶│  ROS Nodes   │         │
│  │   Server     │  (Clock)        │  (all)       │         │
│  └──────────────┘                 └──────────────┘         │
└────────────────────────────────────────────────────────────┘

Note: All communication stays inside container (localhost)
      No external network access required for tests
```

---

## Gazebo Rendering Pipeline (Headless)

```
┌────────────────────────────────────────────────────────────┐
│  Gazebo Physics & Rendering (Headless Mode)                │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Xvfb (X Virtual Frame Buffer)                       │  │
│  │  Port: :99                                            │  │
│  │  Resolution: 1920x1080x24                            │  │
│  │  ┌────────────────────────────────────────────────┐  │  │
│  │  │  Virtual Display (in memory, no GPU)           │  │  │
│  │  └────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
│                          │                                  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Mesa (Software Rendering)                           │  │
│  │  LIBGL_ALWAYS_SOFTWARE=1                             │  │
│  │  ┌────────────────────────────────────────────────┐  │  │
│  │  │  OpenGL 3.3 (Software, CPU-based)              │  │  │
│  │  │  • No GPU required                              │  │  │
│  │  │  • Slower but reliable                          │  │  │
│  │  └────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
│                          │                                  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Gazebo Rendering (OGRE2)                            │  │
│  │  ┌────────────────────────────────────────────────┐  │  │
│  │  │  Scene Graph:                                   │  │  │
│  │  │  • Robot models                                 │  │  │
│  │  │  • Environment                                  │  │  │
│  │  │  • Lights, shadows                              │  │  │
│  │  └────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
│                          │                                  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Gazebo Physics (DART/Bullet)                        │  │
│  │  ┌────────────────────────────────────────────────┐  │  │
│  │  │  • Collision detection                          │  │  │
│  │  │  • Gravity simulation                           │  │  │
│  │  │  • Joint dynamics                               │  │  │
│  │  │  • Contact forces                               │  │  │
│  │  └────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
│                          │                                  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  ROS-Gazebo Bridge                                   │  │
│  │  ┌────────────────────────────────────────────────┐  │  │
│  │  │  /cmd_vel → Gazebo joint commands              │  │  │
│  │  │  Gazebo state → /odom, /joint_states           │  │  │
│  │  │  /clock → All ROS nodes (simulation time)      │  │  │
│  │  └────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘

Performance:
  • Physics update: 1000 Hz (real-time factor: 0.5-1.0)
  • Rendering: Disabled (headless) or 10 Hz (if needed)
  • CPU usage: ~50% (single-threaded physics)
```

---

## Timeline (Typical CI Run with Caching)

```
Time      Event                                   Duration
────────────────────────────────────────────────────────────
00:00     Workflow triggered (git push)
00:05     Runner assigned                         5s
00:10     Checkout code                           5s
00:15     Setup Docker Buildx                     5s
00:20     Restore Docker cache (hit)              5s
00:25     Build Docker image (cached)             40s
01:05     Save Docker cache (new)                 10s
01:15     Run colcon build                        2m 30s
03:45     Start Xvfb                              2s
03:47     Run colcon test (unit)                  30s
04:17     Run colcon test (integration)           1m 30s
05:47     Run colcon test (simulation)            2m
07:47     Generate test results                   5s
07:52     Upload test results                     10s
08:02     Publish to GitHub UI                    5s
08:07     Start matrix jobs (parallel)
08:10     • basic-pub-sub (30s)
08:15     • services-actions (45s)
08:30     • gazebo-simulation (90s)
09:40     Matrix jobs complete                    1m 30s
09:45     Upload matrix artifacts                 5s
09:50     Workflow complete
────────────────────────────────────────────────────────────
Total:    9 minutes 50 seconds

Without caching: ~25 minutes
Speedup: 2.5x faster
```

---

## Resource Usage

```
┌────────────────────────────────────────────────────────────┐
│  GitHub Actions Runner Resources                           │
│  (ubuntu-22.04 runner)                                      │
│                                                              │
│  CPU: 2 cores @ 2.6 GHz                                     │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  ████████████████░░░░░░░░░░░░░░░░░░  45% (avg)       │  │
│  │  Peak: 90% (colcon build)                            │  │
│  │  Idle: 5%  (uploading artifacts)                     │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  Memory: 7 GB                                               │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  ████████████████████████░░░░░░░░░  3.5 GB (avg)     │  │
│  │  Peak: 5 GB (Gazebo simulation)                      │  │
│  │  Base: 2 GB (Docker + ROS)                           │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  Disk: 14 GB SSD                                            │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  ██████████████████████░░░░░░░░░░  8 GB (used)       │  │
│  │  Docker image: 3 GB                                  │  │
│  │  Build artifacts: 2 GB                               │  │
│  │  Cache: 3 GB                                         │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  Network: 1 Gbps                                            │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Download: ~500 MB (Docker layers, first run)        │  │
│  │  Upload: ~50 MB (test artifacts)                     │  │
│  └──────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘

Cost: Free for public repos (2000 min/month)
      $0.008/min for private repos
      This workflow: ~$0.08 per run (private repo)
```

---

This architecture documentation provides visual references for understanding the complete CI/CD system!
