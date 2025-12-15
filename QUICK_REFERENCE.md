# ROS 2 CI/CD Quick Reference

Fast reference for common CI/CD operations with ROS 2 Humble + Gazebo Harmonic.

## Quick Start

### Local Testing (3 commands)
```bash
# 1. Build Docker image
docker build -f Dockerfile.ros2-humble-gazebo -t ros2-ci:latest .

# 2. Run build
docker run --rm -v $(pwd):/workspace/src/project -w /workspace ros2-ci:latest \
  bash -c "source /opt/ros/humble/setup.bash && colcon build"

# 3. Run tests
docker run --rm -v $(pwd):/workspace/src/project -w /workspace ros2-ci:latest \
  bash -c "Xvfb :99 & sleep 2 && export DISPLAY=:99 && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test"
```

### Using Scripts
```bash
# Full CI pipeline locally
chmod +x scripts/local_ci_test.sh
./scripts/local_ci_test.sh

# Run specific test suite
chmod +x scripts/run_integration_tests.sh
./scripts/run_integration_tests.sh --suite gazebo-simulation --timeout 90
```

### Using Docker Compose
```bash
# Development
docker-compose up -d ros2-dev
docker exec -it ros2-humble-dev bash

# Testing
docker-compose up ros2-test

# Headless Gazebo
docker-compose up gazebo-headless
```

---

## Environment Variables

### Required for Headless Gazebo
```bash
export DISPLAY=:99                          # Virtual display
export QT_X11_NO_MITSHM=1                   # Qt compatibility
export LIBGL_ALWAYS_SOFTWARE=1              # Software rendering
export GZ_SIM_RESOURCE_PATH=/usr/share/gazebo
```

### Optional Performance Tweaks
```bash
export MESA_GL_VERSION_OVERRIDE=3.3         # OpenGL version
export LIBGL_ALWAYS_INDIRECT=0              # Direct rendering
export GZ_SIM_SERVER_CONFIG_PATH=/path/to/config
```

---

## Common Commands

### colcon Build
```bash
# Basic build
colcon build

# With verbose output
colcon build --event-handlers console_direct+

# Specific packages only
colcon build --packages-select my_package

# With compile commands (for IDE)
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Debug build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Release build (faster)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# With coverage
colcon build --cmake-args -DCMAKE_CXX_FLAGS='--coverage'

# Parallel jobs
colcon build --parallel-workers 4

# Symlink install (faster iteration)
colcon build --symlink-install
```

### colcon Test
```bash
# Run all tests
colcon test

# Verbose output
colcon test --event-handlers console_direct+

# Specific package
colcon test --packages-select my_package

# Return error code on failure
colcon test --return-code-on-test-failure

# With pytest options
colcon test --pytest-args -v -s

# Run specific test
colcon test --pytest-args -k test_name

# With timeout
colcon test --pytest-args --timeout=30

# Parallel testing
colcon test --parallel-workers 4

# Re-run failed tests only
colcon test --packages-select-test-failures
```

### Test Results
```bash
# Show all test results
colcon test-result --all

# Verbose output
colcon test-result --all --verbose

# Show only failed tests
colcon test-result --all | grep -A5 "failed"

# Find test result files
find build -name "*.xml"
```

---

## Gazebo Commands

### Headless Gazebo
```bash
# Start Gazebo server only (no GUI)
gz sim -s world.sdf

# Run with iterations limit (auto-stop)
gz sim -s -r --iterations 1000 world.sdf

# Verbose output
gz sim -s -v 4 world.sdf

# Empty world (fastest)
gz sim -s empty.sdf
```

### Gazebo + ROS Bridge
```bash
# Start bridge
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

# Multiple topics
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry
```

### Gazebo Troubleshooting
```bash
# Check Gazebo version
gz sim --version

# List available worlds
gz model --list

# Check rendering backend
glxinfo | grep "OpenGL version"

# Gazebo logs
tail -f ~/.gz/log/*/gzserver.log
```

---

## Docker Commands

### Build
```bash
# Basic build
docker build -f Dockerfile.ros2-humble-gazebo -t ros2-ci .

# No cache (clean build)
docker build --no-cache -f Dockerfile.ros2-humble-gazebo -t ros2-ci .

# With build args
docker build --build-arg ROS_DISTRO=humble -t ros2-ci .
```

### Run
```bash
# Interactive
docker run -it --rm ros2-ci bash

# Mount workspace
docker run -it --rm -v $(pwd):/workspace/src/project ros2-ci

# With display (for GUI)
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros2-ci

# Run command directly
docker run --rm ros2-ci bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
```

### Cleanup
```bash
# Remove stopped containers
docker container prune

# Remove unused images
docker image prune

# Remove all (dangerous!)
docker system prune -a
```

---

## GitHub Actions

### Trigger Workflow
```bash
# Push to branch
git push origin feature-branch

# Manual trigger (via web UI or CLI)
gh workflow run ros2-ci.yml

# With inputs
gh workflow run ros2-ci.yml -f package_name=my_package
```

### View Results
```bash
# List runs
gh run list --workflow=ros2-ci.yml

# View specific run
gh run view <run-id>

# Watch run in real-time
gh run watch <run-id>

# Download artifacts
gh run download <run-id> -n test-results
```

### Debug Failed Run
```bash
# View logs
gh run view <run-id> --log

# View logs for specific job
gh run view <run-id> --log --job=build-and-test

# Re-run failed jobs
gh run rerun <run-id> --failed
```

---

## Testing with act (Local GitHub Actions)

### Installation
```bash
# Linux/Mac
curl https://raw.githubusercontent.com/nektos/act/master/install.sh | sudo bash

# Or with package manager
brew install act  # Mac
```

### Usage
```bash
# List workflows
act -l

# Run default workflow
act

# Run specific job
act -j build-and-test

# Run with secrets
act -s GITHUB_TOKEN=ghp_xxx

# Use specific Docker image
act -P ubuntu-22.04=ros2-ci:latest

# Dry run (don't execute)
act -n

# Verbose output
act -v
```

---

## Troubleshooting

### "Cannot open display :99"
```bash
# Check if Xvfb is running
ps aux | grep Xvfb

# Kill existing Xvfb
pkill Xvfb

# Restart with longer delay
Xvfb :99 -screen 0 1920x1080x24 &
sleep 5  # Increase wait time
```

### "Gazebo crashes with segfault"
```bash
# Force software rendering
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# Use simpler world
gz sim -s empty.sdf

# Check logs
cat ~/.gz/log/*/gzserver.log
```

### "colcon build fails with missing dependencies"
```bash
# Update rosdep
rosdep update

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Check for missing packages
rosdep check --from-paths src --ignore-src
```

### "Tests timeout in CI"
```bash
# Increase timeout in workflow
timeout-minutes: 90

# Add timeout to specific test
timeout 60s ros2 launch test.launch.py

# Check for infinite loops
ros2 topic echo /test_status --timeout 5
```

### "Docker build is slow"
```bash
# Enable BuildKit
export DOCKER_BUILDKIT=1

# Use cache
docker build --cache-from ros2-ci:latest -t ros2-ci:latest .

# Reduce layer count
# Combine RUN commands with && \
```

---

## Performance Tips

### Faster Builds
```bash
# Use ccache
apt-get install ccache
export PATH=/usr/lib/ccache:$PATH

# Parallel builds
colcon build --parallel-workers $(nproc)

# Symlink install (no copy)
colcon build --symlink-install

# Skip tests during build
colcon build --cmake-args -DBUILD_TESTING=OFF
```

### Faster Tests
```bash
# Run only changed packages
colcon test --packages-select-test-failures

# Parallel tests
colcon test --parallel-workers 4

# Skip slow tests
colcon test --pytest-args -m "not slow"

# Short timeouts
colcon test --pytest-args --timeout=10
```

### Docker Optimization
```dockerfile
# Multi-stage build
FROM ros:humble as builder
RUN apt-get update && apt-get install -y ...

FROM ros:humble-slim
COPY --from=builder /opt/ros /opt/ros

# Use .dockerignore
# Add: .git, *.log, build/, install/, log/

# Minimize layers
RUN apt-get update && apt-get install -y \
    pkg1 pkg2 pkg3 \
    && rm -rf /var/lib/apt/lists/*
```

---

## Useful Resources

### Official Docs
- ROS 2 Humble: https://docs.ros.org/en/humble/
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic
- colcon: https://colcon.readthedocs.io/

### Tools
- act: https://github.com/nektos/act
- rocker: https://github.com/osrf/rocker
- ros_testing: https://github.com/ros-industrial/ros-industrial-ci

### Example Projects
- ros2/examples: https://github.com/ros2/examples
- gazebosim/ros_gz: https://github.com/gazebosim/ros_gz

---

## Common File Locations

```
Workspace Structure:
/workspace/
├── src/                    # Source code
├── build/                  # Build artifacts
│   └── <package>/
│       └── test_results/   # Test XMLs
├── install/                # Install artifacts
├── log/                    # Build/test logs
└── test_results/           # Copied test results

Docker Paths:
/opt/ros/humble/            # ROS installation
/usr/share/gazebo/          # Gazebo resources
~/.gz/                      # Gazebo user data

Test Results:
build/<package>/test_results/*.xml
build/<package>/pytest.xml
log/latest_test/<package>/stdout.log
```

---

## Emergency Commands

### Kill Everything
```bash
# Stop all ROS nodes
pkill -9 -f ros2

# Stop Gazebo
pkill -9 -f gz

# Stop Xvfb
pkill -9 -f Xvfb

# Stop Docker containers
docker stop $(docker ps -q)
```

### Clean Workspace
```bash
# Remove build artifacts
rm -rf build install log

# Clean colcon cache
rm -rf ~/.colcon

# Reset ROS environment
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
```

### Check Status
```bash
# ROS nodes
ros2 node list

# Topics
ros2 topic list

# Services
ros2 service list

# Gazebo status
gz topic -l
```

---

**Last Updated**: 2025-12-15
**ROS Version**: Humble (Ubuntu 22.04)
**Gazebo Version**: Harmonic (gz-sim8)
