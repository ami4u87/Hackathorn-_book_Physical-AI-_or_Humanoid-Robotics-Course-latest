#!/bin/bash
# Local CI test runner - simulates GitHub Actions locally
# Usage: ./local_ci_test.sh [--build-only] [--test-only]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

BUILD_ONLY=false
TEST_ONLY=false

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --build-only)
      BUILD_ONLY=true
      shift
      ;;
    --test-only)
      TEST_ONLY=true
      shift
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

cd "$PROJECT_ROOT"

echo "================================================"
echo "Local CI Test Runner"
echo "Project: $(basename "$PROJECT_ROOT")"
echo "================================================"

# Build Docker image
if [ "$TEST_ONLY" = false ]; then
  echo ""
  echo "Step 1: Building Docker image..."
  docker build -f Dockerfile.ros2-humble-gazebo -t ros2-humble-gazebo-ci:latest .
fi

if [ "$BUILD_ONLY" = false ]; then
  # Run build
  echo ""
  echo "Step 2: Building ROS 2 workspace..."
  docker run --rm \
    -v "$PROJECT_ROOT":/workspace/src/project \
    -w /workspace \
    ros2-humble-gazebo-ci:latest \
    bash -c "
      set -e
      source /opt/ros/humble/setup.bash
      colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_direct+ \
        --base-paths src/project
    "

  # Run tests
  echo ""
  echo "Step 3: Running tests with headless Gazebo..."
  docker run --rm \
    -v "$PROJECT_ROOT":/workspace/src/project \
    -v "$PROJECT_ROOT/test_results":/workspace/test_results \
    -w /workspace \
    -e DISPLAY=:99 \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    ros2-humble-gazebo-ci:latest \
    bash -c "
      set -e

      # Start Xvfb
      Xvfb :99 -screen 0 1920x1080x24 &
      export DISPLAY=:99
      sleep 2

      # Source environments
      source /opt/ros/humble/setup.bash
      source install/setup.bash

      # Run tests
      colcon test \
        --event-handlers console_direct+ \
        --return-code-on-test-failure \
        --pytest-args -v \
        --base-paths src/project

      # Generate results
      colcon test-result --all --verbose

      # Copy results
      mkdir -p /workspace/test_results
      cp -r build/*/test_results/* /workspace/test_results/ || true
    "

  echo ""
  echo "================================================"
  echo "CI Test Completed Successfully!"
  echo "Test results: $PROJECT_ROOT/test_results"
  echo "================================================"
fi
