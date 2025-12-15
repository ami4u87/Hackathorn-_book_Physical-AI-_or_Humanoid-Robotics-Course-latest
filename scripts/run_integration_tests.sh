#!/bin/bash
# Integration test runner for ROS 2 + Gazebo CI/CD
# Usage: ./run_integration_tests.sh --suite <suite-name> --timeout <seconds> --output <dir>

set -e

# Default values
SUITE="all"
TIMEOUT=60
OUTPUT_DIR="./test_results"
VERBOSE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --suite)
      SUITE="$2"
      shift 2
      ;;
    --timeout)
      TIMEOUT="$2"
      shift 2
      ;;
    --output)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --verbose|-v)
      VERBOSE=true
      shift
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Set up logging
LOG_FILE="$OUTPUT_DIR/${SUITE}_$(date +%Y%m%d_%H%M%S).log"
exec 1> >(tee -a "$LOG_FILE")
exec 2>&1

echo "================================================"
echo "ROS 2 Integration Test Suite: $SUITE"
echo "Timeout: ${TIMEOUT}s"
echo "Output: $OUTPUT_DIR"
echo "Started: $(date)"
echo "================================================"

# Function to run test with timeout
run_test_with_timeout() {
  local test_name=$1
  local test_command=$2
  local test_timeout=$3

  echo ""
  echo "Running test: $test_name"
  echo "Command: $test_command"
  echo "Timeout: ${test_timeout}s"

  timeout --preserve-status ${test_timeout}s bash -c "$test_command" || {
    local exit_code=$?
    if [ $exit_code -eq 124 ]; then
      echo "ERROR: Test '$test_name' timed out after ${test_timeout}s"
      return 124
    else
      echo "ERROR: Test '$test_name' failed with exit code $exit_code"
      return $exit_code
    fi
  }

  echo "SUCCESS: Test '$test_name' passed"
  return 0
}

# Test suite definitions
case $SUITE in
  basic-pub-sub)
    echo "Running basic publisher/subscriber tests..."
    run_test_with_timeout \
      "basic-talker-listener" \
      "ros2 launch src/humanoid_robotics/launch/test_basic_pub_sub.launch.py" \
      30
    ;;

  services-actions)
    echo "Running service and action tests..."
    run_test_with_timeout \
      "service-client-test" \
      "ros2 launch src/humanoid_robotics/launch/test_services.launch.py" \
      45

    run_test_with_timeout \
      "action-client-test" \
      "ros2 launch src/humanoid_robotics/launch/test_actions.launch.py" \
      45
    ;;

  gazebo-simulation)
    echo "Running Gazebo simulation tests..."

    # Test 1: Launch Gazebo headless
    run_test_with_timeout \
      "gazebo-launch" \
      "gz sim -s -r -v 4 --iterations 100 empty.sdf" \
      30

    # Test 2: Spawn robot model
    run_test_with_timeout \
      "robot-spawn" \
      "ros2 launch src/humanoid_robotics/launch/test_gazebo_spawn.launch.py" \
      60

    # Test 3: Robot control test
    run_test_with_timeout \
      "robot-control" \
      "ros2 launch src/humanoid_robotics/launch/test_gazebo_control.launch.py" \
      90
    ;;

  all)
    echo "Running all test suites..."
    $0 --suite basic-pub-sub --timeout 30 --output "$OUTPUT_DIR"
    $0 --suite services-actions --timeout 45 --output "$OUTPUT_DIR"
    $0 --suite gazebo-simulation --timeout 90 --output "$OUTPUT_DIR"
    ;;

  *)
    echo "ERROR: Unknown test suite: $SUITE"
    echo "Available suites: basic-pub-sub, services-actions, gazebo-simulation, all"
    exit 1
    ;;
esac

TEST_EXIT_CODE=$?

echo ""
echo "================================================"
echo "Test Suite Completed: $SUITE"
echo "Finished: $(date)"
echo "Exit Code: $TEST_EXIT_CODE"
echo "Log file: $LOG_FILE"
echo "================================================"

exit $TEST_EXIT_CODE
