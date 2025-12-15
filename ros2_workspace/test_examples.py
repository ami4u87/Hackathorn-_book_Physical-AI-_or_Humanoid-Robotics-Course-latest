#!/usr/bin/env python3

"""
Automated test script to validate all ROS 2 examples in the Physical AI course.

This script tests that all example packages build correctly and can be executed
without errors. It runs a series of tests to validate the functionality of each
example package.
"""

import os
import sys
import subprocess
import time
import signal
from pathlib import Path
from typing import Tuple, List, Dict, Any


def run_command(cmd: str, timeout: int = 10, check_output: bool = False) -> Tuple[bool, str]:
    """
    Run a command and return the result.

    Args:
        cmd: Command string to execute
        timeout: Maximum time in seconds to wait for command completion
        check_output: Whether to capture and return command output

    Returns:
        Tuple of (success: bool, output: str) where success indicates if the command
        succeeded and output contains stdout if check_output=True
    """
    try:
        if check_output:
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=timeout
            )
        else:
            result = subprocess.run(
                cmd,
                shell=True,
                timeout=timeout
            )
        return result.returncode == 0, result.stdout if check_output else ""
    except subprocess.TimeoutExpired:
        return False, "Command timed out"
    except Exception as e:
        return False, str(e)


def test_package_build(package_name: str) -> bool:
    """
    Test that a package builds successfully.

    Args:
        package_name: Name of the ROS 2 package to build

    Returns:
        True if the package builds successfully, False otherwise
    """
    print(f"Testing build for {package_name}...")

    # Change to workspace directory
    os.chdir('/ros2_ws')

    # Build only this package
    success, output = run_command(f"colcon build --packages-select {package_name}", timeout=60)

    if success:
        print(f"✓ {package_name} builds successfully")
        return True
    else:
        print(f"✗ {package_name} build failed")
        print(f"Error: {output}")
        return False


def test_node_execution(package_name: str, executable_name: str, timeout: int = 5) -> bool:
    """
    Test that a node can be executed without errors.

    Args:
        package_name: Name of the ROS 2 package
        executable_name: Name of the executable to run
        timeout: Time in seconds to wait before stopping the node

    Returns:
        True if the node runs without immediate errors, False otherwise
    """
    print(f"Testing execution for {package_name}/{executable_name}...")

    # Source the workspace
    os.environ['AMENT_PREFIX_PATH'] = '/ros2_ws/install'
    os.environ['PYTHONPATH'] = '/ros2_ws/install/lib/python3.10/site-packages'

    # Start the node in the background
    try:
        process = subprocess.Popen(
            ['ros2', 'run', package_name, executable_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # Wait for a short time to see if it runs without error
        time.sleep(timeout)

        # Check if the process is still running (good sign) or if it exited with error
        if process.poll() is None:
            # Process is still running, which is good
            process.send_signal(signal.SIGINT)  # Send Ctrl+C to stop it
            try:
                process.wait(timeout=2)  # Wait for graceful shutdown
            except subprocess.TimeoutExpired:
                process.kill()  # Force kill if it doesn't stop
            print(f"✓ {package_name}/{executable_name} runs without immediate errors")
            return True
        else:
            # Process already exited, check the return code
            stdout, stderr = process.communicate()
            if process.returncode == 0:
                print(f"✓ {package_name}/{executable_name} executed successfully")
                return True
            else:
                print(f"✗ {package_name}/{executable_name} exited with error code {process.returncode}")
                print(f"Stderr: {stderr.decode()}")
                return False
    except Exception as e:
        print(f"✗ {package_name}/{executable_name} failed to start: {str(e)}")
        return False


def test_launch_file(package_name: str, launch_file: str) -> bool:
    """
    Test that a launch file can be executed without errors.

    Args:
        package_name: Name of the ROS 2 package
        launch_file: Name of the launch file to execute

    Returns:
        True if the launch file runs without immediate errors, False otherwise
    """
    print(f"Testing launch file {package_name}/{launch_file}...")

    try:
        process = subprocess.Popen(
            ['ros2', 'launch', package_name, launch_file],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # Wait briefly to see if it starts without error
        time.sleep(3)

        # Check if the process is still running
        if process.poll() is None:
            # Process is still running, which is good
            process.send_signal(signal.SIGINT)  # Send Ctrl+C to stop it
            try:
                process.wait(timeout=2)  # Wait for graceful shutdown
            except subprocess.TimeoutExpired:
                process.kill()  # Force kill if it doesn't stop
            print(f"✓ {package_name}/{launch_file} runs without immediate errors")
            return True
        else:
            # Process already exited with error
            stdout, stderr = process.communicate()
            print(f"✗ {package_name}/{launch_file} exited with error")
            print(f"Stderr: {stderr.decode()}")
            return False
    except Exception as e:
        print(f"✗ {package_name}/{launch_file} failed to start: {str(e)}")
        return False


def test_ros2_installation() -> bool:
    """
    Test that ROS 2 is properly installed and sourced.

    Returns:
        True if ROS 2 is properly installed and available, False otherwise
    """
    print("Testing ROS 2 installation...")

    success, output = run_command("ros2 --version", check_output=True)

    if success and output.strip().startswith("ros2"):
        print(f"✓ ROS 2 installation found: {output.strip()}")
        return True
    else:
        print("✗ ROS 2 installation not found or not properly sourced")
        return False


def test_workspace_setup() -> bool:
    """
    Test that the workspace is properly set up.

    Returns:
        True if the workspace directories exist and are properly configured, False otherwise
    """
    print("Testing workspace setup...")

    # Check if workspace directory exists
    workspace_path = Path("/ros2_ws")
    if not workspace_path.exists():
        print("✗ Workspace directory does not exist")
        return False

    # Check if src directory exists
    src_path = workspace_path / "src"
    if not src_path.exists():
        print("✗ Source directory does not exist")
        return False

    # Check if install directory exists (after build)
    install_path = workspace_path / "install"
    if not install_path.exists():
        print("✗ Install directory does not exist (workspace may not be built)")
        return False

    print("✓ Workspace is properly set up")
    return True


def main() -> int:
    """
    Main test function that orchestrates all validation tests.

    Returns:
        Exit code: 0 if all tests pass, 1 if any test fails
    """
    print("Starting ROS 2 Examples Validation Tests...")
    print("=" * 50)

    # Initialize results
    all_tests_passed = True
    test_results = []

    # Test 1: ROS 2 installation
    result = test_ros2_installation()
    test_results.append(("ROS 2 Installation", result))
    if not result:
        all_tests_passed = False

    # Test 2: Workspace setup
    result = test_workspace_setup()
    test_results.append(("Workspace Setup", result))
    if not result:
        all_tests_passed = False

    # Define the packages to test
    packages_to_test = [
        {
            "name": "publisher_example",
            "executables": ["publisher_node"],
            "launch_files": []
        },
        {
            "name": "subscriber_example",
            "executables": ["subscriber_node"],
            "launch_files": []
        },
        {
            "name": "service_example",
            "executables": ["service_server", "service_client"],
            "launch_files": []
        },
        {
            "name": "action_example",
            "executables": ["action_server", "action_client"],
            "launch_files": []
        },
        {
            "name": "param_example",
            "executables": ["param_node"],
            "launch_files": []
        },
        {
            "name": "launch_example",
            "executables": ["launch_node"],
            "launch_files": ["simple_launch.py", "simple_launch.xml"]
        },
        {
            "name": "tf2_example",
            "executables": ["tf2_broadcaster", "tf2_listener"],
            "launch_files": ["tf2_example_launch.py"]
        }
    ]

    # Test each package
    for pkg_info in packages_to_test:
        pkg_name = pkg_info["name"]

        # Test build
        build_result = test_package_build(pkg_name)
        test_results.append((f"{pkg_name} Build", build_result))
        if not build_result:
            all_tests_passed = False
            continue  # Skip execution tests if build fails

        # Test executables
        for executable in pkg_info["executables"]:
            exec_result = test_node_execution(pkg_name, executable)
            test_results.append((f"{pkg_name}/{executable}", exec_result))
            if not exec_result:
                all_tests_passed = False

        # Test launch files
        for launch_file in pkg_info["launch_files"]:
            launch_result = test_launch_file(pkg_name, launch_file)
            test_results.append((f"{pkg_name}/{launch_file}", launch_result))
            if not launch_result:
                all_tests_passed = False

    # Print results summary
    print("\n" + "=" * 50)
    print("TEST RESULTS SUMMARY:")
    print("=" * 50)

    passed_count = 0
    total_count = len(test_results)

    for test_name, result in test_results:
        status = "PASS" if result else "FAIL"
        icon = "✓" if result else "✗"
        print(f"{icon} {test_name}: {status}")
        if result:
            passed_count += 1

    print(f"\nOverall Result: {passed_count}/{total_count} tests passed")

    if all_tests_passed:
        print("🎉 All tests passed! The ROS 2 examples are working correctly.")
        return 0
    else:
        print("❌ Some tests failed. Please check the output above for details.")
        return 1


if __name__ == "__main__":
    # Change to the workspace directory
    original_dir = os.getcwd()
    try:
        os.chdir('/ros2_ws')
        exit_code = main()
        sys.exit(exit_code)
    except Exception as e:
        print(f"Error running tests: {str(e)}")
        sys.exit(1)
    finally:
        os.chdir(original_dir)