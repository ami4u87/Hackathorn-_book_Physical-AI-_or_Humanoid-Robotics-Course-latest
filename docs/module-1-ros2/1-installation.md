# Installation

This section covers the installation of ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS. ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software that provides services such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

## Prerequisites

Before installing ROS 2, ensure your system meets the following requirements:

- Ubuntu 22.04 LTS (Jammy Jellyfish)
- 64-bit processor
- At least 4GB RAM (8GB recommended)
- At least 10GB free disk space

## Installation Steps

### 1. Set up your sources.list

Ensure that the Ubuntu universe repository is enabled by running:

```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

### 2. Add the ROS 2 GPG key and repository

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. Install ROS 2 packages

Update your apt repository and install the ROS 2 desktop package:

```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

This will install the full desktop environment which includes ROS, development tools, and GUI tools.

### 4. Install additional dependencies

For Python development with ROS 2, install the colcon build system:

```bash
sudo apt install -y python3-colcon-common-extensions
```

Install rosdep for managing system dependencies:

```bash
sudo apt install -y python3-rosdep2
```

Initialize rosdep:

```bash
sudo rosdep init
rosdep update
```

### 5. Source ROS 2 environment

To use ROS 2, you need to source the setup script. Add this line to your `~/.bashrc` file:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Verify Installation

Test that ROS 2 is properly installed by running:

```bash
ros2 --version
```

You should see output similar to `ros2 humble`, indicating that ROS 2 Humble is installed and accessible.

You can also run a simple test:

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2 (in a new terminal)
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

If you see messages being published by the talker and received by the listener, your installation is successful.

## Setting up a Workspace

Now that ROS 2 is installed, let's create a workspace for our projects:

```bash
# Create the workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build
```

After building, source the workspace:

```bash
source install/setup.bash
```

## Troubleshooting

### Common Issues:

1. **Package not found**: If `apt` cannot find the ROS 2 packages, make sure your Ubuntu version is supported and the repository URL is correct.

2. **Permission denied**: If you encounter permission issues, make sure you're using `sudo` for system-level installations.

3. **Python package conflicts**: If you have multiple Python environments, ensure you're installing packages in the correct environment.

### Useful Commands:

- `source /opt/ros/humble/setup.bash` - Source the ROS 2 environment
- `ros2 run <package_name> <executable>` - Run a ROS 2 executable
- `ros2 node list` - List active nodes
- `ros2 topic list` - List active topics

## Next Steps

With ROS 2 installed, you're ready to move on to learning about the publisher-subscriber pattern in the next section. This fundamental concept will form the basis for communication between different parts of your robotic system.