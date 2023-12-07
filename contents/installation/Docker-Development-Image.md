---
last_modified_date: 29/07/2021
layout: default
title: Docker Environment
nav_order: 3
parent: Installation
---

This guide explains how to set up a Docker-based development environment for building and developing Dave repository and related software.

## Step 1: [Install Dependencies]
Follow these steps to install required utilities on your host system.

### [Install Docker] (https://field-robotics-lab.github.io/dave.doc/contents/installation/System-Requirements/#docker) 

### Install Rocker
Check pip version with, `pip --version`, if it's based on python version 2, then install pip3 with `sudo apt install python3-pip`

- Install Rocker (the Open Robotics Docker CLI)
```bash
# Install rocker
pip3 install rocker
# Double check rocker installation to be the latest version
pip3 install --force-reinstall git+https://github.com/osrf/rocker.git@main
# Add path of pip3 installation to PATH
echo "export PATH=\"`python3 -m site --user-base`/bin:\$PATH\"" >> ~/.bashrc
# Load new PATH
source ~/.bashrc
# Check rocker version
rocker --verion
```

## Step 2: Build and run
Once you've installed the dependencies, you can build your Docker development environment with the following commands:
```bash
# Start from wherever you want (Recommend your home directory)
# Clone dockwater package which contains configurations to run Dave in Docker environment with Rocker CLI
git clone https://github.com/Field-Robotics-Lab/dockwater.git
cd dockwater
# Build the Docker image for the ROS Noetic environment (this may take a while, upto 20 or more minutes)
./build.bash noetic
# Run the Docker image for the ROS Noetic environment (this may take a while, upto 5 minutes)
./run.bash dockwater:noetic
# When done, you are now in a Docker container with the ROS Noetic environment running Ubuntu 22.04 Focal LTS. Check with,
lsb_release -a
# You can now run ROS commands, e.g.
rosversion -d
# You can also run Gazebo, e.g.
gazebo --version
```
If the above is successful, you should end up with a command prompt opened into the Docker container. Your user information will be the same and your home directory will be mounted and accessible within the container.

### OPTIONAL: GPU Multibeam sonar
```diff
- DO NOT INCLUDE THIS if you are not using multibeam sonar.
- It require CUDA Library and NVIDIA driver along with the NVIDIA graphics card that supports CUDA feature.
```
For instructions and plugin details : [Multibeam Forward-Looking Sonar](/dave.doc/contents/dave_sensors/Multibeam-Forward-Looking-Sonar)
```bash
# Clone docker package with CUDA support
# if you have already cloned dockwater package, skip this step
git clone -b cuda-dev https://github.com/Field-Robotics-Lab/dockwater.git
cd dockwater
# Change to cuda-dev branch which contains configurations to run Dave in Docker environment with CUDA support
git checkout cuda-dev
# Build the Docker image for the ROS Noetic environment (this may take a while, upto 30 or more minutes. It takes long to download and install CUDA libraries)
./build.bash noetic
# Run the Docker image for the ROS Noetic environment  (this may take a while, upto 5 minutes)
./run.bash -c dockwater:noetic
# When done, you are now in a Docker container with CUDA support
```

### Notes
* The `build.bash` and `run.bash` scripts may take a few minutes the first time they run.
* Code for the Docker development images is hosted in the [dockwater repository](https://github.com/Field-Robotics-Lab/dockwater). See the [repository wiki](https://github.com/Field-Robotics-Lab/dockwater/wiki) for more information about this project and supported use cases.

## Next: [Get the Source Code](/dave.doc/contents/installation/Clone-Dave-Repositories)