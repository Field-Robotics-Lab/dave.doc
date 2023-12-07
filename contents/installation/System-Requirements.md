---
last_modified_date: 29/11/2023
layout: default
title: System Requirements
nav_order: 1
parent: Installation
---

# System Requirements for Running dave
## Hardware

* A modern multi-core CPU, e.g. Intel Core i5
* 8 Gb of RAM
* A discrete Graphics Card, e.g. Nvidia GTX 650
    * The environment can be run without a GPU, but the Gazebo simulation will run much faster (should run in real-time) with access to a GPU. Without a GPU the simulation is likely to run slower than real-time.

## Software
 - Recommended
   * Ubuntu Desktop 20.04 Focal (64-bit)
     * If you're system is other than 20.04 (e.g. 22.04 Jammy Jellyfish), you can use Docker to run the Dave environment which will run 20.04 in docker environment. Check the Docker Requirements below and proceed. Installing dicrectly on host requires much more work.

 - Legacy mode
   * Ubuntu Desktop 18.04 Bionic (64-bit)
   * Gazebo 9.11.0+ (<9.11.0 is not sufficient)
   * ROS Melodic

### Nvidia Driver
Our tutorials assume you have an Nvidia graphics card configured to use the proprietary driver.
* There are many online guides for configuring the Nvidia driver correctly on Ubuntu.
* Here's [one example with graphical and command-line options](https://linuxhint.com/update-nvidia-drivers-ubuntu-22-04-lts/).
* You may test your driver installation by running `nvidia-smi` in a terminal. If the command is not found, the driver is not installed correctly. Also test 3D Rendering with `glxgears` which can be used after installing the `mesa-utils` package with `sudo apt install mesa-utils`.
* Last tested version for multibeam sonar is, `Ubuntu 22.04`, `NVIDIA nvidia-driver-535`, `CUDA 12.2`. The default install of Ubuntu 22.04 which can be chcked with `nvidia-smi` command on the terminal. When you change NVIDIA driver, please restart.

### Docker
Running Docker is optional but it's more trustworthy if you are not so familiar with installation process. If you choose to use the container-based installation instructions, the following are required:
* Docker v19.03 or higher ([installation instructions](https://docs.docker.com/engine/install/ubuntu/))
    * Make sure to also complete the [Linux post-install instructions](https://docs.docker.com/engine/install/linux-postinstall/) linked at the bottom of the page.
    * To use NVIDIA GPUs with Docker, you will need to install the following:
        * nvidia-container-toolkit ([installation instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html))
          * We are using Docker as a container runtime engine
          * Make sure to restart docker after installation of nvidia-container-toolkit `sudo systemctl restart docker`
          ```
        * Check [Dockwater prerequisites NVIDIA driver versions](https://github.com/osrf/rocker#nvidia-settings) to see which NVIDIA driver versions are supported for your system Ubuntu versions.
        * Tested with Docker Installation method at Ubuntu 22.04 6.2.0-37-generic with nvidia-535 driver worked fine

## Peripherals
We also recommend a gamepad for testing the UUV and its arm and sensor devices. In the examples we use a Logitech F310 ([Walmart](https://www.walmart.com/ip/Logitech-F310-GamePad/16419686)).