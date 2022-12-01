---
last_modified_date: 06/11/2021
layout: default
title: Get Source Codes
nav_order: 4
parent: Installation
---

## Dave repositories

Project Dave consists of this repository along with other UUV components available online.  Although some of the UUV components are available as packages, we recommend that you download them from source and work with them in your own `uuv_ws` ROS workspace so that you can modify your work.

### Clone Dave

Clone this repository and other relevant repositories provided under Field-Robotics-Lab:
  ```bash
  mkdir -p ~/uuv_ws/src
  cd ~/uuv_ws/src
  git clone https://github.com/Field-Robotics-Lab/dave.git
  ```

### Use `vcs` to clone source dependencies

If not already installed - [install vcstool](http://wiki.ros.org/vcstool).

Use `vcs` to read the input file and clone required dependencies

```bash
cd ~/uuv_ws/src
vcs import --skip-existing --input dave/extras/repos/dave_sim.repos .
```

If you get error `Could not determine ref type of version: git@github.com: Permission denied (publickey).`, try following
[Git Hub Error Permission Denied Publickey](https://docs.github.com/en/authentication/troubleshooting-ssh/error-permission-denied-publickey)
and remove clone failed empty directory and try again
```
cd ~/uuv_ws/src
rm -rf dockwater ds_msgs ds_sim eca_a9 rexrov2 uuv_manipulators uuv_simulator
vcs import --skip-existing --input dave/extras/repos/dave_sim.repos .
```


### OPTIONAL: GPU Multibeam sonar
```diff
- DO NOT INCLUDE THIS if you are not using multibeam sonar.
- It require CUDA Library and NVIDIA driver along with the NVIDIA graphics card that supports CUDA feature.
```
For instructions and plugin details : [Multibeam Forward-Looking Sonar](/dave.doc/contents/dave_sensors/Multibeam-Forward-Looking-Sonar/#installation)
```bash
cd ~/uuv_ws/src
vcs import --skip-existing --input dave/extras/repos/multibeam_sim.repos .
```

## Step 3: [Build the Dave Simulation Environment](/dave.doc/contents/installation/Build-Dave-Environment)
