---
layout: default
title: vcstool
nav_exclude: true
---

[vcstool](https://github.com/dirk-thomas/vcstool) makes managing workspaces for a project with multiple repositories much easier (creating, updating, etc).
It's easy to maintain repos files for past versions of the project. This makes it easier to create a workspace for that specific version of the project.

## DAVE workspace creation with vcstool

Create the DAVE workspace from the included [repos file](https://github.com/Field-Robotics-Lab/dave/blob/master/extras/repos/dave_sim.repos).

  ```bash
  sudo apt install python-vcstool
  mkdir -p ~/uuv_sim/src
  mv dave_sim.repos ~/uuv_sim/src/.
  cd ~/uuv_sim/
  vcs import src < src/dave_sim.repos
  ```