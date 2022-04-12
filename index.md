---
layout: default
title: Home
nav_order: 1
description: "Project DAVE"
permalink: /
last_modified_date: 04/11/2022
---

# Project `DAVE`

DAVE Aquatic Virtual Environment(DAVE) is a simulation environment to support the rapid testing and evaluation of underwater robotic solutions, specifically underwater vehicles (AUVs/UUVs) operating autonomously and completing missions that involve autonomous maniputlation.  The environment is built upon existing ROS and Gazebo infrastructure, particularly the [UUV Simulator](https://github.com/uuvsimulator/) and WHOI's [ds_sim](https://bitbucket.org/whoidsl/ds_sim/src/master/).

## Quick Start

* For a high-level overview of the project, see the release highlight videos:
    * [Release 4.3.1 Highlight Video, March 2022](https://www.youtube.com/watch?v=5HixvTjXzsg)

      [![Release 4.3.1 Highlight Video](https://user-images.githubusercontent.com/7955120/160326859-5e360b2b-c45f-4900-9750-6803cce6fa46.png)](https://www.youtube.com/watch?v=5HixvTjXzsg)

       {% youtube "https://www.youtube.com/watch?v=5HixvTjXzsg" %}

    * [Release 2.0.0 Highlight Video, September 2020](https://vimeo.com/462024036)
        * Narrated version of the [Electrical Mating](https://vimeo.com/463124725) demonstration.
    * [Release 1.0.0 Highlight Video, June 2020](https://vimeo.com/426414758).
* For an overview of what is included in the most recent release, see the [Changelog](https://github.com/Field-Robotics-Lab/dave/blob/master/Changelog.md)
* To try the simulation, follow the [Installation Tutorial](contents/installation/Installation).

## Project Objective

The objective of the project is to provide the following capabilities:

* Visual, physical and (hydro)dynamic models of generlized vehicle, manipulator and sensor elements.
* Simulation of sensing specific to underwater robotics including perception (e.g., sonar, underwater lidar and optical imaging) and navigation (e.g., DVL and USBL).
* Parameterized representations of the ocean environment including seafloor bathymetry and ocean currents.

These capabilities will enable support the development of autonomous systems capable of multi-phase underwater missions over large time and space scales.

## Contributing

We welcome contributions from the robotics community. We are particularly interested in contributions to extend and improve capabiliteis associated with the objectives described above. To contribute, please submit an issue or a [pull request](https://github.com/Field-Robotics-Lab/dave/pulls).

In addition to our officially supported, core features, Dave also includes a number of features that are supported exclusively by community members. Click here for the current list of [community supported features](Community-Supported-Features).

# Tutorials and Demonstrations

Step-by-step guides to illustrate working examples how to accomplish certain tasks and demonstrate features of the project.

## Project Setup

Dave depends upon ROS Noetic and Gazebo 11 - with community support for previous versions of ROS and Gazebo.  The project may be installed directly on your host or run using Docker.

* Check to make sure you meet the [System Requirements](/dave.doc/contents/installation/System-Requirements).
* Follow the [Installation Tutorials](/dave.doc/contents/installation/Installation) to set up your system.

## Dave Models

### Underwater Vehicle Models

* [New-Underwater-Vehicle](/dave.doc/contents/dave_models/New-Underwater-Vehicle): Given a visual and dynamic model of an underwater vehicle, how to instantiate the model in Gazebo.
* Vehicle Models: Visual and physical models of existing vehicle platforms.
    * [Dave ROV Models](/dave.doc/contents/dave_models/vehicle_examples): Custom ROV's
    * [Dave Glider Models](/dave.doc/contents/dave_models/Glider-Models): Underwater and wave glider model examples.
    * [UUV Simulator Models](/dave.doc/contents/dave_models/uuv_sim_vehicles): ROV and AUV models from UUV Simulator project.
* [Manipulator-Models](/dave.doc/contents/dave_models/Manipulator-Models)

### Bathymetry Models

* [Bathymetry Models](/dave.doc/contents/dave_models/Bathymetry-Models): Underwater heightmap included in the repository.
* [Building a simple underwater environment via heightmaps](/dave.doc/contents/dave_models/Building-a-simple-underwater-environment-via-heightmaps): How to build your own bathymetry.

### Object Models

* [Object Models](/dave.doc/contents/dave_models/Dave-Object-Models): A collection of useful objects for composing underwater search and manipulation scenarios.

* [Object Degradation Tools](/dave.doc/contents/dave_models/Object-Degradation-Tools): Tools for altering the geometric shape of a 3D mesh model and for adding customized SDF properties such as surface material to be interpreted by Gazebo plugins.

## Perception and Sensing

* [Multibeam Forward Looking Sonar](/dave.doc/contents/dave_sensors/Multibeam-Forward-Looking-Sonar)
* [Doppler Velocity Logger](/dave.doc/contents/dave_sensors/whn_dvl_examples)
    * [DVL Water Tracking and Current Profiling](/dave.doc/contents/dave_sensors/DVL-Water-Tracking)
    * [DVL Seabed Gradient Estimation](/dave.doc/contents/dave_sensors/DVL-Seabed-Gradient)
* [3D Underwater Lidar](/dave.doc/contents/dave_sensors/3D-Underwater-Lidar)
* [Ultra-Short Base-Line](/dave.doc/contents/dave_sensors/usbl_tutorial)
* [Underwater Camera](/dave.doc/contents/dave_sensors/Underwater-Camera-Roadmap)

## Ocean Environmental Models

* [Ocean Current Models](/dave.doc/contents/dave_env/Ocean-Current):  A plugin for constant/stratified ocean current with Gauss-Markov model definitions.
* [Bathymetry generation and auto spawning](/dave.doc/contents/dave_env/Bathymetry-Integration): A plugin that automatically spawns and removes bathymetry grids converted preliminarily from NOAA data.
* [Occlusion](/dave.doc/contents/dave_env/Occlusion): An example of bottom occlusion due to silt or object occlusion due to marine growth.

## Subsea Manipulator Models and Manipulation Feature Demonstrations

* [Gamepad Teleoperation Mapping](/dave.doc/contents/manipulator_demos/Logitech-F310-Gamepad-Mapping): Default mapping of gamepad interface to thruster allocation and manipulators.
* [Electrical Mating Plugin](/dave.doc/contents/manipulator_demos/Electrical-Plug-Mating-Plugin): Youssef's GSoC demo of a custom plugin to implement the subsea mating of an electrical connector.  The plugin implements constraints on alignment and forces necessary to complete the manipulation.
* [BOP panel manipulation mission](/dave.doc/contents/manipulator_demos/BOP-Panel-Manipulation-Mission): Implementing the blowout preventer panel example from the UUV Simulator.
* [Manipulation force feedback](/dave.doc/contents/manipulator_demos/Manipulator-Force-Feedback): First implementation of joint-based force feedback to enhance perception.
* [Swapping-out-the-Oberon7-arm-with-another-manipulator](/dave.doc/contents/manipulator_demos/Swapping-out-the-Oberon7-arm-with-another-manipulator): Tutorial demonstrating how to swap out the oberon7 arm for another custom made arm.
* [Retrieving a bar from the seafloor](/dave.doc/contents/manipulator_demos/Teleop-Bar-Retrieval): Coordinated telecoperation of vehicle and manipulator.
* [Bimanual Manipulation Example](/dave.doc/contents/manipulator_demos/Bimanual-Manipulation-Setup-and-Examples): Equipping the RexROV with dual Oberon7s.

## Navigation system design
* [Terrain Aided Navigation (TAN) Senarios](contents/Terrain-Aided-Navigation-(TAN)-Senarios)

## Integrated Scenarios
* [Integrated World](/dave.doc/contents/integrated_world)

# Notes and Sandboxes

## Descriptions
Descriptions of how things work, how certain aspects of the project are implemented (descriptions of the methods and techniques used by developer) or results of reverse engineering existing implementations.  May include recommendations for further work and improvements.

* UUV Simulator [Image Sonar](/dave.doc/contents/image_sonar_description): Description of foward looking sonar implementation.
* UUV Simulator [Doppler Velocity Loggers](/dave.doc/contents/dave_sensors/dvl_description): Description of UUV Simulator and WHOI DSL environment DVL plugin implementations.

## Historical notes

* [Working notes for UUV Simulator](/dave.doc/contents/Notes).
* [UUV Simulator Reference](/dave.doc/contents/uuv_simulator_reference)







