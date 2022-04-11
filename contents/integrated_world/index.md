---
last_modified_date: 04/11/2022
layout: default
title: Integrated Demo
nav_order: 7
has_children: false
permalink: contents/integrated_world
---

# Introduction

In the integrated scenario we render different task scenarios in a single world. Our goal is to show that all these individual models can seamlessly function together. This world is filled with multiple vehicles, representative environments and object models to illustrate simulation capability and facilitate multiple missions.

# Scene

The world's seabed is a scaled Santorini bathymetry including high regions and relatively flat floor in a small area. The gif shows a Rexrov UUV spawing above central mound.

![new_world](https://user-images.githubusercontent.com/24695820/159145007-1544f8a0-3d79-4b14-bb78-37ac3af41d38.gif)

Refer the GUI screenshot below.
Starting from bottom, the world features two mud pits: one with variable depth and the other of constant depth that follows the terrain.

On the top left, there's a combined the [Electrical Mating Plugin](/dave.doc/contents/manipulator_demos/Electrical-Plug-Mating-Plugin) and [Bimanual Manipulation](/dave.doc/contents/manipulator_demos/Bimanual-Manipulation-Setup-and-Examples) demos with a dual arm Rexrov.

Additionally, there are new [Object Models](/dave.doc/contents/dave_models/Dave-Object-Models) for the robot to interact with. Some objects are simulated to be physically degraded (which can be manipulated as well).

Lastly, featuring multiple UUVs teleoperated using separate joysticks and arms commanded via MoveIt. These UUVs can be teleported to the various demo stations in the world. Check Usage for details.

![DAVE Integrated world schematic](https://user-images.githubusercontent.com/24695820/160191650-973af7b8-0cab-4335-999b-7d0a222bbdb5.jpg)


# Usage

To launch the demo world

```bash
roslaunch dave_demo_launch dave_integrated_demo.launch
```

## UUVs

The world spawns two UUVs, a single arm `rexrov0` UUV which can be teloperated using a joystick connected to `js0` and a dual arm `rexrov` UUV which can be teleoperted using a joystick connected to `js1`. The arms on this UUV can be commanded via the moveit interface. See Electrical Station below for more information.

## Stations

The world has multiple stations where UUVs can interact with objects and the environment. A helpful script has been developed to quickly "teleport" the desired UUV to the various stations. Run the teleporter script:

```bash
roslaunch dave_nodes integrated_world_teleporter.launch
```

Then to teleport `rexrov0` to the mudpit, type "rexrov0 mud" (without quotes) and hit enter. The UUV should automatically teleport. Gazebo does not have a service to update the camera view through a service call but this can be done in Ignition which is left for future work.
Locations include

* artillery
* mud
* home

### Home

The `rexrov0` UUV spwans here

![image](https://user-images.githubusercontent.com/13482049/160080895-b5e602e9-51fd-429b-b0fa-2f08d133a0d7.png)

The `flight_data_recorder` object can be picked up. The other objects are static to reduce computational load.

## Mud

![image](https://user-images.githubusercontent.com/13482049/160095698-3ab08b5e-10c9-4a18-bbb7-7d9e1d2c40ee.png)

The `rexrov0` can pick up the anchor that is loaded with the mud plugin and drag it along the mud. The vase beside the anchor can also be picked up. The degraded vase and the narrower vase are static to reduce computational load.

## Artillery

![image](https://user-images.githubusercontent.com/13482049/160096348-574eab6b-62f5-44df-a602-f173dcdd0449.png)

## Electrical

![image](https://user-images.githubusercontent.com/13482049/160096404-890dcb1b-0906-4a98-a357-932f4340b9f0.png)

The dual arm `rexrov` spawns here. The arms can be controlled with via `moveit`. First start the controller.

```bash
roslaunch rexrov_oberon7_moveit rexrov_dual_arm_moveit_planning_execution.launch moveit_controller_manager:=rexrov
```

To instruct the UUV to pick up the plug and bring it close to the recepticle

```bash
rosrun dave_nodes bimanual_integrated_world.py
```

The user can teleoperate the UUV via `js1` to finish the insertion.

# End Note

This scenario is an example of many possible complex simulated underwater environments that could be created with DAVE.
Some next steps in this particular example are:

* Adding the [Multibeam Forward Looking Sonar](/dave.doc/contents/dave_sensors/Multibeam-Forward-Looking-Sonar) and process the range data of the environment and objects.

* Changing GUI camera view angles after the vehicle is teleported.
