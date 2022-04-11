---
layout: default
title: Dave ROV Models
parent: Vehicle Models
grand_parent: Dave Models
nav_order: 2
---

# Examples of Vehicles

Collection of how to spawn existing models.

To spawn a collection of the DAVE models in the same Gazebo world, run

```
roslaunch dave_robot_launch uuv_collection.launch paused:=true
```

which should generate a view like this (sans text labels),

![dave_collective_annote](https://user-images.githubusercontent.com/8921143/159903174-8c82c247-8aa1-46b5-b411-dca24cc4c3ad.png)

Note that the functionality (closed-loop control, teleoperation, etc.) is not setup in this collective example, but is illustrated for each vessel in the examples below.

## Dave ROVs

### Smilodon

```
roslaunch smilodon_gazebo smilodon.launch
roslaunch uuv_dave joy_thrusterop.launch namespace:=smilodon
```

![smilodon](../images/smilodon1.png)

![smilodon_rear](../images/smilodon_rear.png)


### Caracara

```
roslaunch caracara_gazebo caracara.launch
roslaunch uuv_dave joy_thrusterop.launch namespace:=caracara
```

![caracara](../images/caracara.png)

![caracara2](../images/caracara2.png)



### Caldus

```
roslaunch caldus_gazebo caldus.launch
roslaunch uuv_dave joy_thrusterop.launch namespace:=caldus
```

![caldus](../images/caldus.png)


### Virgil
```
roslaunch dave_robot_launch virgil.launch
```
which includes gamepad telop following the convention [Gamepad Teleoperation Mapping](/dave.doc/contents/Logitech-F310-Gamepad-Mapping).


![virgil](https://user-images.githubusercontent.com/8268930/159816122-97fae421-ac6a-47f6-b344-06375ccff1d1.png)


![Screenshot from 2022-03-29 20-52-49bn](https://user-images.githubusercontent.com/8921143/160748396-98543533-a530-4384-b3b4-b67e80f9affd.png)



