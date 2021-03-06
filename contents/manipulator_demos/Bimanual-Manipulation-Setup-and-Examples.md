---
last_modified_date: 11/04/2022
layout: default
title: Bimanual Manipulation
parent: Manipulator Demos
nav_order: 7
has_children: false
---

## Summary
This example demonstrates three key developments
- Configuring an arbitrary arm for MoveIt and Gazebo
- Launch two robots in an environment as well as move_group modification
- Add and configure two arms for REXROV then demonstrate key movements

![image](https://user-images.githubusercontent.com/8268930/158883555-5c13c2ad-f80b-4595-b4f2-04ede3b23618.gif)
![image](https://user-images.githubusercontent.com/8268930/158883680-1fadc261-026e-4ef2-aa3a-28af7b43a8f3.gif)

### MoveIt
MoveIt is an open-source project that provides the tools to develop complex manipulation scenarios. It integrates a number of motion planning libraries. MoveIt runs on top of ROS and builds on ROS functionality.

Configuring an arm for MoveIt is made simpler by using the MoveIt Setup Assistant. For our example, we followed the [ROS Industrial/Universal Robot](https://github.com/ros-industrial/universal_robot) format for our robot `*.urdf` files. We found that by adopting this structure over the UUV Simulator structure allowed for greater customization and easier non-MoveIt configurations and sensor additions.

The process for configuring an arm is as follows, with files located in the `uuv_manipulators/oberon7` sub-directories:
1. Start the MoveIt Setup Assistant. This will generate the MoveIt configuration files for planners, collision checking, pick-and-place infrastructure, joystick teleoperation, and predetermined poses. The robot will not be controllable in Gazebo as no ROS controllers will exist, only "fake" MoveIt controllers.
2. Create the ros controllers. These must be done "by hand" and are specific to the robot. For the Gazebo side, we created three files: `joint_state_controller.yaml`, `arm_controller.yaml`, and `hand_controller.yaml`.
3. Create a launch file to start all of these controllers. In our case, this launch file is `oberon7.launch` and includes a node to load ROS controllers.

### Dual Oberon7
After configuring one arm, we can make small modifications to allow for _N_ arms in a simulation. In this example, we add group namespaces to several files to allow us to add two arms to one simulation while not having to generate separate controller files.

To start the Gazebo simulation in an empty world, run

```
roslaunch dave_demo_launch dave_two_arm_demo.launch
```

Once the controllers have loaded it is safe to start the MoveIt move group and RViz

```
roslaunch oberon7_moveit_config oberon7_multi_planning_execution.launch
```

Because we are treating each arm as a separate robot, this will bring up two RViz windows. Each robot can accept and execute a motion plan simultaneously, but are not aware of each other in terms of collision avoidance.


![image](https://user-images.githubusercontent.com/8268930/154535219-23ec0f52-8197-43b8-9cb0-76baf48d3ce0.png)
![image](https://user-images.githubusercontent.com/8268930/154535236-ececfd58-18db-4c67-8210-6fdefacbe407.png)

In each of the two launch files for this example we added a `group ns`. This allows us to differentiate between the two arms, add a starting pose, define a parent to connect the arms to, and load ros controllers under each namespace in `oberon7_multi.launch`.

From `dave_two_arm_demo.launch`

```
<group ns="robot1">
    <include file="$(find oberon7_gazebo)/launch/oberon7_multi.launch">
        <arg name="xyzpos" value="0.0 0.0 2.0"/>
        <arg name="parent" value="$(arg parent)"/>
        <arg name="robot_name" value="robot1"/>
    ...
```

From `oberon7_multi_planning_execution.launch`

```
<group ns="robot1">
    <arg name="moveit_controller_manager" default="oberon7" />
    ...
    <include file="$(find oberon7_moveit_config)/launch/move_group.launch">
    ...
```

#### Generating Poses
There are multiple ways of generating poses for each arm. For this demo we consider three of them, all of which require RViz.

- Random goal pose
- Pre-saved poses
- Drag group end-effector

To choose a random goal pose, for an arm in RViz choose `<random valid>` in the "Goal State" drop down menu. Similarly, saved poses are avaiable under the "Start State" and "Goal State" drop down menus. To switch to the gripper, choose `hand` from the "Planning Group" drop down. The end effector can be more specifically moved by clicking and holding the highlighted coordinate frame and simply dragging it to the position you want the arm to move to.

![image](https://user-images.githubusercontent.com/8268930/155809278-c3cd32ef-4e4a-45c9-8ba9-b6ec4b95fda0.gif)

### Dual Oberon7 on REXROV
Adding two controllable arms to another robot requires a different solution to the `group ns` solution outlined above. In this case, the oberon7 xacro files were modified to accept a namespace value. This namespace is reflected in the controllers required to run each arm. There are pros and cons to this setup as well.

Starting the Gazebo simulation in the `dave_ocean_waves.world` world.

```
roslaunch dave_demo_launch dave_oberon7_moveit.launch
```

Once the simulation has started and controllers have been loaded, the MoveIt interface can be started along with RViz. Starting this prematurely will result in the example not working.

```
roslaunch rexrov_oberon7_moveit rexrov_dual_arm_moveit_planning_execution.launch moveit_controller_manager:=rexrov
```

![image](https://user-images.githubusercontent.com/8268930/154536497-3e6a86dc-c463-436c-ba23-f44beef83aec.png)
![image](https://user-images.githubusercontent.com/8268930/154536507-549f330e-6eb0-4a27-9937-9859da79f5fe.png)

#### How to move the arms
The arms on the REXROV operate similarly to the independent Oberon7 arms in the previous example, despite significant differences in controllers and configuration. The easiest way to move them is via RViz or `joystick_control.launch`. However, for more advanced control, a ROS node can be created.

### Choosing a Controller
The bimanual manipulator example has been built with two controllers: a position controller, and an effort controller. The position controller may be used when the position of the arm and gripper is of interest. In cases such as gripping or force-feedback applications, the effort controllers are likely to be most appropriate. While we can mix position and effort controllers, for the demo below we configure the example with the effort controllers.

To ensure the example uses the effort controllers there are two steps:
1. in `dave_demo_launch/launch/dave_oberon7_moveit.launch`, ensure that the `controller_type` argument is set to `effort`
2. in `oberon7_moveit_config/config/dual_controllers.yaml`, ensure that the position controllers, such as `arm_position_l`, are commented out.

Once complete, move on to the next section to launch the demo.

### Simple RexROV Demo
This demo uses a ROS node to call the MoveIt move_groups to actuate each of the grippers and each of the arms. The code also demonstrates different methods for acquiring the pose of an object in coordinates that can be passed directly to the gripper.

To run the demo, we follow the same procedure as above then run the node. Start by launch Gazebo and controllers

```
roslaunch dave_demo_launch dave_oberon7_moveit.launch
```

Once controllers are loaded and Gazebo is running launch MoveIt and RViz with

```
roslaunch rexrov_oberon7_moveit rexrov_dual_arm_moveit_planning_execution.launch moveit_controller_manager:=rexrov
```

Once RViz is loaded, you can run the node

```
rosrun dave_nodes bimanual_simple_demo.py
```

The resulting example shows a sequence of actions which includes both arms and grippers. The left arm moves into a grasping position and the respective gripper opens. The end effector is moved to the static bar. The gripper closes around the bar. Next, the right arm moves into a grasping position. For this action, we added a helper function that reads the state of the target and converts it into a pose that can be sent to the MoveIt commander. The result is a movement that is robust to small variations caused by the first gripping maneuver. The responsible function is `get_target_pose()` in `bimanual_simple_demo.py`.

![image](https://user-images.githubusercontent.com/8268930/158883555-5c13c2ad-f80b-4595-b4f2-04ede3b23618.gif)

#### Stabilization
Using the previously mentioned effort controller for grasping, we are able to use one arm with the intent to stabilize while the other is free to continue with other tasks. Inspiration for bimanual manipulation is taken from [this "Oceaneering" video](https://www.youtube.com/watch?v=h-ke3Eux_rs&t=47s). While we could instantiate a link representative of a "grasp", we chose to modify our effort controller parameters to support grasping we considered "reasonable". In this case, controller goal and trajectory constraints, and maximum effort and friction values specific to the gripper were tested until a reliable grasping capability was achieved. It is worth noting that the object being grasped has a significant impact on graspability. In the case of the pot and static rod in this example, friction values were added to the model files/definitions.

![image](https://user-images.githubusercontent.com/8268930/158883680-1fadc261-026e-4ef2-aa3a-28af7b43a8f3.gif)

Notes:
- It is possible to control multiple arms simultaneously by adding move_groups to a new "umbrella" move_group. When this move_group is called, collision checking is disabled between arms. [Multi-Group MoveIt Tutorial PR](https://github.com/ros-planning/moveit_tutorials/pull/621)

