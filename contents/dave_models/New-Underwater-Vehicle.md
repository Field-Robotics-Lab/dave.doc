---
last_modified_date: 04/11/2022
layout: default
title: New Underwater Vehicle
parent: Vehicle Models
grand_parent: Dave Models
nav_order: 1
---


# Creating and Integrating a New U/W Vehicle

Given a visual and dynamic model of an underwater vehicle, this page discuses how to instantiate the model in Gazebo.

<p align="center">
<img src="https://user-images.githubusercontent.com/8268930/161824116-4142af5d-b4ab-42e7-9f28-fc85569cf993.gif"/>
</p>


In this example we will integrate a new robot with the name `virgil` that already has visual/collision meshes as well as a texture image file.

Much of the content in this page has its foundation in the RexROV from `uuv_simulator`

The final description for the robot discussed on this page can be found in `dave/urdf/robots/virgil_description`

## Overview
1. Create Directories and Import Mesh Files
2. Build URDF
3. Gazebo plugins
4. Add thrusters
5. Add sensors
6. Add joystick control
7. Create launch files to test

## Create Directories and Import Mesh Files
First, locate the `dave/urdf/robots` directory. This directory is the home to a variety of robot descriptions. We'll create a package for our new robot by running `catkin_create_pkg virgil_description`. In the newly created `virgil_description` directory create three directories: `meshes`, `launch`, and `urdf`. You will not need the `src` directory, so you can delete it.

Add your meshes and textures to the `meshes` directory.

Ex. In `virgil_description/meshes` the contents are `virgil.dae`, `virgil_diffuse.png`, and `virgil_prop.dae`. These are the meshes for the main body of the UUV, the texture image, and the mesh for the propellers respectively.

Note: It may be necessary to edit the mesh file to link to the correct texture filename.

## Build URDF
In `virgil_description/urdf` create `virgil.xacro`. This file will serve as the base definition of our robot.

After defining the xml version, robot definition, and initial arguments, we can begin creating the links and joints that make up the robot. We first create a "dummy" base link to eliminate the root link inertia warning given by Gazebo.
```
<!-- "Dummy" base link to eliminate root link inertia warning -->
<link name="$(arg namespace)/base_link"/>
```
Next we create the link that defines the body of the UUV. Virgil is based on existing U/W vehicle, so the mass, height, length, and width were taken from given specifications. The inertia matrix is based on a rectangular prism of the same dimensions.
```
        <inertial>
        	<origin xyz="0 0 0" rpy="0 0 0"/>
        	<mass value="4762.7"/>
        	<inertia
        		ixx="7116.41" ixy="0.0" ixz="0.0"
        		iyy="6637.07" iyz="0.0"
        		izz="3134.17"/>
        </inertial>
```
Next we define the visual and collision properties of the UUV. Both utilize the body mesh we added to the `meshes` directory, `virgil.dae` in this case. For mesh alignment we want our robot to be consistent with ROS [coordinate frame conventions](https://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions) (hence the 90 degree rotation in the origin such that positive `x` is forward). For virgil, our resulting visual/collision properties are
```
        <visual>
            <origin xyz="0 0 -0.9" rpy="0 0 1.5708"/>
            <geometry>
                <mesh filename="file://$(find virgil_description)/meshes/virgil.dae" scale="1 1 1"/>
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 -0.9" rpy="0 0 1.5708"/>
            <geometry>
                <mesh filename="file://$(find virgil_description)/meshes/virgil.dae" scale="1 1 1"/>
            </geometry>
        </collision>
```
We must then create a joint between the "dummy" link and our base link
```
    <joint name="$(arg namespace)/virgil_base_joint" type="fixed">
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <parent link="$(arg namespace)/base_link"/>
        <child link="$(arg namespace)/virgil_link"/>
    </joint>
```

Note: Orientations may not be consistent! While we had to rotate the mesh 90 degrees according to coordinate frame conventions, this might not be required.

## Gazebo plugins
In the same file, we need to add our joint state publisher and run the `gazebo_ros_control` plugin.
```
    <!-- Default joint state publisher -->
    <gazebo>
        <plugin name="uuv_joint_state_publisher" filename="libuuv_joint_state_publisher.so">
            <robotNamespace>$(arg namespace)</robotNamespace>
            <updateRate>50</updateRate>
        </plugin>
    </gazebo>

    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
            <robotNamespace>$(arg namespace)</robotNamespace>
            <robotParam>$(arg namespace)</robotParam>
        </plugin>
    </gazebo>
```

## Add thrusters
We will define our thrusters in our current `.xacro` file, but they can be defined in a separate file. Before we define the `thruster_macro`, we set the propeller mesh file and include the files required for the thrusters.
```
    <xacro:property name="prop_mesh_file" value="file://$(find virgil_description)/meshes/virgil_prop.dae"/>
    <xacro:include filename="$(find uuv_descriptions)/urdf/common.urdf.xacro"/>
    <xacro:include filename="$(find uuv_gazebo_ros_plugins)/urdf/thruster_snippets.xacro"/>
```
Next we define the thruster macro which accepts an `origin` and `thruster_id` arguments. The `origin` refers to the respective locations and orientations of each of the thrusters/propellers and the `thruster_id` allows us to identify each thruster individually which will be required when we setup joystick control.
```
    <xacro:macro name="thruster_macro" params="thruster_id *origin">
        <xacro:thruster_module_first_order_basic_fcn_macro
            namespace="$(arg namespace)"
            thruster_id="${thruster_id}"
            x_axis="0"
            y_axis="0"
            z_axis="1"
            mesh_filename="${prop_mesh_file}"
            dyn_time_constant="0.05"
            rotor_constant="0.00031">
            <xacro:insert_block name="origin"/>
        </xacro:thruster_module_first_order_basic_fcn_macro>
    </xacro:macro>
```
An example for a single thruster using this macro:
```
     <xacro:thruster_macro thruster_id="0">
        <origin xyz="-1.51341 0.53491 ${1.18597-0.9}" rpy="1.5708 0.0 ${1.5708+0.610865}"/>
     </xacro:thruster_macro>
```

Note: We can superimpose propellers on the vehicle mesh

 [thruster_snippets.xacro](https://github.com/uuvsimulator/uuv_simulator/blob/master/uuv_gazebo_plugins/uuv_gazebo_ros_plugins/urdf/thruster_snippets.xacro) definitions which make use of the [ThrusterROSPlugin](https://github.com/uuvsimulator/uuv_simulator/blob/master/uuv_gazebo_plugins/uuv_gazebo_ros_plugins/src/ThrusterROSPlugin.cc)

## Add sensors
There are a number of sensors available in DAVE. In `virgil.xacro`, we include the required sensor files as well as `virgil_sensors.xacro`. `virgil_sensors.xacro` defines which sensors are used and where they are placed on the vehicle
```
    <!-- Includes -->
    <xacro:property name="namespace" value="$(arg namespace)"/>
    <xacro:property name="inertial_reference_frame" value="world"/>
    <xacro:include filename="$(find uuv_sensor_ros_plugins)/urdf/sensor_snippets.xacro"/>
    <xacro:include filename="$(find uuv_gazebo_ros_plugins)/urdf/snippets.xacro"/>
    <xacro:include filename="$(find virgil_description)/urdf/virgil_sensors.xacro"/>
```

## Add joystick control
`/dave/examples/dave_nodes/src/virgil_thrusterop.py` defines the mapping between the joystick, such as a PS4 controller, and the thrusters on the vehicle. This mapping should correspond to [this guide](/dave.doc/contents/manipulator_demos/Logitech-F310-Gamepad-Mapping). As an example, there are three vertical thrusters on virgil. They are operated via the number 1 axis on the joystick. The values from the joystick are multiplied by a gain which scales the magnitude of the thrust as well as the direction. For axis 1 (up and down)
```
        # Put thruster gains into dict
        self.gain_dict = dict()
        self.gain_dict[1] = [1000.0, 1000.0, 1000.0]

        # Joystick to thruster i.d. mapping
        # Keys are the joystick axes, publishers
        self.joy2thrust = dict()

        # up down
        self.joy2thrust[1] = [rospy.Publisher('/%s/thrusters/%d/input'%(namespace,4), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,5), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,6), FloatStamped, queue_size=1)]

```
where 4, 5, 6 are the thruster ids.

## Create launch files to test
We need to create three launch files to load virgil into a world and allow us to control it with our teleoperation node.
In `/virgil_description/launch` we'll create `upload_virgil.launch`. This will be called from our main launch file, `virgil.launch`. `upload_virgil.launch` contains the `xacro` command which parses `virgil.xacro` and passes arguments
```
	<arg name="namespace" default="virgil"/>
	<arg name="debug" default="false"/>
	<param name="/$(arg namespace)/virgil" command="$(find xacro)/xacro '$(find virgil_description)/urdf/virgil.xacro' debug:=$(arg debug) namespace:=$(arg namespace)"/>
```
In `dave_nodes/launch` we create the `virgil_thrusterop.launch` file which runs the `joy_node` as well as the `virgil_thrusterop.py` node we created
```
  <arg name="joy_id" default="0"/>
  <arg name="namespace" default="virgil"/>

  <node pkg="joy" type="joy_node" name="joystick">
    <param name="autorepeat_rate" value="10"/>
    <param name="dev" value="/dev/input/js$(arg joy_id)"/>
  </node>

  <node pkg="dave_nodes" type="virgil_thrusterop.py" name="virgil_thrusterop"
  output="screen">
    <param name="namespace" value="$(arg namespace)"/>
  </node>
```
The launch file for virgil is `dave/examples/dave_robot_launch/launch/virgil.launch`. We first start gazebo with an underwater world.
```
	<arg name="gui" default="true"/>
	<arg name="paused" default="false"/>
	<arg name="world_name" default="$(find dave_worlds)/worlds/dave_ocean_waves.world"/>
	<arg name="namespace" default="virgil"/>
	<arg name="velocity_control" default="false"/>
	<arg name="joy_id" default="0"/>
	<arg name="debug" default="false"/>
	<arg name="verbose" default="false"/>
	<arg name="x" default="6"/>
	<arg name="y" default="4"/>
	<arg name="z" default="-93"/>
	<arg name="roll" default="0"/>
	<arg name="pitch" default="0"/>
	<arg name="yaw" default="0"/>

	<!-- Use Gazebo's empty_world.launch with dave_ocean_waves.world -->
	<include file="$(find gazebo_ros)/launch/empty_world.launch">
		<arg name="world_name" value="$(arg world_name)"/>
		<arg name="paused" value="$(arg paused)"/>
		<arg name="use_sim_time" value="true"/>
		<arg name="gui" value="$(arg gui)"/>
		<arg name="headless" value="false"/>
		<arg name="debug" value="$(arg debug)"/>
		<arg name="verbose" value="$(arg verbose)"/>
	</include>
```
then call the upload launch file we created as well as the thrusterop launch file we created.
```
  	<include file="$(find virgil_description)/launch/upload_virgil.launch"/>

  	<include file="$(find dave_nodes)/launch/virgil_thrusterop.launch">
    	<arg name="joy_id" value="$(arg joy_id)"/>
    	<arg name="namespace" value="$(arg namespace)"/>
  	</include>
```
Finally, we spawn the robot in the world and start `robot_state_publisher` node
```
  	<node name="spawn_virgil" pkg="gazebo_ros" type="spawn_model"
  		respawn="false" output="screen"
  		args="-urdf -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg roll) -P $(arg pitch) -Y $(arg yaw) -model $(arg namespace) -param /$(arg namespace)/virgil"/>

  	<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen">
  		<remap from="robot_description" to="/$(arg namespace)/virgil"/>
  	</node>
```

## Test
To test our new robot, run
```
roslaunch dave_robot_launch virgil.launch
```
We should now see our UUV in the Gazebo GUI.

We can now test/check that everything is working as expected. To see the joystick topic, run
```
rostopic echo /joy
```
which should show the axes array changing with stick input. Correspondingly, when we give an input for moving up (left joystick forward) we should see the data field change when we run
```
rostopic echo /virgil/thrusters/0/thrust
```
We can follow this same procedure for each sensor and thruster.

Other vehicle [examples](/dave.doc/contents/dave_models/vehicle_examples)