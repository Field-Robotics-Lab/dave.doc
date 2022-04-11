---
layout: default
title: carlos_create_models
nav_exclude: true
---


Below is email instructions for creating new Gazebo models

---------------------

Hi,

We'd like to propose you an initial task to get familiar with Gazebo and VRX before moving forward with more complex tasks. The goal is to integrate some 3D objects into VRX. Our 3D artist, created the following meshes:

  * A WAM-V battery to be placed in each WAM-V pontoon
  * A set of WAM-V computer boxes to be placed on the upper plate
  * A groundstation composed by a few tents, tables and an antenna
  * A monocular camera with a post
  * A 3D lidar with a post

These are roughly all the instructions to make this task:

1. Download the mesh[es] . See the link in the associated Bitbucket issue.
2. You can open the mesh with meshlab for a quick inspection.
3. Typically the mesh uses a texture (.png file). You'll have to edit the mesh file (.dae usually) and find the place where we load the texture.

<init_from>file://H:\Dropbox\SubT\Robots\Sensors\Lidar_01.png</init_from>

And replace it with:

<init_from>Lidar_01.png</init_from>

Feel free to rename the .dae and .png files to lowercase and simple names if needed.

4. Then, I recommend to create a standalone model for testing. Go to vrrx_gazebo/models and create a new directory for your model. Create a model.config and model.sdf inside. You can copy it from another model as a template. Create a mesh directory and place your .dae and .png inside.

Once you have this ready, you should be able to start VRX and insert the new model via the GUI. Look for the insert tab on the left column of the GUI. Be aware that unless your model is static, if will free fall once inserted. You can make it static or pause the simulation.

Once you can visualize your model, this is the time to check that it has the right orientation, offsets, etc. Otherwise, you can edit the sdf and insert it again for testing (you don't need to restart Gazebo).

5. The next step is to work on the collision elements. We prefer not to use a mesh as a collision element because is less efficient and usually not needed. Instead, we use one or multiple simple shapes (cylinders, spheres and boxes) to approximate the object. Replace the collision element of your model with one or multiple simple shapes. We don't need to be absolutely right here, it's just an approximation. You can visualize the collision of your object by selecting the model in Gazebo, right click, view->collisions .

6. Next is to work on the inertial properties. Find the <inertial> block of another model as a template and copy it into your sdf file. You'll need to estimate the mass of your object and its moment of inertia matrix. For the mass, feel free to find the specs of a similar object. For the moments of inertia, I typically follow this reference:

https://en.wikipedia.org/wiki/List_of_moments_of_inertia

and I approximate my model to a sphere, cylinder or box. Once you have your values, you can visualize it in Gazebo by selecting your model, right click, view->inertia . As a rule of thumb, your visualized inertia should surround your object, similar to the collision visualization.

7. And the final step is to integrate it with VRX. For batteries, or boxes, you'll need to edit wamv_gazebo.urdf.xacro, for sensors, you'll need to edit the macros under the wamv_gazebo/urdf/sensors directory. And for the ground station, you'll need to tweak the sandisland.xacro file (notice that there's a flat section on the beach designed to place the groundstation.

We have created individual issues in the VRX issue tracker to track the progress of these tasks. Feel free to assign yourself to any of these, ask questions there or update the tickets with your progress.

https://bitbucket.org/osrf/vrx/issues?status=new&status=open

Let's start!