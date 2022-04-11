---
last_modified_date: 04/11/2022
layout: default
title: Object Degradation Tool
parent: Object Models
grand_parent: Dave Models
nav_order: 2
---

## 3D mesh model distortion

Geometric distortions of 3D models may be desired for many use cases, such as
to test the generalization of machine learning algorithms for manipulation.

One way to programmatically distort models is to use Blender Python scripting.
Blender is a free open source 3D modeling software used by professional artists
for games and films.
It offers a Python API for almost anything available in its GUI.

This gives us the capability to quickly distort object models to incremental
extents, to be used for comparisons and evaluations of algorithms that use the
object models in applied scenarios.

See the README
[here](https://github.com/Field-Robotics-Lab/dave/blob/273a2465f1a8566b015d58dc361ad225167f39e8/urdf/scripts/mesh_distortion/README.md)
for documentation and tutorial.

Example videos of Coke cans degraded to increased extents:
- [Coke](https://user-images.githubusercontent.com/7608908/140995575-75c2d9de-6b2b-4904-ae80-b59311442e27.mp4)
- [Coke Can](https://user-images.githubusercontent.com/7608908/140995592-db9bd2cc-29ee-4004-b412-a44cc50be284.mp4)

## Custom SDF model properties

Custom model properties, achieved by
[custom elements and attributes](http://sdformat.org/tutorials?tut=custom_elements_attributes_proposal),
are useful for properties not captured by typical sensors, which can be picked
up in a custom plugin.
For example, physical properties like surface material, roughness, slippage,
and other properties affect the signal strength reflected back to multi-beam
sonars.
Such properties are neither captured by the standard SDF specification nor
typical sensors, and are application-dependent.

Users may choose to write custom sensors or plugins to make use of such
custom properties.

The advantage of specifying these properties in the SDF is to keep them separate
from the plugin code, close to the object, and be associated with specific
objects.

See the README
[here](https://github.com/Field-Robotics-Lab/dave/blob/273a2465f1a8566b015d58dc361ad225167f39e8/urdf/scripts/sdf_custom_properties/README.md)
for documentation and tutorial.

## Reference: Fouling rating scale

We do not have active plans to adhere to this scale.
It is reproduced here for reference for possible adaptations.

Reproduced from Table 081-1-1 in Waterborne Underwater Hull Cleaning of Navy
Ships, Chapter 081, Naval Ships' Technical Manual, S9086-CQ-STM-010,
revision 5, 1 Oct 2006.

Fouling ratings (FR) in order of increasing severity

Type | FR | Description
:---: | :---: | ---
Soft | 0 | A clean, foul-free surface; red and/or black AF paint or a bare metal surface.
Soft | 10 | Light shades of red and green (incipient slime). Bare metal and painted surfaces are visible beneath the fouling.
Soft | 20 | Slime as dark green patches with yellow or brown colored areas (advanced slime). Bare metal and painted surfaces may by obscured by the fouling.
Soft | 30 | Grass as filaments up to 3 inches (76 mm) in length, projections up to 1/4 inch (6.4 mm) in height; or a flat network of filaments, green, yellow, or brown in color; or soft non calcareous fouling such as sea cucumbers, sea grapes, or sea squirts projecting up to 1/4 inch (6.4 mm) in height. The fouling can not be easily wiped off by hand.
Hard | 40 | Calcareous fouling in the form of tubeworms less than 1⁄4 inch in diameter or height.
Hard | 50 | Calcareous fouling in the form of barnacles less than 1⁄4 inch in diameter or height.
Hard | 60 | Combination of tubeworms and barnacles, less than 1⁄4 inch (6.4 mm) in diameter or height.
Hard | 70 | Combination of tubeworms and barnacles, greater than 1⁄4 inch in diameter or height.
Hard | 80 | Tubeworms closely packed together and growing upright away from surface. Barnacles growing one on top of another, 1⁄4 inch or less in height. Calcareous shells appear clean or white in color.
Hard | 90 | Dense growth of tubeworms with barnacles, 1⁄4 inch or greater in height; Calcareous shells brown in color (oysters and mussels); or with slime or grass overlay.
Composite | 100 | All forms of fouling present, Soft and Hard, particularly soft sedentary animals without calcareous covering (tunicates) growing over various forms of hard growth.

## Future work

We are upstreaming these tools to OSRF repositories as usage examples.
Mesh distortion tutorial is being upstreamed [here](https://github.com/ignitionrobotics/ign-gazebo/pull/1401).