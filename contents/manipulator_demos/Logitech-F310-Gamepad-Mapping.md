---
last_modified_date: 04/11/2022
layout: default
title: Gamepad Mapping
parent: Manipulator Demos
nav_order: 1
has_children: false
---

This page describes the default mapping for the [Logitech F310 Gamepad](https://www.logitech.com/assets/35017/gamepad-f310-gsw.pdf) using the Rexrov UUV and Oberon 7 arm.  This same mapping is used for the as the default mapping for teleoperation of the other UUV models within the DAVE environment - see [vehicle_examples](/dave.doc/contents/dave_models/vehicle_examples).

## Maneuvering the UUV
In the default mode (when the RB button is not depressed) gamepad controls UUV movement as follows.

### Right Analog Mini Stick
* Up: Thrust forward.
* Down: Thrust reverse.
* Left: Slide left.
* Right: Slide right.

### Left Analog Mini Stick
* Up: Lift.
* Down: Sink.
* Left: Rotate counter-clockwise.
* Right: Rotate clockwise.

## Manipulating the Oberon 7 Robot Arm
When the RB button is depressed, the gamepad controls the Oberon 7 Arm (part names are based on the [ViperX-250 specifications](http://support.interbotix.com/html/specifications/vx250.html)).

### Right Analog Mini Stick
* Left: Swivel waist left.
* Right: Swivel waist right.
* Up: Swivel waist up.
* Down: Swivel waist down.

### Left Analog Mini Stick
* Left: Rotate wrist clockwise.
* Right: Rotate wrist counter-clockwise.
* Up: Swivel elbow up.
* Down: Swivel elbow down.

### DPAD
* Up: Swivel wrist up.
* Down: Swivel wrist down.

### Buttons
* B: Open grabber.
* X: Close grabber.

**Note:** There is a noticeable delay in response time for arm controls, presumably by design.
