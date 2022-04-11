---
layout: default
title: Underwater Camera
parent: Dave Sensors
nav_order: 5
has_children: false
---

# Underwater Camera Roadmap (Work in Progress)

## Turbidity and Light Attenuation
Turbidity and light attenuation are important underwater effects that give water its volume. In order to get some effects (volumetric shadows, god rays, etc…) working, water must be rendered as a volume / participating medium.

[[https://user-images.githubusercontent.com/14305903/159775251-35f5e1f4-e30f-41ec-89c7-0ef3636f382f.png]]
[[https://user-images.githubusercontent.com/14305903/159775255-482342b1-cc19-4cae-9e1d-7d66487fb723.png]]

It becomes much more difficult to see at depths so a light source near the camera may need to be implemented. Attenuation and in-scattering can be adjusted to an amount that doesn’t degrade visibility at depths.

### Approach 1: In-scattering with atmosphere (treat it like fog)
Create a fragment shader that calculates the in-scattering between the camera and the current fragment. Calculate the reflectance of the fragment surface and combine the two calculations to get the output color.

## Backscatter
Reflections from particles / dust floating around in water produces backscatter. To best demonstrate the effect, the particles should be animated and should also have their reflections attenuated.

<img src="https://user-images.githubusercontent.com/14305903/159775337-76999059-a53d-475a-b94e-24abacd05492.png" width=45% height=45%>
<img src="https://user-images.githubusercontent.com/14305903/159775339-7add24dc-a7bb-417e-83eb-5592897b1192.png" width=45% height=45%>

### Approach 1: Create seamless animated spherical textures
To simulate movement, one will create multiple different animated textures that contain backscatter with a transparent background. After rendering the scene normally, a subpass will be created that will render multiple spheres with the backscatter texture applied. The spheres should be rendered around the camera with increasing radii. As the camera moves around, the sphere that has just been passed through by the camera will be repositioned and rescaled. With proper fading values, the transition between spheres will be seamless.

### Approach 2: Generate a group of primitives to be rotationally rendered into the scene
Before the renderpass, generate a group of primitives that will represent the particles in the backscatter. Add a subpass after the original renderpass that will render the generated primitives with a given shader.

## Water Refraction (God Rays)
#### Requires: Turbidity and Light Attenuation for a participating media

## Underwater Volumetric Shadows
#### Requires: Turbidity and Light Attenuation for a participating media
Since the turbidity effect renders water as a volume, lights should be able to cast shadows on the volume.

<img src="https://user-images.githubusercontent.com/14305903/159775384-17b24a85-2cd3-4723-943e-bdffda239fa0.png" width=45% height=45%>
<img src="https://user-images.githubusercontent.com/14305903/159775388-f90d9e1e-f379-4ffc-bbff-fb3e37846ac9.png" width=45% height=45%>

### Approach 1: Ray-marching with shadow volumes

## Underwater Caustics

### Approach 1: Apply texture to ground surface

### Approach 2: Physically accurate caustics (hard)

## Underwater Color Distortion
A simple solution is to add a hue compositor, but in order to get a physically accurate representation of color distortion, one would need to calculate the water light absorption given a wavelength. This approach is more viable through ray tracing and monte carlo sampling but will be much harder to implement through a raster engine.

### Approach 1: Hue compositor

### Approach 2: Ray tracing with Monte Carlo sampling

## Underwater Camera Distortion
Camera distortion is already implemented in both Gazebo Classic and Ignition. Parameters can set the lens to have a barrel or pincushion distortion.

### Approach 1: Already implemented
To create either a pincushion or barrel distortion, adjust the SDF distortion values for the camera.

## Underwater Lights

<img src="https://user-images.githubusercontent.com/14305903/159775423-47e796c0-c99e-4280-8737-33baaaba2b4d.png" width=45% height=45%>
<img src="https://user-images.githubusercontent.com/14305903/159775428-f6608e92-61de-4aa1-806c-a3be6746b935.png" width=45% height=45%>