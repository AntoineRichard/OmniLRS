# Rendering Configuration

In the following we will go over the different configurations available to you to tweak the rendering in the simulation.

## RTXRealTime & RTXInteractive

Let's start by what they are. RTXRealTime is has it's name imply, a high FPS renderer that trades some of the rendering quality to push more frames per second. The RTXInteractive renderer makes less compromises on the rendering quality and hence the overall quality of rendered frame is higher at the cost of slower render time.

> [!Note]
> We only expose a few parameters of the fairly large amount available to tune. They should be expended when time allows.

Arguments:
- `samples_per_pixel_per_frame`: (`int`), RTXInteractive only.
- `max_bounces`: (`int`), RTXInteractive only.
- `max_specular_transmission_bounces`: (`int`), RTXInteractive only.
- `max_volume_bounces`: (`int`), RTXInteractive only.
- `subdiv_refinement_level`: (`int`), RTXInteractive only.
- `renderer`: (`str`), The type of renderer. `PathTracing` for RTXInteractive, `RayTracedLighting` for RTXRealTime.
- `headless`: (`bool`), Whether the simulation should be ran headless or not.

Example:
```yaml
renderer:
  renderer: "PathTracing"
  headless: False
  samples_per_pixel_per_frame: 32
  max_bounces: 6
  max_specular_transmission_bounces: 6
  max_volume_bounces: 4
  subdiv_refinement_level: 0
```
```yaml
renderer:
  renderer: "RayTracedLighting"
  headless: False
  samples_per_pixel_per_frame: 32
  max_bounces: 6
  max_specular_transmission_bounces: 6
  max_volume_bounces: 4
  subdiv_refinement_level: 0
```

## Lens Flares

Should you want to make Michael Bay proud, you may be interested in adding lens flares. Though lens flares in isaac are pretty basic, so not sure he'll be that impressed. Regardless, here are the available settings!

Arguments:
- `enable`: (bool), Set to true to enable the lens flare effect.
- `scale`: (float), The scale of the flare. 0 disables it.
- `blades`: (int), Number of blade used when simulating the flares.
- `aperture_rotation`: (float), Rotation of the lens' aperture.
- `sensor_diagona`l: (float), Size of the diagonal of the sensor used to simulate the flares.
- `sensor_aspect_ratio`: (float), Aspect ratio of the sensor used to simulate the flares.
- `fstop`: (float), fstop of the lens used to simulate the flares.
- `focal_length`: (float), Focal length of the lens used to simulate the flares.

> [!Note]
> These camera parameters are completely independent of the ones you have inside the scene's cameras.

Example:
```yaml
lens_flares:
  enable: False
  scale: 0.5
  blades: 9
  aperture_rotation: 0.0
  sensor_diagonal: 28.0
  sensor_aspect_ratio: 1.5
  fstop: 2.8
  focal_length: 12.0
```

## Motion Blur

To add motion blur to the rendered image, you can use the following arguments:

Arguments:
- `enable`: `(bool)`, Set to true to enable motion blur.
- `max_blur_diameter_fraction`: `(float)`, TODO.
- `exposure_fraction`: `(float)`, TODO.
- `num_samples`: `(int)`, TODO.

Example
```yaml
motion_blur:
  enable: False
  max_blur_diameter_fraction: 0.02
  exposure_fraction: 1.0
  num_samples: 8
```

## Chromatic Aberrations

To add chromatic aberrations to your renders, you can use the following arguments:

Arguments:
- `enable`: `(bool)`, Set to true to enable chromatic aberrations.
- `strength`: `((float, float, float))`, The strength of the shift for each channel. Convention: R,G,B.
- `model`: `((str, str, str))`, The model to be used to distort the colors in each channel. Convention: R,G,B. Options are "Radial", "Barrel".

Example:
```yaml
chromatic_aberration:
  enable: False
  strength: [-0.055, -0.075, 0.015]
  model: ["Radial", "Radial", "Radial"]
```

