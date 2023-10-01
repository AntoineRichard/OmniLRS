# OmniLRS v1.0

In this repository, you will find the tools developped jointly by the Space Robotics group from the University of Luxembourg (SpaceR),
and the Space Robotics Lab from Tohoku University in Japan (SRL).

With this initial release we provide our small scale environments:
 - The lunalab 
 - The lunaryard (3 versions 20m, 40m, 80m)

We also provide 3 operation modes:
 - ROS1: allows to run ROS1 enabled robots
 - ROS2: allows to run ROS2 enabled robots
 - SDG: or Synthetic Data Generarion, allows to capture synthetic data to train neural-networks.

For both ROS1 and ROS2 we prepared 4 different robots:
 - EX1: SRL's own rover.
 - Leo Rover: a rover from XXX used by SpaceR.
 - Husky: the rover from Clearpath Robotics.
 - Turtlebot: A popular educational robot.

Finally, we provide simple configurations for different renderers:
 - path_tracing: A slower rendering method that provides realisitic light bounces.
 - ray_tracing: A fast rendering method that does not provide pitched back shadows.

## Getting started:

### Requirements:

Sofware:
 - Ubuntu 20.04 or 22.04
 - ROS1 or ROS2 (if you want to use their respective modes). Note that using SDG only does not require having either installed.
 - IsaacSim-2022.2.1

Harware:
 - An Nvidia GPU with more that 8Gb of VRAM.
 - An Nvidia GPU from the 2000 series (Turing) and up.

### Running the sim:

To run the simulation we use a configuration manager called Hydra.
Inside the `cfg` folder, you will find three folders:
 - `mode`
 - `environment`
 - `rendering`

In each of these folders, there are different configuration files, that parametrized different elements of the simulation. 

For instance, to run the lunalab environment with ROS2, and ray-traced lighting one can use the following command:
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunalab mode=ROS2 rendering=ray_traced
```
Similarly, to run the lunaryard environment with ROS2, one can use the following command:
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunaryard_20m mode=ROS2 rendering=ray_traced
```

The rendering mode can be changed by using `rendering=path_traced` instead of `rendering=ray_traced`.
Changing form `ray_traced` to path `path_traced` tells Hydra to use `cfg/rendering/path_traced.yaml` instead of `cfg/rendering/ray_traced.yaml`.
Hence, if you wanted to change some of these parameters, you could create your own yaml file inside `cfg/rendering`
and let Hydra fetch it.

If you just want to modify a parameter for a given run, say disabling the lens-flare effets, then you can also edit parameters directly from the command line:
For instance:
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunaryard_20m mode=ROS2 rendering=ray_traced rendering.lens_flares.enable=False
```

We provide bellow a couple premade command line that can be useful, the full description of the configuration files is given here:
Lunalab, ROS1
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunalab mode=ROS1 rendering=ray_traced
```
Lunalab, ROS2
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunalab mode=ROS2 rendering=ray_traced

```
Lunalab, SDG
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunalab4SDG mode=SDG rendering=path_traced rendering.renderer.headless=True
```

### ROS1:




With a ROS Core already started run:
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh lunalab_ros1.py
```

You should see Isaac Sim starting the scene with the lab. Once the scene is loaded you can navigate around or use one of our readily available cameras.


### ROS2:

run:
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh lunalab_ros1.py
```

You should see Isaac Sim starting the scene with the lab. Once the scene is loaded you can navigate around or use one of our readily available cameras.

### Useful topics:
 - Name:`/Lunalab/Projector/TurnOn`, type:`bool`, function: Turns the projector light on and off. Usage: Send True to turn on, False to turn off.
 - Name:`/Lunalab/Projector/Intensity`, type:`float32`, function: Changes the intensity of the light. Usage: Send a percentage of the default value, sending 50 will half the intensity of the light. 
 - Name:`/Lunalab/Projector/Radius`, type:`float`, function: Changes the radius of the light source, default: 0.01. Usage: N/A.
 - Name:`/Lunalab/Projector/Pose`, type:`pose`, function: Changes the pose of the projector in the scene. Usage: N/A.
 - Name:`/Lunalab/Projector/Color`, type:`rgba`, function: Changes the color of the light source. Usage: Send the value as RGBA, A is ignored.
 - Name:`/Lunalab/CeilingLights/TurnOn`, type:`bool`, function: Turns the ceiling lights on and off. Usage: Send True to turn on, False to turn off.
 - Name:`/Lunalab/CeilingLights/Intensity`, type:`float`, function: Changes the intensity of the light on the ceiling. Usage: Send the intensity value.
 - Name:`/Lunalab/CeilingLights/Radius`, type: `float`, function: Changes the radius of the light sources. Usage: N/A.
 - Name:`/Lunalab/CeilingLights/FOV`, type: `float`, function: Changes the width of the light cone. Usage: Send the value in degrees.
 - Name:`/Lunalab/CeilingLights/Color`, type: `rgba`, function: Changes the color of the light sources. Usage: Send the value as RGBA, A is ignored.
 - Name:`/Lunalab/Curtains/Extend`, type: `bool`, function: Extends or folds the curtains. Send True to extend, send False to fold.
 - Name:`/Lunalab/Terrain/Switch`, type: `int8`, function: Changes the terrain. Usage: Send 0 for the first terrain, send 1 for the second, sending 2 (or higher) will likely result in a crash.
 - Name:`/Lunalab/Terrain/EnableRocks`, type: `bool`, function: Enables the rocks in the scene. Usage: Send True to enable rocks, send False to disable rocks.
 - Name:`/Lunalab/Terrain/RandomizeRocks`, type: `int32`, function: Randomizes the position of N rocks. Usage: Send the number of rocks you would like to have on the terrain.
 - Name:`/Lunalab/Terrain/PlaceRocks`, type: `str`, function: WIP. Usage: WIP.
 - Name:`/Lunalab/Render/EnableRTXRealTime`, type:`empty`, function: Switch the render mode to ray-traced (light). Usage: N/A.
 - Name:`/Lunalab/Render/EnableRTXInteractive`, type:`empty`, function: Switch the render mode to path-traced (heavy). Usage: N/A.
 - Name:`/Lunalab/LenseFlare/EnableLensFlares`, type:`bool`, function: Enables the addition of lens flares during the post-processing stage. Usage: Send True to enable, send False to disable.
 - Name:`/Lunalab/LenseFlare/NumBlades`, type:`int8`, function: Changes the number of blades.
 - Name:`/Lunalab/LenseFlare/Scale`, type:`float`, function: Changes the scale of the effect.
 - Name:`/Lunalab/LenseFlare/ApertureRotation`, type:`float`, function: Rotates the flare.
 - Name:`/Lunalab/LenseFlare/FocalLenght`, type:`float`, function: 
 - Name:`/Lunalab/LenseFlare/Fstop`, type:`float`, function: 
 - Name:`/Lunalab/LenseFlare/SensorAspectRatio`, type:`float`, function: 
 - Name:`/Lunalab/LenseFlare/SensorDiagonal`, type:`float`, function: 


 ### Example of commands to run:

 - rostopic pub -r 5 /Lunalab/Terrain/RandomizeRocks std_msgs/Int32 "data: 16" 
 - rostopic pub /Lunalab/Terrain/Switch std_msgs/Int8 "data: 1" 
 - rostopic pub /Lunalab/LensFlare/EnableLensFlares std_msgs/Bool "data: true"