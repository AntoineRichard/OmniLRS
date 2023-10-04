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

Assets:
 - Download the assets from: https://drive.google.com/file/d/1NpgMdD__DaU_mogeA7D-GqObMkGJ5-fN/view?usp=sharing
 - Unzip the assets inside the git repository.

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
