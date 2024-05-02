# OmniLRS v1.0

In this repository, you will find the tools developped jointly by the Space Robotics group from the University of Luxembourg (SpaceR),
and the Space Robotics Lab from Tohoku University in Japan (SRL).

> Please note that this is only a partial release. The entirety of the code and assets/robots will be released at a later date.
> We will also provide docker as well as Foxglove interfaces to ease the interaction with the simulation.
> Should you run into any bug, or would like to have a new feature, feel free to open an issue.

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
 - path_tracing: A slower rendering method that provides realistic light bounces.
 - ray_tracing: A fast rendering method that does not provide pitched back shadows.

## Getting started:

### Installation & Requirements 

Software:
 - Ubuntu 20.04 or 22.04
 - ROS1 or ROS2 (if you want to use their respective modes). Note that using SDG only does not require having either installed.
 - IsaacSim-2023.1.1

Hardware:
 - An Nvidia GPU with more than 8Gb of VRAM.
 - An Nvidia GPU from the 2000 series (Turing) and up.


Native installation:
```bash
git clone --recurse-submodules https://github.com/AntoineRichard/OmniLRS.git
cd OmniLRS
git submodule init
git submodule update

sudo apt-get install gdal-bin
sudo apt-get install libgdal-dev
version=$(gdal-config --version)

~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh -m pip install opencv-python omegaconf hydra-core skyfield gdal==$version
```

Docker Installation (Comes packaged with ROS2 humble): #TODO a ROS1 (Noetic) version
```bash
git clone --recurse-submodules https://github.com/AntoineRichard/OmniLRS.git
cd OmniLRS
git submodule init
git submodule update
./docker/build_docker.sh
```

Assets:
 - Download the assets from: https://drive.google.com/file/d/1NpgMdD__DaU_mogeA7D-GqObMkGJ5-fN/view?usp=sharing
 - Unzip the assets inside the git repository. (The directory should be as shown in [Directory Structure](#directory-structure)
 - To download the lunar Southpole DEMs run the following lines:
```bash
./scripts/get_dems.sh
./scripts/extract_dems.sh
```
 - To use the stellar engine, a tool that precisely computes the position of the sun and the earth relatively to a position on the moon for a given time stamp, you will need to run the following script:
```bash
./scripts/get_ephemeris_data.sh
```


<details><summary><b>Running the sim</b></summary>

Note that the command provided below are meant to be used with the native installation.
If you are using docker, run the container using:
```bash
./docker/run_docker.sh
```
You can then run the same commands inside the docker but prefix them with `/isaac-sim/python.sh` instead of `~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh`.

Note that downloading the assets is needed to run any of the commands below.

To run the simulation we use a configuration manager called Hydra.
Inside the `cfg` folder, you will find three folders:
 - `mode`
 - `environment`
 - `rendering`

In each of these folders, there are different configuration files, that parametrized different elements of the simulation. 

For instance, to run the lunalab environment with ROS2, and ray-traced lighting one can use the following command:
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunalab mode=ROS2 rendering=ray_tracing
```
Similarly, to run the lunaryard environment with ROS2, one can use the following command:
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunaryard_20m mode=ROS2 rendering=ray_tracing
```

The rendering mode can be changed by using `rendering=path_tracing` instead of `rendering=ray_tracing`.
Changing form `ray_tracing` to path `path_tracing` tells Hydra to use `cfg/rendering/path_tracing.yaml` instead of `cfg/rendering/ray_tracing.yaml`.
Hence, if you wanted to change some of these parameters, you could create your own yaml file inside `cfg/rendering`
and let Hydra fetch it.

If you just want to modify a parameter for a given run, say disabling the lens-flare effects, then you can also edit parameters directly from the command line:
For instance:
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunaryard_20m mode=ROS2 rendering=ray_tracing rendering.lens_flares.enable=False
```

We provide bellow a couple premade command line that can be useful, the full description of the configuration files is given here:
Lunalab, ROS1
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunalab mode=ROS1 rendering=ray_tracing
```
Lunalab, ROS2 (foxy)
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunalab mode=ROS2 rendering=ray_tracing
```
Lunalab, ROS2 (humble)
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunalab mode=ROS2 rendering=ray_tracing mode.bridge_name=humble
```
Lunalab, SDG
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py environment=lunalab4SDG mode=SDG rendering=path_tracing rendering.renderer.headless=True
```
</details>

<details><summary><b>Simulation Interaction</b></summary>
Since we do not have custom topics, we had to use the base ROS topics for everything.
 Most of the simulation interactions are fairly straightforward, so we only provide details for the less obvious topics.

Interacting with the robots:
- Spawning a robot:
- Teleporting a robot:
- Reseting a robot:
- Reseting all robots:

Interacting with the terrain:
- Randomizing the terrain
- Randomizing the rocks
- Hiding the rocks

Changing the render mode:
- Path tracing
- ray tracing
 
</details>


## Citation
Please use the following citation if you use `OmniLRS` in your work.
```bibtex
@article{richard2023omnilrs,
  title={OmniLRS: A Photorealistic Simulator for Lunar Robotics},
  author={Richard, Antoine and Kamohara, Junnosuke and Uno, Kentaro and Santra, Shreya and van der Meer, Dave and Olivares-Mendez, Miguel and Yoshida, Kazuya},
  journal={arXiv preprint arXiv:2309.08997},
  year={2023}
}
```

## Directory Structure
```bash
.
├── assets
├── cfg
│   ├── environment
│   ├── mode
│   └── rendering
├── src
│   ├── configurations
│   ├── environments
│   ├── environments_wrappers
│   │   ├── ros1
│   │   ├── ros2
│   │   └── sdg
│   ├── labeling
│   ├── robots
│   ├── ros
│   └── terrain_management
└── WorldBuilders
```
