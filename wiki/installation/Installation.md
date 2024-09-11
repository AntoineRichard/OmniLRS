# Installation
In this page we'll walk you through the installation process of our simulation. Since our simulation is built on top of Isaac, you will need an Nvidia GPU to run it.

Hardware requirement:
- An Nvidia GPU:
  - with 8+ Gb of VRAM (some scene will work on 4Gb)
  - RTX Series 2000 or above.
- A recent 12+ threads CPU.
- 32Gb of RAM. (for some scene 16Gb is enough)
- 10+ Gb of free space.

Operating System:
- Linux distros similar to Ubuntu 20.04 or 22.04.

> [!WARNING]
> Windows is not supported.

To install the simulation we strongly suggest using [docker](#docker-install). Though the install could also be done using a [native installation](#native-installation).

## Native installation

The first thing that needs to be done before we proceed with the native installation is to install Isaac. We support two version 2023.1.1 and 4.1.0. Though we'd recommend sticking to **2023.1.1** as there are some issues with renderings in 4.1.0. Our dockers currently come in the 2023.1.1 version of Isaac.

> [!TIP]
> If you're unsure on how to install Isaac sim, look-up the following: [How to install Isaac Sim.](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html)

To simplify the remainder of the installation process of the framework we provide a script that will automatically download all the assets, as well as install the required dependencies. It will not install Isaac Sim.
> [!IMPORTANT]
> Run this command at the root of the repository. 

```bash
scripts/install_native.sh
```

If you'd rather do it yourself, here are the commands:
```bash
# Pulls WorldBuilder
git submodule init
git submodule update

# Install GDAL
sudo apt-get install gdal-bin
sudo apt-get install libgdal-dev
version=$(gdal-config --version)

# Install Python packages for Isaac Sim
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh -m pip install opencv-python omegaconf hydra-core skyfield gdal==$version zfpy numba
python3 -m pip install --upgrade pip
python3 -m pip instal gdal==$version gdown black

# Download the assets from Google Drive
gdown 1XuFlDRELPQmjJFLP1E54IOhRiFn5iM_7
unzip assets_v6.zip
rm assets_v6.zip
gdown 1sXrsT7ZdA3VslMREBtjo6-Ou1-v8-8dI
unzip lunar_rocks.zip -d assets/USD_Assets/rocks
rm lunar_rocks.zip

# Download the DEMs of the lunar southpole and format them. This can take a long time.
./scripts/get_dems.sh
./scripts/extract_dems_override.sh
# Get Ephemeris data
./scripts/get_ephemeris_data.sh
```

Once this is done you should be off to the races!
However, before you venture forward, check that the assets folder has been created with success.
It should contain the following:
```bash
├── assets
│   ├── Ephemeris
│   ├── Terrains
│   |   ├── Lunalab
│   |   ├── Lunaryard
│   |   └── SouthPole
│   ├── Textures
│   └── USD_Assets
│       ├── common
│       ├── environments
│       ├── lunalab
│       ├── robots
│       └── rocks
```
See [getting started](#getting-started) to learn more about starting your first scene.

## Docker Installation

Before we install the simulation, please follow the procedure [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) to install all the required components to install IsaacSim in a docker container.

> [!TIP]
> You will need an [nvcr.io](https://catalog.ngc.nvidia.com/) account.

Once you're all set, the use following to build the image:
```bash
./omnilrs.docker/build_docker.sh
```

Once the image is built the simulation should be ready to go.
Though you will still need to download the assets. If you want to, you can download them from docker directly.

First pull the submodule
```bash
# Pulls WorldBuilder
git submodule init
git submodule update
```
Then start the docker
```bash
./omnilrs.docker/run_docker.sh
```
And run the script in the docker:
```bash
scritps/install_docker.sh
```
This will download the assets from docker and it should work fine. The issue is that all the generated folder will be
owned by root. So you may want to change that afterwards by running:
```bash
chown -r $USER assets
chgrp -r $USER assets
```

Provided you have Gdal and gdown installed on your system, you can also run:
```bash
scripts/download_only_native.sh
```

See [getting started](#getting-started) to learn more about starting the simulation.
