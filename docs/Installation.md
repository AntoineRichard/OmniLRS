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

> Windows is not supported.

To install the simulation we strongly suggest using docker. Though the install could also be done using a native installation.

## Native installation

The first thing that needs to be done before we proceed with the native installation is to install Isaac. We support two version 2023.1.1 and 4.1.0. Though we'd recommend sticking to **2023.1.1** as there are some issues with renderings in 4.1.0.

[How to install Isaac?](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html)

To simplify the installation process of the framework we provide an script that will automatically download all the assets, as well as install the required dependencies.

```bash
scripts/install.sh
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
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh -m pip install opencv-python omegaconf hydra-core skyfield gdal==$version zfpy, gdown, black, numba

# Download the assets from Google Drive
gdown 1LfdJ8cogFU8Eid2EL-0bu9E383lftC_W 
unzip assets_v6.zip
rm assets.zip

# Download the DEMs of the lunar southpole and format them.
./scripts/get_dems.sh
./scripts/extract_dems_override.sh
# Get Ephemeris data
./scripts/get_ephemeris_data.sh
```

Once this is done you should be off to the races!
See [getting started](GettingStarted) to learn more about starting your first scene.

## Docker Install

Before we install the simulation, please follow the procedure [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) to install all the required components. Once you're all set, the following to build the image:
```
./docker/build.sh
```

Once the image is built you should all set to run your first scene.
See [getting started](GettingStarted) to learn more about starting the simulation.
