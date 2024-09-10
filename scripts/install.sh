#!/bin/bash

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
rm assets_v6.zip

# Download the DEMs of the lunar southpole and format them.
./scripts/get_dems.sh
./scripts/extract_dems_override.sh
# Get Ephemeris data
./scripts/get_ephemeris_data.sh