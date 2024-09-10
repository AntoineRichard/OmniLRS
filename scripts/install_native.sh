#!/bin/bash

# Pulls WorldBuilder
echo "Pulling WorldBuilder"
git submodule init
git submodule update

# Install GDAL
echo "Installing GDAL"
sudo apt-get install gdal-bin
sudo apt-get install libgdal-dev
version=$(gdal-config --version)

# Install Python packages for Isaac Sim
echo "Installing Python packages for Isaac Sim"
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh -m pip install opencv-python omegaconf hydra-core skyfield gdal==$version
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh -m pip install zfpy numba
echo "Installing Python packages for default Python"
python3 -m pip --upgrade pip
python3 -m pip install gdown black gdal==$version

# Download the assets from Google Drive
echo "Downloading assets from Google Drive"
source ~/.bashrc
gdown 1LfdJ8cogFU8Eid2EL-0bu9E383lftC_W
unzip -qq assets_v6.zip
rm assets_v6.zip
gdown 1sXrsT7ZdA3VslMREBtjo6-Ou1-v8-8dI
unzip lunar_rocks.zip -d assets/USD_Assets/rocks
rm lunar_rocks.zip

# Download the DEMs of the lunar southpole and format them.
echo "Downloading DEMs of the lunar southpole and formatting them"
./scripts/get_dems.sh
./scripts/extract_dems_override.sh
# Get Ephemeris data
echo "Getting Ephemeris data"
./scripts/get_ephemeris_data.sh
