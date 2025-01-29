#!/bin/bash

# Flag for IsaacSim version
USE_ISAAC_V4=false
while getopts ':v' opt; do
    case $opt in 
        v) 
            USE_ISAAC_V4=true
            ;;
        *) 
            echo 'Unsupported argument' >&2
            exit 1
            ;;
    esac
done

# Pulls WorldBuilder
echo "Pulling WorldBuilder"
git submodule init
git submodule update

# Install GDAL
echo "Installing GDAL"
sudo apt install python3 python3-dev python3-pip 
sudo apt-get install gdal-bin libgdal-dev
version=$(gdal-config --version)

# Install Python packages for Isaac Sim
if $USE_ISAAC_V4 
then
    echo "Installing Python packages for Isaac Sim 4.1.0"
    if [[ -e ~/.local/share/ov/pkg/isaac-sim-4.1.0/python.sh ]]
    then
        ~/.local/share/ov/pkg/isaac-sim-4.1.0/python.sh -m pip install opencv-python omegaconf hydra-core skyfield gdal==$version
        ~/.local/share/ov/pkg/isaac-sim-4.1.0/python.sh -m pip install zfpy numba empy lark
        ~/.local/share/ov/pkg/isaac-sim-4.1.0/python.sh -m pip install --upgrade numpy==1.22.0
    else
        echo "Could not find Isaac Sim 4.1.0 installation"
    fi
else
    echo "Installing Python packages for Isaac Sim 2023.1.1"
    if [[ -e  ~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh ]]
    then
        ~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh -m pip install opencv-python omegaconf hydra-core skyfield gdal==$version
        ~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh -m pip install zfpy numba empy
    elif [[ -e ~/.local/share/ov/pkg/isaac-sim-2023.1.1/python.sh ]]
    then
        ~/.local/share/ov/pkg/isaac-sim-2023.1.1/python.sh -m pip install opencv-python omegaconf hydra-core skyfield gdal==$version
        ~/.local/share/ov/pkg/isaac-sim-2023.1.1/python.sh -m pip install zfpy numba empy
    else
         echo "Could not find Isaac Sim 2023.1.1 installation"
    fi
fi

echo "Installing Python packages for default Python"
python3 -m pip --upgrade pip
python3 -m pip install gdown black gdal==$version

# Download the assets from Google Drive
echo "Downloading assets from Google Drive"
source ~/.bashrc
gdown 1XuFlDRELPQmjJFLP1E54IOhRiFn5iM_7
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
