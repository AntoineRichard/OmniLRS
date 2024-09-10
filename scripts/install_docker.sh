#!/bin/bash

# Pulls WorldBuilder
git submodule init
git submodule update

# Download the assets from Google Drive
/isaac_sim/python.sh gdown 1LfdJ8cogFU8Eid2EL-0bu9E383lftC_W
unzip assets_v6.zip
rm assets_v6.zip

# Download the DEMs of the lunar southpole and format them.
./scripts/get_dems.sh
./scripts/extract_dems_override.sh
# Get Ephemeris data
./scripts/get_ephemeris_data.sh
