#!/bin/bash

# Download the assets from Google Drive
gdown 1XuFlDRELPQmjJFLP1E54IOhRiFn5iM_7
unzip assets_v6.zip
rm assets_v6.zip
gdown 1sXrsT7ZdA3VslMREBtjo6-Ou1-v8-8dI
unzip lunar_rocks.zip -d assets/USD_Assets/rocks
rm lunar_rocks.zip

# Download the DEMs of the lunar southpole and format them.
./scripts/get_dems.sh
./scripts/extract_dems_override.sh
# Get Ephemeris data
./scripts/get_ephemeris_data.sh
