#!/bin/bash
mkdir -p assets/Ephemeris
CWD=$(pwd)
cd assets/Ephemeris
wget https://naif.jpl.nasa.gov/pub/naif/JUNO/kernels/spk/de421.bsp
wget https://naif.jpl.nasa.gov/pub/naif/generic_kernels/fk/satellites/moon_080317.tf
wget https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/a_old_versions/pck00008.tpc
wget https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/moon_pa_de421_1900-2050.bpc
cd $CWD