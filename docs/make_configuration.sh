#!/bin/bash

rm configuration/Configurations.md
for i in $(ls configuration/configs_*); do cat $i >> configuration/Configurations.md; done
