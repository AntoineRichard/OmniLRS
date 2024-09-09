# Configurations

The following provide detailed explenations on how the simulation can be configured. To managed our configurations we rely on [hydra](https://hydra.cc/docs/intro/) a configuration manager. We'll start by getting familiar with Hydra.

## How to Hydra

In short hydra allows to combined multiple .yaml file into a single configuration, it also allows to change the value of one of the file's parameters on the fly. Finally hydra provides a mean to perform simple operations inside a yaml file.

Our default configuration is the following:
```yaml
defaults:
  - environment: lunalab
  - rendering: ray_tracing
  - mode: ROS2
  - physics: default_physics

hydra:
  output_subdir: null
  run:
    dir: .
```
This means that running:
```bash
python.sh run.py
```
Will result in running the simulation using the following configs:
- cfg/environment/lunalab.yaml
- cfg/rendering/ray_tracing.yaml
- cfg/mode/ROS2.yaml
- cfg/physics/default_physics.yaml

To use a different environment one could run:
```bash
python.sh run.py environment=lunaryard_20m
```
This will load the `cfg/environment/lunaryard_20m.yaml` file instead of the lunalab one.
The same logic applies to the rest of the configs. 
To override a single parameter one could run:
```bash
python.sh run.py environment=lunaryard_20m environment.stellar_engine_settings.start_date.year=2023
```
This will change the year of the starting date of the stellar engine.

Now that you know hydra's basics, let's jump into how to configure the different environments.
