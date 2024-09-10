# Getting started:

In this page we'll explore how you can start a simulation!
Before we get into how we launch a simulation, let's briefly see how configurations are managed.
Our simulation uses hydra to manage the configuration of the differement simulation element.

> [!TIP]
> Learn more about hydra [here](https://hydra.cc/docs/intro/)!

Our base config is organized around 4 main files:
- [environment](../environments/Environments.md): Defines everything that deals with the environment to be simulated.
- [mode](../modes/Modes.md): Defines what mode should be use, ROS1, ROS2, or synthetic data generation.
- [rendering](../rendering/Rendering.md): Defines everything that relates to the rendering.
- [physics](../physics/Physics.md): Defines everything that relates to the physics engine.

## Example commands

> [!CAUTION]
> The following assumes you are running ROS2/SpaceROS. While the code has ROS1 compatibility, we do not provide base configs or robots for ROS1.

> [!IMPORTANT]
>If you are using docker, first run the container by using:
```bash
./omnilrs.docker/run_docker.sh
```

> [!IMPORTANT]
> If you are using the native installation, make sure ROS2 is sourced **before running the simulation**.

You can then run the commands inside the docker, as if you were using the native installation. To run isaac prefix `python.sh` by `/isaac-sim/` in docker, and `~/.local/share/ov/pkg/isaac_sim-2023.1.1/` in the native installation. Before 

Run your first scene using:
```bash
python.sh run.py
```
This will launch a lunalab environment with ROS2 and ray_traced rendering.

You can run the lunaryard by using:
```bash
python.sh run.py environment=lunaryard_20m
```

You can run the largescale environment by using:
```bash
python.sh run.py environment=largescale
```

> [!TIP]
> To learn more about how to run scenes please refere to the Wiki [here](https://github.com/AntoineRichard/OmniLRS/wiki/Configuration)!

## More examples

Deformable lunalab:
```bash
python.sh run.py environment=lunalab_deformable
```

Deformable lunaryard:
```bash
python.sh run.py environment=lunaryard_20m_deformable
```

Larger lunaryard:
```bash
python.sh run.py environment=lunaryard_40m
```

