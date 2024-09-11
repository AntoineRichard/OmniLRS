 Welcome to OmniLRS's Wiki

<center>
<img src="./media/Logov2.png" width=520/>
</center>

Omniverse Lunar Robotics Simulator or OmniLRS, is a simulation tools developped jointly by the Space Robotics group from the University of Luxembourg (SpaceR),
and the Space Robotics Lab from Tohoku University in Japan (SRL).

> [!IMPORTANT]
> We are now opening it to the community and strongly encourage Space Roboticists to help us grow the feature set of this simulation! Don't be shy shoot a PR!

## Integration & Workflow with SpaceROS
> [!IMPORTANT]
> Isaac Sim is using ROS2 by default, most of the tools available in Isaac are meant for ROS2. Hence, this simulation uses ROS2. To use this simulation with SpaceROS, the ROS2 simulation docker must first be spinned up, and then in a second time, another container running SpaceROS must be launched to interact with the simulation.
> To illustrate this, we provide a simple teleop demonstration with the sim in ROS2 and SpaceROS sending velocity commands.
> Check the Wiki for a [step-by-step guide](#run-with-spaceros) on how to run this demo.

## Simulation Environments Overview

|  <div style="width:70px">Name</div>  |  <div style="width:230px">Description</div>  | Images            |
|------------|-------------|---------------------------------|
| **Lunalab**            |  <div style="width:230px"> Digital-Twin of lunar analog at the University of Luxembourg. This environment also supports terrain deformation as the rover drives on it. </div> | <img src="media/env_img/lunalab.png" width=520/> |
| **Lunaryard**            |  <div style="width:230px">A small scale procedually generated lunar environment. If lunar coordinates and a date is provided the position of the earth and sun are computed using ephemerides resulting in realistic lighting. This feature is also available in the large scale environments. This environment also support terrain deformation as the rover drives on it.</div>  | <img src="media/env_img/lunaryard_husky_ex1.png" width=520/> |
| **LargeScale**           |  <div style="width:230px">Semi procedural lunar environment. It uses real DEM to reconstuct the coarse terrain, usually 5meters per pixel and then uses procedural generation to augment it to 2.5cm per pixel. The terrain itself can be generated at a even higher resolution to smooth out shadows. This very fine terrain allows to reconstruct fine terrain features increasing the engineering value of the sim. The whole of this is bundled inside Geometry clip maps, allowing to render very large scenes.</div> | <img src="media/env_img/large_scale.png" width=520/>

> [!NOTE]
> Please note that this is a partial release. More robots will be made available at a later date. Should you run into a bug, or would like to request a new feature, feel free to open an issue. Want to collaborate, reach out to us!

## Wiki's overview

> [!WARNING]
> This wiki is still under construction, if you are looking for something in particular that's not yet docummented please open an issue.

- [Installation](https://github.com/AntoineRichard/OmniLRS/wiki/Installation)
- [Getting Started](https://github.com/AntoineRichard/OmniLRS/wiki/GettingStarted)
- [Interacting with the Scene](https://github.com/AntoineRichard/OmniLRS/wiki/ros_topics)
- [Configuring simulation modes](https://github.com/AntoineRichard/OmniLRS/wiki/Modes)
- [Configuring the environments](https://github.com/AntoineRichard/OmniLRS/wiki/Environments)
- [Deformation engine](https://github.com/AntoineRichard/OmniLRS/wiki/DeformationEngine)
- [Configuring robots](https://github.com/AntoineRichard/OmniLRS/wiki/Robots#how-to-set-up-robots)
- [Configuring the rendering](https://github.com/AntoineRichard/OmniLRS/wiki/Physics)
- [Configuring the physics](https://github.com/AntoineRichard/OmniLRS/wiki/Rendering)
- [Contribute](https://github.com/AntoineRichard/OmniLRS/wiki/Contribute)
- [FAQ (WIP)](https://github.com/AntoineRichard/OmniLRS/wiki/FAQ)

## Run with SpaceROS

> [!IMPORTANT]
> The following assumes you've gone through the [installation process](https://github.com/AntoineRichard/OmniLRS/wiki/Installation) and the [getting started process](https://github.com/AntoineRichard/OmniLRS/wiki/GettingStarted). It also assumes you went with the docker install. However, we provide an alternative option to run the simulation using a native installation.

### Simple Teleoperation Demo
In the following we provide a basic example to exaplain the workflow between OmniLRS and SpaceROS. This workflow extends
to other ROS style development.

First we will start by spinning up the simulation's docker:
```bash
./omnilrs.docker/run_docker.sh
```

> [!TIP]
> If you installed the simulation in a native environment (no docker) you can simply run `PATH_TO_ISAAC/python.sh run.py`.
> `PATH_TO_ISAAC` is typically `~/.local/share/ov/pkg/isaac_sim-2023.1.1`.

It will open an interactive terminal in which you can start a simulation:
```
/isaac-sim/python.sh run.py
```
This will open a simulation with the lunalab and a Husky in it.



> [!IMPORTANT]
> Build the docker for the demo by running: `./spaceros_demo.docker/build_docker.sh`

Now we will start the SpaceROS docker
```bash
./spaceros_demo.docker/run_docker.sh
```
This will open an interactive terminal with spaceROS already sourced and a teleoperation built natively. 

You can now run the teleoperation node:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Following the instructions on screen you should be able to visualize the robot moving in the lab!

### Justification
We believe the simulation should be an independent component from the software that's running in it.
This seperation of the components allows for multiple things. Say you'd like to test a navigation algorithm on a very specific type of hardware,
well, you can have a powerful computer running the simulation, and that specialized hardware connected to the same network that will
interact with the simulation. Another argument is that this separation also eases the deployments of algorithms.
If they are already built in a separate docker, this docker can be directly shipped to a robot for real world validation.