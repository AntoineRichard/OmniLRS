 Welcome to OmniLRS's Wiki

<center>
<img src="media/Logov2.png" width=520/>
</center>

Omniverse Lunar Robotics Simulator or OmniLRS, is a simulation tools developped jointly by the Space Robotics group from the University of Luxembourg (SpaceR),
and the Space Robotics Lab from Tohoku University in Japan (SRL).

> [!IMPORTANT]
> We are now opening it to the community and strongly encourage Space Roboticists to help us grow the feature set of this simulation! Don't be shy shoot a PR!


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

- [Installation](installation/Installation.md)
- [Getting Started](getting_started/GettingStarted.md)
- [Interracting with the Scene](scene_interaction/ros_topics.md)
- [Configuring simulation modes](modes/Modes.md)
- [Configuring the environments](environments/Environments.md)
- [Configuring the rendering](rendering/Rendering.md)
- [Configuring the physics](physics/Physics.md)
- [Contribute](contribute/Contribute.md)
- [FAQ](FAQ/FAQ.md)
