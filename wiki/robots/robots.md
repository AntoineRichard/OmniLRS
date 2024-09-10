# How to set up Robots!?

In this page, we will explore how robots can be integrated to OmniLRS.

> [!Note](#how-to-set-up-robots)
> Isaac provides many ways to import, create, or modify existing robots. This can be done through a URDF importer, a mujoco importer, or an Onshape importet. Robots can also be rigged manually from within the UI. We encourage anyone non-familiar with Isaac Sim to [take a look at Isaac's Sim comprehensive suite of tutorials for ROS/ROS2](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#).

## Robots and where to find them

By default, we provide two robots to be used in the simulation. These are generic non-space robots.
The reasoning behind that is that Space hardware is so unique that end-users would either use generic off-the-shelf robots or highly specialized ones.
Hence we settled with two popular options from [ClearPath Robotics](https://clearpathrobotics.com/) (the Husky and the Jackal), which we fitted with a basic suite of sensors.

Available robots & sensor payloads:
- Husky:
  - Velodyne VLP16: `ros2_husky_PhysX_vlp16.usd`
  - Velodyne VLP16 + RGB + Depth + IMU: `ros2_husky_PhysX_vlp16_mono_depth_imu.usd`
- Jackal:
  - Velodyne VLP16: `ros2_jackal_PhysX_vlp16.usd`
  - Velodyne VLP16 + RGB Left + RGB Right + IMU: `ros2_jackal_PhysX_vlp16_stereo.usd`

> [!CAUTION]
> These robots are ROS2 capable only. Their omnigraphs are setup for ROS2. This makes them compatible with SpaceROS.

> [!NOTE]
> This list will grow in the coming month as we try more robots onto it. We are planning on adding Leo Rovers and Quadrupeds in the near future.

## Changing the default robot used when starting an environment

To change the default robot used when starting an environment, you need to edit the `robot_settings` inside the environment config.
The `robot_settings` who defines the set of parameters managing robots provides the following set of parameters.

Arguments:
- `uses_nucleus`: `(str)`, whether or not the robot loader should try to find the robot on Omniverse Nucleus cloud. True means the robot is on nucleus. False the robot is local.
- `is_ROS2`: `(bool)`, if the robot is ROS2 enabled. True if the robot is using ROS2.
- `max_robots`: `(int)`, The maximum amount of robots in the scene.
- `robots_root`: `(str)`, The path where the robots are generated in the scene.
- `parameters`: `(list(RobotsParameters))`, A list of parameters defining robots.

> [!IMPORTANT]
> Large scale environments only support one robot at a time. This is due to the type of algorithms used to render the terrain.

To define a robot one can use the following arguments:
- `robot_name`: `(str)`, the name of the robot. It's mostly used to teleport it or reset it.
- `usd_path`: `(str)`, the path of the robot on nucleus or locally.
- `pose`:
  - `position`: `(float,float,float)` x, y, z position.
  - `orientation`: `(float,float,float)` w, x, y, z quaternion.
- `domain_id`: `int`, The domain id of the robot. (not functional)
- `target_links`: `(list(str))` The links used to compute the deformation forces when derforming terrains.

> [!IMPORTANT]
> For the large scale environments the position and orientation of the robot is ignored and the `starting_position` parameter is used instead.
> The Z position of the robot is computed automatically.


Example:
```bash
robots_settings:
  uses_nucleus: False
  is_ROS2: True
  max_robots: 5
  robots_root: "/Robots"
  parameters:
      -
      robot_name: husky
      usd_path: assets/USD_Assets/robots/ros2_husky_PhysX_vlp16.usd
      pose:
        position: [5.0, 5.0, 0.5]
        orientation: [1, 0, 0, 0]
      domain_id: 0
      target_links: ["front_left_wheel_link", "front_right_wheel_link", "rear_left_wheel_link", "rear_right_wheel_link"]
```