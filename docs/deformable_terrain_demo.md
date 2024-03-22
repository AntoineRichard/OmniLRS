## Deform terrain

### 0. Run IsaacSim.

*lunalab*
```bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh run.py mode=ROS1 environment=lunalab_deformable rendering=ray_tracing
```

*lunaryard*\
Note: If you do not have enough VRAM, try to commment rock settings (S1)
```bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh run.py mode=ROS1 environment=lunaryard_deformable_20m rendering=ray_tracing
```

You will see the parts of mesh which contacts with rover wheel show deformation.


### 1. Launch controller ROS node.

joystick teleoperation. \
ROS package is here: 
`
https://github.com/jnskkmhr/omni_ex1_joy
`

```bash
noetic
cd /home/lunar5/jnskkmhr/catkin_ws && source devel/setup.bash
roslaunch omni_ex1_joy ex1_ackermann_teleop.launch
```



### S1: Decrease rocks to lower computation overhead
Comment large rocks and small rocks.

```yaml
rocks_settings:
  instancers_path: /Lunaryard/Rocks
  rocks_settings:
    medium_rocks:
      seed: ${....seed}
      collections: ["apollo_rocks"] # Where to get the rock models from.
      use_point_instancer: True # If True, the rocks will be instanced using the PointInstancer.
                                # If False, it will use the custom instancer that works for SDG.
      requests: # A list of request used to distribute the rocks.
        req_pos_xy: # The name does not matter.
          attribute: Position
          axes: ["x", "y"]
          layer:
            name: Image
            # data: Is loaded automatically from the DEM.
            mpp_resolution: ${.......lunaryard_settings.resolution}
            output_space: 2
          sampler:
            name: ThomasCluster
            randomization_space: 2
            lambda_parent: 0.1
            lambda_daughter: 200
            sigma: 0.2
            seed: ${.......seed}

        req_pos_z:
          attribute: Position
          axes: ["z"]
          layer:
            name: Image
            output_space: 1
          sampler:
            name: Image
            randomization_space: 1
            # resolution: Resolution is infered automatically from the loaded DEM.
            # data: Is loaded automatically from the DEM.
            mpp_resolution: ${.......lunaryard_settings.resolution}

        req_random_z_rot:
          attribute: Orientation
          axes: ["x", "y", "z", "w"]
          layer:
            name: RollPitchYaw
            rmax: 0
            rmin: 0
            pmax: 0
            pmin: 0
            ymax: 6.28318530718
            ymin: 0
          sampler:
            name: Uniform
            randomization_space: 3
            seed: ${.......seed}

        req_scale:
          attribute: Scale
          axes: ["xyz"]
          layer:
            name: Line
            xmin: 1.0
            xmax: 2.0
          sampler:
            name: Uniform
            randomization_space: 1
            seed: ${.......seed}

    # large_rocks:
    #   seed: ${....seed}
    #   collections: ["apollo_rocks"] # Where to get the rock models from.
    #   use_point_instancer: True # If True, the rocks will be instanced using the PointInstancer.
    #                             # If False, it will use the custom instancer that works for SDG.
    #   parent: medium_rocks
    #   requests: # A list of request used to distribute the rocks.
    #     req_pos_xy: # The name does not matter.
    #       attribute: Position
    #       axes: ["x", "y"]
    #       layer:
    #         name: Image
    #         # data: Is loaded automatically from the DEM.
    #         mpp_resolution: ${.......lunaryard_settings.resolution}
    #         output_space: 2
    #       sampler:
    #         name: ThomasCluster
    #         randomization_space: 2
    #         lambda_parent: 0.25
    #         lambda_daughter: 1
    #         sigma: 0.4
    #         inherit_parents: True
    #         seed: ${.......seed}

    #     req_pos_z:
    #       attribute: Position
    #       axes: ["z"]
    #       layer:
    #         name: Image
    #         output_space: 1
    #       sampler:
    #         name: Image
    #         randomization_space: 1
    #         # resolution: Resolution is infered automatically from the loaded DEM.
    #         # data: Is loaded automatically from the DEM.
    #         mpp_resolution: ${.......lunaryard_settings.resolution}

    #     req_random_z_rot:
    #       attribute: Orientation
    #       axes: ["x", "y", "z", "w"]
    #       layer:
    #         name: RollPitchYaw
    #         rmax: 0
    #         rmin: 0
    #         pmax: 0
    #         pmin: 0
    #         ymax: 6.28318530718
    #         ymin: 0
    #       sampler:
    #         name: Uniform
    #         randomization_space: 3
    #         seed: ${.......seed}

    #     req_scale:
    #       attribute: Scale
    #       axes: ["xyz"]
    #       layer:
    #         name: Line
    #         xmin: 5.0
    #         xmax: 10.0
    #       sampler:
    #         name: Uniform
    #         randomization_space: 1
    #         seed: ${.......seed}

    # small_rocks:
    #   seed: ${....seed}
    #   collections: ["apollo_rocks"] # Where to get the rock models from.
    #   use_point_instancer: True # If True, the rocks will be instanced using the PointInstancer.
    #                             # If False, it will use the custom instancer that works for SDG.
    #   parent: medium_rocks
    #   requests: # A list of request used to distribute the rocks.
    #     req_pos_xy: # The name does not matter.
    #       attribute: Position
    #       axes: ["x", "y"]
    #       layer:
    #         name: Image
    #         # data: Is loaded automatically from the DEM.
    #         mpp_resolution: ${.......lunaryard_settings.resolution}
    #         output_space: 2
    #       sampler:
    #         name: ThomasCluster
    #         randomization_space: 2
    #         lambda_parent: 0.3
    #         lambda_daughter: 100
    #         sigma: 0.5
    #         inherit_parents: True
    #         seed: ${.......seed}

    #     req_pos_z:
    #       attribute: Position
    #       axes: ["z"]
    #       layer:
    #         name: Image
    #         output_space: 1
    #       sampler:
    #         name: Image
    #         randomization_space: 1
    #         # resolution: Resolution is infered automatically from the loaded DEM.
    #         # data: Is loaded automatically from the DEM.
    #         mpp_resolution: ${.......lunaryard_settings.resolution}

    #     req_random_z_rot:
    #       attribute: Orientation
    #       axes: ["x", "y", "z", "w"]
    #       layer:
    #         name: RollPitchYaw
    #         rmax: 0
    #         rmin: 0
    #         pmax: 0
    #         pmin: 0
    #         ymax: 6.28318530718
    #         ymin: 0
    #       sampler:
    #         name: Uniform
    #         randomization_space: 3
    #         seed: ${.......seed}

    #     req_scale:
    #       attribute: Scale
    #       axes: ["xyz"]
    #       layer:
    #         name: Line
    #         xmin: 0.01
    #         xmax: 0.05
    #       sampler:
    #         name: Uniform
    #         randomization_space: 1
    #         seed: ${.......seed}
```

### S2: Switch texture
Uncomment one of them.
```yaml
texture_path: /Lunaryard/Looks/Basalt
  # texture_path: /Lunaryard/Looks/Sand
  # texture_path: /Lunaryard/Looks/Regolith_footprint
  # texture_path: /Lunaryard/Looks/Synthetic_Regolith_02
```