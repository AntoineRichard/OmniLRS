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
## Environments Configs

In the following we go through the different environments as well as the settings that can be used to tune them.
Here is the list of default environments that can be generated:
- The [Lunalab](#lunalab), an digital twin of the lunalab at the university of Luxembourg.
- The [lunaryard](#lunaryard), a small procedural environment meant for basic testing.
- The [large_scale](#largescale), a large scale semi-procedural environment meant for full fledged tests.

Upcoming environments:
- The `rover_scape` from NASA Ames, at the NASA ARC, CA, USA.
- The `the_moon_rostock` from PTS space in Rostock's Airport, Germany.
- The `confined_large_scale`, a visually large environment that allows for terrain deformation.

> All the environments are seeded and unique per seed. That means that they are also reproducible. Given a seed the environment should always be the same.


### Lunalab

The `lunalab` environment is a replica of the Lunalab [1] from the University of Luxembourg. This 10.0 by 6.5 meter
environment allows to test rovers at the uni of luxembourg. The ground is made of small basalt rocks, and can be used
to perform small scale tests prior to large scale testing in analog environments. This environment was modeled in
blender and provides 2 terrain that were scanned using a total station.

> This environment is compatible with deformable terrains!

![Sample image lunalab](../media/env_img/lunalab.png)


Environment arguments:
- `projector_position`: `(float3)`, The position of the projector in the scene. To replicate similar scenarios to that of the lab, we'd recommend using the following settings: \[0.5\<X\<6.0, 0.0, 0.2\<Z\<2.0\]
- `projector_orientation`: `(float4)`, The pose of the projector in the scene as a quaternion. Convention: (w,x,y,z).
- `projector_on`: `(bool)`, If the projector should be turned on when the scene is loaded. True means the projector will be on.
- `ceiling_lights_on`: `(bool)`, If the ceiling lights should be turned on when the scene is loaded. True means the ceiling lights will be on.


The lunaryard can be futher customized by using one of the following objects:
- [Rock Generator](#extras-rocks), an object that can spawn rocks on the generated terrain.
- [Terrain Generator](#extras-terrain), an object that is used to define how procedurally generated terrain look. It also allows to set the terrain deformation parameters as well as loading DEM.

> A complete example of a lunalab environment configuration can be found in: `cfg/environment/lunalab.yaml`

### Lunaryard

The `lunaryard` is a procedural small scale environment (from 10 to 80 meters) meant to perform simple tests with different algorithms. It's simplicity means it will easily run on most machines and will allow to perform basic debugging steps.

> This environment is compatible with deformable terrains!

![Sample image lunaryard](../media/env_img/lunaryard_husky_ex1.png)

Environment arguments:
- `lab_length`: `(float)`, The length of the lab in meters. While there is no enforced limits we would recommend staying below 100 meters.
- `lab_width`: `(float)`, The width of the lab in meters. While there is no enforced limits we would recommend staying below 100 meters.
- `resolution`: `(float)`, The size of the pixels used to generate the map from which the terrain is derived. Also know as meters-per-pixel (mpp).
- `terrain_id`: `(int)`, The id of the terrain. More on that in [Terrain Generator](#extras-terrain).
- `coordinates`: `(Coordinates)`, The [coordinates](#coordinates) of the terrain. This is used by the [stellar engine](#stellar-engine) to compute accurate sun and earth positions. 
- `earth_elevation`: `(float)`, The elevation of the earth in degrees. Overwritten if the [stellar engine](#stellar-engine) is enabled.
- `earth_azimuth`: `(float)`, The azimuth of the earth in degrees. Overwritten if the [stellar engine](#stellar-engine) is enabled.
- `earth_distance`: `(float)`, The distance between the earth and the moon in meters (Give a non-scaled value). Overwritten if the [stellar engine](#stellar-engine) is enabled.

The lunaryard can be futher customized by using one of the following objects:
- [Sun](#sun), an object allowing to tune the sun settings.
- [Stellar Engine](#extra-sun), an object allowing to move the earth and sun based on the current time and coordinates on the moon.
- [Rock Generator](#extras-rocks), an object that can spawn rocks on the generated terrain.
- [Terrain Generator](#extras-terrain), an object that is used to define how procedurally generated terrain look. It also allows to set the terrain deformation parameters as well as loading DEM.

> A complete example of a lunaryard environment configuration can be found in: `cfg/environment/lunayard_20m.yaml`


### LargeScale

The `large_scale` environment is meant to fully test navigation and mapping algorithms. It offers the ability to perform
very long traverses (20km+) while maintaining a locally very high resolution terrain mesh. This environment is a lot
more resource intensive than the others, be it on the CPU, RAM, GPU, and VRAM.

> This environment is **not** compatible with deformable terrains yet.

![Sample image large scale](../media/env_img/large_scale.png)

This environment offers significantly more configuration settings than the others. Though we would recommend not to change them.

Environment arguments:
- `seed`: `(int)`, The global seed for the configuration. It automatically sets all the other seeds involved in the generation of the environment. Can be overriden.
- `crater_gen_seed`: `(int)`, Overrides the global seed.
- `crater_gen_distribution_seed`: `(int)`, Overrides the global seed.
- `crater_gen_metadata_seed`: `(int)`, Overrides the global seed.
- `rock_gen_main_seed`: `(int)`, Overrides the global seed.

- `profiling`: `(bool)`, Whether the profiler should be turned on. True means the profiler is enabled.
- `update_every_n_meters`: `(float)`, The amount of distance that need to be crossed to trigger an update of the terrain.
- `z_scale`: `(float)`, The scaling value for height of the terrains.
- `block_size`: `(int)`, The size in meters of a terrain block.

- `dbs_max_elements`: `(int)`, The maximum number of elements that can be stored in a databese.
- `dbs_save_to_disk`: `(bool)`, Whether the databases should be saved on disk. Not enabled yet.
- `dbs_write_interval`: `(int)`, How often the databases should be wrote to disk after an update.

- `hr_dem_resolution`: `(float)`, The resolution in meters per pixel of the procedurally generated high resolution map.
- `hr_dem_generate_craters`: `(bool)` Whether craters should be added onto the high resolution map. If no craters are generated, the high resolution map is then just a bicubic interpolation of the original DEM. If true generates craters.
- `hr_dem_interpolation_padding`: `(int)` The amount of padding used when interpolating the original DEM, we recommend setting this to 2, as it's the amount required to perform bicubic interpolation without artefacts.

- `lr_dem_folder_path`: `(str)`, Path to the folder containing the low-resolution DEM.
- `lr_dem_name`: `(str)`, Name of the low-resolution DEM file.
- `starting_position`: `(tuple)`, Starting position of the terrain generation in meters from the center of the map.

- `geo_cm_num_texels_per_level`: `(int)`, Number of texels per level in the geometry clip map.
- `geo_cm_target_res`: `(float)`, The target resolution of the geometry clip map.
- `geo_cm_fine_interpolation_method`: `(str)`, Method used for the interpolation of the fine geometry clip map, options are `bilinear` or `bicubic`.
- `geo_cm_coarse_interpolation_method`: `(str)`, Method used for the interpolation of the coarse geometry clip map, options are `bilinear` or `bicubic`.
- `geo_cm_fine_acceleration_mode`: `(str)`, The mode of acceleration for the fine geometry clip map update, options are `gpu` or `hybrid`.
- `geo_cm_coarse_acceleration_mode`: `(str)`, The mode of acceleration for the coarse geometry clip map update, options are `gpu` or `hybrid`.
- `geo_cm_apply_smooth_shading`: `(bool)`, Whether to apply smooth shading to the geometry clip maps.
- `geo_cm_semantic_label`: `(str)`, A label for the type of surface being generated, e.g., `terrain`.
- `geo_cm_texture_name`: `(str)`, Name of the texture to use for the terrain surface.
- `geo_cm_texture_path`: `(str)`, Path to the texture file for the terrain surface.

- `terrain_collider_enabled`: `(bool)`, Whether the terrain collider should be enabled.
- `terrain_collider_resolution`: `(float)`, The resolution of the terrain collider in meters.
- `terrain_collider_mode`: `(str)`, Mode for terrain collider generation, e.g., `meshSimplification`.
- `terrain_collider_cache_size`: `(int)`, Size of the cache for storing terrain colliders. That is the maximum amount of `block_size`x`block_size` meters colliders used in the scene.
- `terrain_collider_building_threshold`: `(float)`, Threshold for building terrain colliders. Minimum distance from a collider boundary to trigger the generation of a new ones. Can affect colliders-based lidars. 

- `rock_gen_cfgs`: `(list)`, List of configurations for rock generation, each configuration specifying parameters for rock size, density, and placement.

> Notes & considerations:
> - All seeds can be individually overridden, but if not provided, the global seed is used by default.
> - Queue sizes for the terrain generation process affect memory usage and parallelism performance.
> - The geometry clip map parameters allow fine-tuning of how terrain is generated, interpolated, and shaded. Increasing the number of texels and reducing the resolution can have adverse effects on the update time of the terrain's visual mesh.
> - In Isaac, colliders cannot be generated asynchronously from the main simulation thread. Hence, they are only generated when necessary. If your robot is using a "PhysX based lidar", it will not seen the terrain as the robot comes close to the edge of a collider. Use an "RTX-based lidar" instead.


The large scale environments can be futher customized by using one of the following objects:
- [Sun](#sun), an object allowing to tune the sun settings.
- [Stellar Engine](#extra-sun), an object allowing to move the earth and sun based on the current time and coordinates on the moon.

> A complete example of a large_scale environment configuration can be found in: `cfg/environment/largescale.yaml`


## Extras: Light Sources

In the following we detail how the sun can be customized. This argument is only applicable to "open space" environments such as the [LargeScale](#largescale) and [Lunaryard](#lunaryard).

### Sun

This allows to customize the default parameters for the Sun in the [LargeScale](#largescale) and [Lunaryard](#lunaryard) environments.

Parameters:
- `intensity`: `(float)`, The intensity of the sun light. The higher the value, the brighter the scene will be.
- `angle`: `(float)`, The angle of the sun light in degrees. Note that this denotes the size of the sun in sky, not its azimuth or elevation. The larger the angle, the bigger the sun will appear.
- `diffuse_multiplier`: `(float)`, A multiplier for the effect of the sun has on the diffuse response of materials. It can be used to create stronger flares by setting this value to 0.1 and multiplying the light intensity by 10.
- `specular_multiplier`: `(float)`, A multiplier for the effect the sun has on the specular response of materials. It can be used to create stronger flares by setting this value to 0.1 and multiplying the light intensity by 10.
- `color`: `((float, float, float))`, The color to be given to the light. Format is (R,G,B), with 0.0 <= R,G,B <= 1.0.
- `temperature`: `(float)`, The temperature of the light source in Kelvin.
- `azimuth`: `(float)`, The azimuth of the sun in degrees. If the [Stellar Engine](#stellar-engine) is enabled it will override this parameter.
- `elevation`: `(float)`, The elevation of the sun in degrees. If the [Stellar Engine](#stellar-engine) is enabled it will override this parameter.

Example:
```yaml
sun_settings:
  intensity: 1750.0
  angle: 0.53
  diffuse_multiplier: 1.0
  specular_multiplier: 1.0
  color: [1.0, 1.0, 1.0]
  temperature: 6500.0
  azimuth: 180.0
  elevation: 45.0
```

### Coordinates

This allows to set the coordinates of the [Lunaryard](#lunaryard) environment. The coordinate is used by the [Stellar Engine](#stellar-engine) to compute the position of the sun and earth for a given date.

Parameters:
- `latitude`: `(float)`, The latitude of the environment in degrees.
- `longitude`: `(float)`, The longitude of the environment in degrees.

Example:
```yaml
coordinates:
  latitude: 46.8
  longitude: -26.3
```

### Date

This allows to set the date of the [LargeScale](#largescale) and [Lunaryard](#lunaryard) environments. The date is used by the [Stellar Engine](#stellar-engine) to compute the position of the sun and earth for a given coordinate on the moon.

Parameters:
- `year`: `(int)`, The year.
- `month`: `(int)`, The month.
- `day`: `(int)`, The day.
- `hour`: `(int)`, The hour.
- `minute`: `(int)`, The minute.

Example:
```yaml
start_date:
  year: 2024
  month: 5
  day: 1
  hour: 12
  minute: 50
```
Used inside the [Stellar Engine](#stellar-engine)'s configuration.

### Stellar Engine

The Stellar uses both the [Date](#date) and the [Coordinates](#coordinates) to estimate the position of the sun and earth in the lunar sky.

<span style="color: red;">**Known limitations**:</span> We use [skyfield] to compute the position of the earth and sun. This python library does not support well multi-segment ephemeris. Hence we recommend to use the ones provided with the sim.

Parameters:
- `start_date`: `(Date)`, a date as defined in [Date](#date-configuration).
- `time_scale`: `(float)`, a time multiplier that allows to make time pass faster of slower.
- `update_interval`: `(float)`, the amount of scaled time that must pass for the position of the stellar bodies to be updated.
- `distance_scale`: `(float)`, the scaling factor to be applied to the distance between the center of the scene and the earth. A value below 1 will lead to the distance being reduced.
- `ephemeris_path`: `(str)`, the path to the ephemeris data.
- `ephemeris`: `(str)`, the name of the ephemeris.
- `moon_pa`: `(str)`, the name of the "spice binary lunar pck".
- `moon_tf`: `(str)`, the name of the "frame kernel".
- `pck`: `(str)`, the name of the "pck".
- `frame`: `(str)`, the name of the frame to be used.

Example:
```yaml
stellar_engine_settings:
  start_date:
    year: 2024
    month: 5
    day: 1
    hour: 12
    minute: 50
  time_scale: 1
  update_interval: 600
  distance_scale: 0.001
  ephemeris_path: assets/Ephemeris
  ephemeris: de421.bsp
  moon_pa: moon_pa_de421_1900-2050.bpc
  moon_tf: moon_080317.tf
  pck: pck00008.tpc
  frame: MOON_ME_DE421
```

## Extras: Rocks

In this section we provide some details regarding the distribution of rocks in the scene. This is only applicable to small scale environments such as the [Lunalab](#lunalab) or the [Lunaryard](#lunaryard). Large scale environments rely on different methods to generate and manage rocks.

To generate rocks we rely 3 different objects. The base block is a [request](#request), it assigns a type of parameter to be randomized with a given distribution. A [group of requests](#request-group), that is a bundle of requests. And finally a [rock generation configuration](#rock-generation).
How to set up these is explained below:
- [request](#request)
- [request group](#request-group)
- [rock generation configuration](#rock-generation)

### Request
A request allows to set a given parameter of the rocks to be randomized. As of now, we only randomize so called `xformOp`s. These are a set of geometric transformation that control the scale, rotation, and translation of assets in OpenUSD. A request applies a statistical distribution, like a `uniform` distribution on a geometric primitive, or a digital elevation map. The axes on which the randomization should be applied can also be selected.

Parameters:
- `attribute`: `(str)`, The name of the attribute to be randomized. Can be `Scale`, `Orientation`, or `Position`.
- `axes`: `(list)`, The axis of the selected attribute to be randomized. A list containing `x`, `y`, `z`, `w`. Note that the orientation is always defined as a quaternion. Aggregating multiple dimmensions i.e. writing `xyz` means that the values for `x` `y` and `z` will be the same.
- `num`: `(int)`, The number of points to be generated. If there is a PointProcess, or a ClusterPointProcess inside the same [request group](#request-group) this parameter should not be set. If there is none, it should be similar for all requests within the same [request group](#request-group).
- `inherit_parents`: `(bool)`, If the distribution has a parent distribution. This is only applicable to "ClusterPointProcesses" kind of distributions.
- `layer`: `str`, the layer object to sample on. More info is given [here](#request-layers).
- `sampler`: `str`, the sampler object to sample from. More info is given [here](#request-samplers)

Examples:

Single request, that scales the x y an z dimensions similarly. The values range from 0.01 to 0.05 and the distribution used is a uniform distribution.
```yaml
req_scale:
  attribute: Scale
  axes: ["xyz"]
  num: 1000
  inherit_parents: False
  layer:
    name: Line
      xmin: 0.01
      xmax: 0.05
  sampler:
    name: Uniform
    randomization_space: 1
    seed: 42
```

Single request where the orientation is sampled for a given axis. Here the yaw is randomized.
```yaml
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
    seed: 42
```

### Request Group

A request group defines a bundle of requests to execute, the type of assets to be used, and whether of not we should use an instancer. 

Parameters:
- `seed`: `(int)`, the seed to be used.
- `collections`: `(str)`, the name of the folder in which the rocks are saved.
- `use_point_instancer`: `(bool)`, if the assets should be generated with an instancer or not. <span style="color: red;">**Limitations of instancers in isaac sim**:</span> they do not work well with semantic information. Hence, if you want to collect semantic data, do not use them!
- `parent`: `(str)`, to be set if another [request group](#request-group) if the parent of this one. Set to the name of that other [request group](#request-group).
- `requests`: `list[request]`, the list of requests to be executed.

Example:

A request group without parent.
```yaml
medium_rocks:
      seed: ${....seed}
      collections: ["apollo_rocks"] # Uses the apollo rocks inside assets/USD_Assets/rocks
      use_point_instancer: True # Set to True for fast sampling, set to False for SDG
      requests:
        req_pos_xy: # Sample XY position
          attribute: Position
          axes: ["x", "y"]
          layer:
            name: Image
            mpp_resolution: ${.......lunaryard_settings.resolution}
            output_space: 2
          sampler:
            name: ThomasCluster
            randomization_space: 2
            lambda_parent: 0.25
            lambda_daughter: 200
            sigma: 0.2
            seed: ${.......seed}

        req_pos_z: # Get Z from the elevation map.
          attribute: Position
          axes: ["z"]
          layer:
            name: Image
            output_space: 1
          sampler:
            name: Image
            randomization_space: 1
            mpp_resolution: ${.......lunaryard_settings.resolution}

        req_random_z_rot: # Randomize Z orientation
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

        req_scale: # Randomize the scale homogeneously.
          attribute: Scale
          axes: ["xyz"]
          layer:
            name: Line
            xmin: 1.0
            xmax: 1.0
          sampler:
            name: Uniform
            randomization_space: 1
            seed: ${.......seed}
```


### Rock Generation

This aggregates multiple requests groups.

Parameters:
- `instancers_path`: `(str)`, The path where the instancers will be generated.
- `rocks_settings`: `(list(request_group))`, A list of request groups.

Example:
```yaml
rocks_settings:
  instancers_path: /Lunaryard/Rocks
  rocks_settings:
    medium_rocks:
      seed: ${....seed}
      collections: ["apollo_rocks"]
      use_point_instancer: True
      requests:
        req_pos_xy:
          attribute: Position
          axes: ["x", "y"]
          layer:
            name: Image
            mpp_resolution: ${.......lunaryard_settings.resolution}
            output_space: 2
          sampler:
            name: ThomasCluster
            randomization_space: 2
            lambda_parent: 0.25
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
            xmax: 1.0
          sampler:
            name: Uniform
            randomization_space: 1
            seed: ${.......seed}

    large_rocks:
      seed: ${....seed}
      collections: ["apollo_rocks"]
      use_point_instancer: True 
      parent: medium_rocks
      requests:
        req_pos_xy:
          attribute: Position
          axes: ["x", "y"]
          layer:
            name: Image
            mpp_resolution: ${.......lunaryard_settings.resolution}
            output_space: 2
          sampler:
            name: ThomasCluster
            randomization_space: 2
            lambda_parent: 0.25
            lambda_daughter: 5
            sigma: 0.05
            inherit_parents: True
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
            xmin: 2.0
            xmax: 5.0
          sampler:
            name: Uniform
            randomization_space: 1
            seed: ${.......seed}

    small_rocks:
      seed: ${....seed}
      collections: ["apollo_rocks"]
      use_point_instancer: True
      parent: medium_rocks
      requests:
        req_pos_xy:
          attribute: Position
          axes: ["x", "y"]
          layer:
            name: Image
            mpp_resolution: ${.......lunaryard_settings.resolution}
            output_space: 2
          sampler:
            name: ThomasCluster
            randomization_space: 2
            lambda_parent: 0.5
            lambda_daughter: 500
            sigma: 0.3
            inherit_parents: True
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
            xmin: 0.01
            xmax: 0.05
          sampler:
            name: Uniform
            randomization_space: 1
            seed: ${.......seed}
```
