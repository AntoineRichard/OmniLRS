## Deform terrain

Run simulation.
```
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh run.py mode=ROS1 environment=lunaryard_deformable_10m rendering=ray_tracing
```
Then, publish flag to enable deformation.

```
rostopic pub -r 1 /Lunalab/Terrain/Deform std_msgs/Bool "data: true"
```