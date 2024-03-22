## Deform terrain

### 0. Run IsaacSim.

*lunalab*
```bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh run.py mode=ROS1 environment=lunalab_deformable rendering=ray_tracing
```

*lunaryard*
```bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh run.py mode=ROS1 environment=lunaryard_deformable_10m rendering=ray_tracing
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