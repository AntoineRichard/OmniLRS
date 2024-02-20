## Deform terrain

### 0. Run simulation Isaac first.

*lunalab*
```bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh run.py mode=ROS1 environment=lunalab_deformable rendering=ray_tracing
```

*lunaryard*
```bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh run.py mode=ROS1 environment=lunaryard_deformable_10m rendering=ray_tracing
```

Now deformation happens as a physics callback (tehcnically, 1/4 of physics fps), so you don't need to publish topic to activate deformation.


### 1. Then, launch controller (such as teleop_twist_keyboard or teleop_twist_joy.)

~~keyboard teleoperation~~
\
I removed keyboard teleoperation since using twist for control is not a correct way for EX1. 
```bash
noetic
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

joystick teleoperation, 
```bash
noetic
cd /home/lunar5/jnskkmhr/catkin_ws && source devel/setup.bash
roslaunch omni_ex1_joy ex1_ackermann_teleop.launch
```