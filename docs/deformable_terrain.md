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


### 1. Then, launch controller (such as teleop_twist_keyboard or teleop_twist_joy.)
keyboard teleoperation
```bash
noetic
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
or joystick teleoperation, 
```bash
noetic
cd /home/lunar5/jnskkmhr/catkin_ws && source devel/setup.bash
roslaunch omni_ex1_joy ex1_teleop_joy.launch
```

### 2. Finally, publish flag to enable deformation.

```bash
noetic
rostopic pub -r 6 /Lunalab/Terrain/Deform std_msgs/Bool "data: true"
```
In the future, I will run this in physics callback (technically, run at 6Hz = 1/10 step of physics.)