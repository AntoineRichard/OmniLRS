# Environments ROS 1 & 2 topics

Our simulation provides an interface to alter the simulation's state through ROS topics. This will change in the future as it is likely that we will shift towards a ZeroMQ-based stack with a webserver so that more simulation parameters can be tuned.

> [!NOTE]
> For most of these operations we should be using services rather than topics. We are fully aware of this, but Omniverse does not allow to easily compile custom services outside of so-called "extensions". This partially explain the planned move to ZeroMQ. We may also release an extension to address some of these shortcomings.

## Common
Rendering topics:
- `/OmniLRS/Render/EnableRTXRealTime`: `(std_msgs/Empty)`, Switches the renderer to RTXRealTime.
- `/OmniLRS/Render/EnableRTXInteractive`: `(std_msgs/Empty)`, Switches the renderer to RTXInteractive.
- `/OmniLRS/LensFlare/EnableLensFlares`: `(std_msgs/Bool)`, Enables or disables the lens flares. Setting it to true will enable them.
- `/OmniLRS/LensFlare/NumBlades`: `(std_msgs/Int8)`, The number of blades used when simulating the lens flares. Does not affect the rest of the rendering.
- `/OmniLRS/LensFlare/Scale`: `(std_msgs/Float32)`, The scale of the lens flare. Does not affect the rest of the rendering.
- `/OmniLRS/LensFlare/ApertureRotation`: `(std_msgs/Float32)`, The aperture rotation of the lens flare. It can be used to rotate the lens flare. Does not affect the rest of the rendering.
- `/OmniLRS/LensFlare/FocalLength`: `(std_msgs/Float32)`, The focal length used to simulate the lens flare. Does not affect the rest of the rendering.
- `/OmniLRS/LensFlare/Fstop`: `(std_msgs/Float32)`, The fstop setting of the camera used when simulating the lens flare. Does not affect the rest of the rendering.
- `/OmniLRS/LensFlare/SensorAspectRatio`: `(std_msgs/Float32)`, The sensor aspect ratio used to simulate the lens flare. Does not affect the rest of the rendering.
- `/OmniLRS/LensFlare/SensorDiagonal`: `(std_msgs/Float32)`, The sensor diagonal used to simulate the lens flare. Does not affect the rest of the rendering

Robot topics:
- `/OmniLRS/Robots/Spawn`: `(geometry_msgs/PoseStamped)`, A repurposed message to spawn a given robot at a given position. The `frame_id` field should be used like so: `robot_name:usd_path`. Where the robot_name is the name to be given to the robot in the simulation, and usd_path is the path to the usd file on the disk. The `position` and `orientation` field are used to set where the robot should be spawned in the global simulation frame.
- `/OmniLRS/Robots/Teleport`: `(geometry_msgs/PoseStamped)`, A repurposed message to teleport a given robot at a given position. The `frame_id` field should contain the name of the robot to be teleported. The `position` and `orientation` field are used to set where the robot should be teleported in the global simulation frame.
- `/OmniLRS/Robots/Reset`: `(std_msgs/String)`, Resets the robot whose name is in the string. A reset puts the robot back where it was when it was first spawned.
- `/OmniLRS/Robots/ResetAll`: `(std_msgs/Empty)`, Resets all the robots in the scene. A reset puts the robot back to where it was when it was first spawned.

> [!NOTE]
> Large scale environments do not support multiple robots.

## Lunalab 
Topics offered by the lunalab in addition to the ones in [common](#common).
- `/OmniLRS/Projector/TurnOn`: `(std_msgs/Bool)`, Turns on or off the projector in the lunalab. Setting it to true turns the projector on.
- `/OmniLRS/Projector/Intensity`: `(std_msgs/Float32)`, Set the intensity of the projector. This value is set as a percentage of the default value. Setting the value to 50.0 will set the projector's output to 50%.
- `/OmniLRS/Projector/Radius`: `(std_msgs/Float32)`, Sets the radius of the projector in the lunalab. Radius in meters.
- `/OmniLRS/Projector/Pose`: `(geometry_msgs/Pose)`, Sets the pose of the projector. We'd recommend to stay between the following boundaries: `[0.5<x<6.0,y=0,0.25<z<2.0]`
- `/OmniLRS/CeilingLights/TurnOn`: `(std_msgs/Bool)`, Turns on or off the ceiling lights. True turns the ceiling lights on.
- `/OmniLRS/CeilingLights/Intensity`: `(std_msgs/Float32)`, Intensity of the ceiling lights. Arbitrary units.
- `/OmniLRS/OmniLRS/Curtains/Extend`: `(std_msgs/Bool)`, Extends the curtains in the lab. True extends the curtains, false folds them.
- `/OmniLRS/Terrain/Switch`: `(std_msgs/Int32)`, Changes the terrain. If the value is positive the terrain will be taken out of the folder given to the terrain_manager. If the value is negative, a random procedural terrain is generated. If the value is positive, and exceeds the number of terrains availabe this will result in an error.
- `/OmniLRS/Terrain/EnableRocks`: `(std_msgs/Bool)`, Displays rocks in the scene. Set to true to display.
- `/OmniLRS/Terrain/RandomizeRocks`: `(std_msgs/Int32)`, Randomizes rocks. The value tells the samplers how many rocks should be generated. If you are using point-processes, the value will be discarded, and the samplers will rely on the densities that were given to them.

### Lunaryard
Topics offered by the lunaryard in addition to the ones in [common](#common).
- `/OmniLRS/Sun/Intensity`: `(std_msgs/Float32)`, The intensity of the sun. Arbitrary units.
- `/OmniLRS/Sun/Pose`: `(geometry_msgs/Pose)`, The pose of the sun. Will only use the quaternions to get the elevation and azimuth.
- `/OmniLRS/Sun/Color`: `(std_msgs/ColorRGBA)`, The color of the sun, in the 0,1 range.
- `/OmniLRS/Sun/ColorTemperature`: `(std_msgs/Float32)`, The color temperature of the sun in Kelvin.
- `/OmniLRS/Sun/AngularSize`: `(std_msgs/Float32)`, The size of the sun as an angular value in degrees. Usually 0.53 degrees.
- `/OmniLRS/Terrain/Switch`: `(std_msgs/Int32)`, Changes the terrain. If the value is positive the terrain will be taken out of the folder given to the terrain_manager. If the value is negative, a random procedural terrain is generated. If the value is positive, and exceeds the number of terrains availabe this will result in an error.
- `/OmniLRS/Terrain/EnableRocks`: `(std_msgs/Bool)`, Displays rocks in the scene. Set to true to display.
- `/OmniLRS/Terrain/RandomizeRocks`: `(std_msgs/Int32)`, Randomizes rocks. The value tells the samplers how many rocks should be generated. If you are using point-processes, the value will be discarded, and the samplers will rely on the densities that were given to them.

### LargeScale
Topics offered by the large scale environment in addition to the ones in [common](#common).
- `/OmniLRS/Sun/Intensity`: `(std_msgs/Float32)`, The intensity of the sun. Arbitrary units.
- `/OmniLRS/Sun/Pose`: `(geometry_msgs/Pose)`, The pose of the sun. Will only use the quaternions to get the elevation and azimuth.
- `/OmniLRS/Sun/Color`: `(std_msgs/ColorRGBA)`, The color of the sun, in the 0,1 range.
- `/OmniLRS/Sun/ColorTemperature`: `(std_msgs/Float32)`, The color temperature of the sun in Kelvin.
- `/OmniLRS/Sun/AngularSize`: `(std_msgs/Float32)`, The size of the sun as an angular value in degrees. Usually 0.53 degrees.

Examples:
```bash
# Turn off the projector
ros2 topic pub --once /OmniLRS/Projector/TurnOn std_msgs/msg/Bool data:\ 0
# Randomize 12 rocks
ros2 topic pub --once /OmniLRS/Terrain/RandomizeRocks std_msgs/msg/Int32 data:\ 12
# Enables rocks
ros2 topic pub --once /OmniLRS/Terrain/EnableRocks std_msgs/msg/Bool data:\ 1
# Change terrain
ros2 topic pub --once /OmniLRS/Terrain/Switch std_msgs/msg/Int32 data:\ -1
# Changes the sun's color to purple
ros2 topic pub --once /OmniLRS/Sun/Color std_msgs/msg/ColorRGBA "{'r':1.0, 'g':0.0, 'b':1.0, 'a':0.0}"
# Reset all the robots
ros2 topic pub /OmniLRS/Robots/ResetAll std_msgs/msg/Empty 
# Teleports the husky
ros2 topic pub --once /OmniLRS/Robots/Teleport geometry_msgs/msg/PoseStamped "{'header':{'stamp':{'sec':0.0,'nanosec':0.0},'frame_id':'/husky'},pose:{'position':{'x':'10.0','y':5.0,'z': 1.0},'orientation':{'x':0.0,'y':0.0,'z':0.0,'w':1.0}}}"
# Spawns a husky
ros2 topic pub --once /OmniLRS/Robots/Spawn geometry_msgs/msg/PoseStamped "{'header':{'stamp':{'sec':0.0,'nanosec':0.0},'frame_id':'husky2:/home/antoine/Documents/Lunalab/assets/USD_Assets/robots/husky_vlp16.usd'},pose:{'position':{'x':'10.0','y':10.0,'z': 1.0},'orientation':{'x':0.0,'y':0.0,'z':0.0,'w':1.0}}}"
```
