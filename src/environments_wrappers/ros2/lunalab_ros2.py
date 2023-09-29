# System imports
from threading import Thread
import numpy as np
import time
import sys
import os

from omni.isaac.core import World
from pxr import Gf
import omni

# Custom libs
from src.environments.lunalab import LabController
from src.robots.robot import RobotManager

# Loads ROS2 dependent libraries
from std_msgs.msg import Bool, Float32, ColorRGBA, Int8, Int32, String, Empty
from rclpy.executors import SingleThreadedExecutor as Executor
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
import rclpy

class ROS_LabManager(Node):
    """
    ROS2 node that manages the lab environment"""

    def __init__(self, environment_cfg, flares_cfg) -> None:
        super().__init__("Lab_controller_node")
        print(environment_cfg)
        self.LC = LabController(**environment_cfg, flares_settings=flares_cfg)
        self.LC.load()
        self.trigger_reset = False

        self.create_subscription(Bool, "/Lunalab/Projector/TurnOn", self.setProjectorOn, 1)
        self.create_subscription(Float32, "/Lunalab/Projector/Intensity", self.setProjectorIntensity, 1)
        self.create_subscription(Float32, "/Lunalab/Projector/Radius", self.setProjectorRadius, 1)
        self.create_subscription(Pose, "/Lunalab/Projector/Pose", self.setProjectorPose, 1)
        #self.create_subscription(ColorRGBA, "/Lunalab/Projector/Color", self.setProjectorColor, 1)
        self.create_subscription(Bool, "/Lunalab/CeilingLights/TurnOn", self.setCeilingOn, 1)
        self.create_subscription(Float32, "/Lunalab/CeilingLights/Intensity", self.setCeilingIntensity, 1)
        #self.create_subscription(Float32, "/Lunalab/CeilingLights/Radius", self.setCeilingRadius, 1)
        #self.create_subscription(Float32, "/Lunalab/CeilingLights/FOV", self.setCeilingFOV, 1)
        #self.create_subscription(ColorRGBA, "/Lunalab/CeilingLights/Color", self.setCeilingColor, 1)
        self.create_subscription(Bool, "/Lunalab/Curtains/Extend", self.setCurtainsMode, 1)
        self.create_subscription(Int32, "/Lunalab/Terrain/Switch", self.switchTerrain, 1)
        self.create_subscription(Bool, "/Lunalab/Terrain/EnableRocks", self.enableRocks, 1)
        self.create_subscription(Int32, "/Lunalab/Terrain/RandomizeRocks", self.randomizeRocks, 1)
        self.create_subscription(Empty, "/Lunalab/Render/EnableRTXRealTime", self.useRTXRealTimeRender, 1)
        self.create_subscription(Empty, "/Lunalab/Render/EnableRTXInteractive", self.useRTXInteractiveRender, 1)
        self.create_subscription(Bool, "/Lunalab/LensFlare/EnableLensFlares", self.setLensFlareOn, 1)
        self.create_subscription(Int8, "/Lunalab/LensFlare/NumBlades", self.setLensFlareNumBlade, 1)
        self.create_subscription(Float32, "/Lunalab/LensFlare/Scale", self.setLensFlareScale, 1)
        self.create_subscription(Float32, "/Lunalab/LensFlare/ApertureRotation", self.setLensFlareApertureRotation, 1)
        self.create_subscription(Float32, "/Lunalab/LensFlare/FocalLength", self.setLensFlareFocalLength, 1)
        self.create_subscription(Float32, "/Lunalab/LensFlare/Fstop", self.setLensFlareFstop, 1)
        self.create_subscription(Float32, "/Lunalab/LensFlare/SensorAspectRatio", self.setLensFlareSensorAspectRatio, 1)
        self.create_subscription(Float32, "/Lunalab/LensFlare/SensorDiagonal", self.setLensFlareSensorDiagonal, 1)

        self.modifications = []

    def clearModifications(self) -> None:
        """
        Clears the list of modifications to be applied to the lab."""

        self.modifications = []

    def applyModifications(self) -> None:
        """
        Applies the list of modifications to the lab."""

        for mod in self.modifications:
            mod[0](mod[1])
        self.clearModifications()

    def reset(self) -> None:
        """
        Resets the lab to its initial state."""

        pass

    def setProjectorOn(self, data:Bool) -> None:
        """
        Turns the projector on or off.
        
        Args:
            data (Bool): True to turn the projector on, False to turn it off."""
        
        self.modifications.append([self.LC.turnProjectorOnOff, data.data])

    def setProjectorIntensity(self, data:Float32) -> None:
        """
        Sets the projector intensity.
        
        Args:
            data (Float32): Intensity in percentage."""
        
        default_intensity = 300000000.0
        data = default_intensity*float(data.data)/100.0
        self.modifications.append([self.LC.setProjectorIntensity, data])

    def setProjectorRadius(self, data: Float32) -> None:
        """
        Sets the projector radius.
        
        Args:
            data (Float32): Radius in meters."""
        
        self.modifications.append([self.LC.setProjectorRadius, data.data])

    def setProjectorColor(self, data:ColorRGBA) -> None:
        """
        Sets the projector color.
        
        Args:
            data (ColorRGBA): Color in RGBA format."""
        
        color = [data.r, data.g, data.b]
        self.modifications.append([self.LC.setProjectorColor, color])

    def setProjectorPose(self, data:Pose) -> None:
        """
        Sets the projector pose.
        
        Args:
            data (Pose): Pose in ROS2 Pose format."""
        
        position = [data.position.x, data.position.y, data.position.z]
        quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.modifications.append([self.LC.setProjectorPose, (position, quaternion)])

    def setCeilingOn(self, data:Bool) -> None:
        """
        Turns the ceiling lights on or off.
        
        Args:
            data (Bool): True to turn the lights on, False to turn them off."""
        
        self.modifications.append([self.LC.turnRoomLightsOnOff, data.data])

    def setCeilingIntensity(self, data:Float32) -> None:
        """
        Sets the ceiling lights intensity.
        
        Args:
            data (Float32): Intensity in percentage."""
        
        self.modifications.append([self.LC.setRoomLightsIntensity, data.data])

    def setCeilingRadius(self, data:Float32) -> None:
        """
        Sets the ceiling lights radius.
        
        Args:
            data (Float32): Radius in meters."""
        
        self.modifications.append([self.LC.setRoomLightsRadius, data.data])

    def setCeilingFOV(self, data:Float32) -> None:
        """
        Sets the ceiling lights field of view.
        
        Args:
            data (Float32): Field of view in degrees."""
        
        self.modifications.append([self.LC.setRoomLightsFOV, data.data])

    def setCeilingColor(self, data:ColorRGBA) -> None:
        """
        Sets the ceiling lights color.
        
        Args:
            data (ColorRGBA): Color in RGBA format."""
        
        color = [data.r, data.g, data.b]
        self.modifications.append([self.LC.setRoomLightsColor, color])

    def setCurtainsMode(self, data:Bool) -> None:
        """
        Sets the curtains mode.
        
        Args:
            data (Bool): True to extend the curtains, False to retract them."""
        
        self.modifications.append([self.LC.curtainsExtend, data.data])

    def switchTerrain(self, data:Int32) -> None:
        """
        Switches the terrain.
        
        Args:
            data (Int32): 0 for the first terrain, 1 for the second terrain."""
        
        self.modifications.append([self.LC.switchTerrain, data.data])
        self.trigger_reset = True

    def enableRocks(self, data:Bool) -> None:
        """
        Enables or disables the rocks.
        
        Args:
            data (Bool): True to enable the rocks, False to disable them."""
        
        self.modifications.append([self.LC.enableRocks, data.data])
        self.trigger_reset = True

    def randomizeRocks(self, data:Int32) -> None:
        """
        Randomizes the rocks.
        
        Args:
            data (Int32): Number of rocks to randomize."""

        int(data.data)
        self.modifications.append([self.LC.randomizeRocks, data.data])
        self.trigger_reset = True

    def placeRocks(self, data:str) -> None:
        """
        Places the rocks.
        
        Args:
            data (str): Path to the file containing the rocks positions."""
        
        self.modifications.append([self.LC.placeRocks, data.data])
        self.trigger_reset = True
    
    def useRTXRealTimeRender(self, data:Empty) -> None:
        """
        Enables or disables RTX real time rendering.
        
        Args:
            data (Empty): Empty message."""
        
        self.modifications.append([self.LC.enableRTXRealTime, 0])

    def useRTXInteractiveRender(self, data:Empty) -> None:
        """
        Enables or disables RTX interactive rendering.
        
        Args:
            data (Empty): Empty message."""
        
        self.modifications.append([self.LC.enableRTXInteractive, 0])

    def setLensFlareOn(self, data:Bool) -> None:
        """
        Enables or disables lens flares.
        
        Args:
            data (Bool): True to enable lens flares, False to disable them."""
        
        self.modifications.append([self.LC.enableLensFlare, data.data])

    def setLensFlareNumBlade(self, data:Int8) -> None:
        """
        Sets the number of blades of the lens flares.
        
        Args:
            data (Int8): Number of blades."""
        
        data = int(data.data)
        self.modifications.append([self.LC.setFlareNumBlades, data])

    def setLensFlareScale(self, data:Float32) -> None:
        """
        Sets the scale of the lens flares.
        
        Args:
            data (Float32): Scale."""
        
        data = float(data.data)
        self.modifications.append([self.LC.setFlareScale, data])

    def setLensFlareFstop(self, data:Float32) -> None:
        """
        Sets the fstop of the lens flares.
        
        Args:
            data (Float32): Fstop."""

        data = float(data.data)
        self.modifications.append([self.LC.setFlareFstop, data])

    def setLensFlareFocalLength(self, data:Float32) -> None:
        """
        Sets the focal length of the lens flares.
        
        Args:
            data (Float32): Focal length."""

        data = float(data.data)
        self.modifications.append([self.LC.setFlareFocalLength, data])

    def setLensFlareSensorAspectRatio(self, data:Float32) -> None:
        """
        Sets the sensor aspect ratio of the lens flares.
        
        Args:
            data (Float32): Sensor aspect ratio."""
        
        data = float(data.data)
        self.modifications.append([self.LC.setFlareSensorAspectRatio, data])

    def setLensFlareSensorDiagonal(self, data:Float32) -> None:
        """
        Sets the sensor diagonal of the lens flares.
        
        Args:
            data (Float32): Sensor diagonal."""
        
        data = float(data.data)
        self.modifications.append([self.LC.setFlareSensorDiagonal, data])

    def setLensFlareApertureRotation(self, data:Float32) -> None:
        """
        Sets the aperture rotation of the lens flares.
        
        Args:
            data (Float32): Aperture rotation."""
        
        data = float(data.data)
        self.modifications.append([self.LC.setFlareApertureRotation, data])

    def cleanScene(self):
        """
        Cleans the scene."""

        self.destroy_node()

class ROS_RobotManager(Node):
    """
    ROS2 node that manages the robots."""

    def __init__(self) -> None:
        super().__init__("Robot_spawn_manager_node")
        spawning_pose_list = [{"position":Gf.Vec3d(1.5,-0.5,0.25),"orientation":Gf.Quatd(0.707, Gf.Vec3d(0,0,0.707))},
                              {"position":Gf.Vec3d(2.5,-0.5,0.25),"orientation":Gf.Quatd(0.707, Gf.Vec3d(0,0,0.707))},
                              {"position":Gf.Vec3d(3.5,-0.5,0.25),"orientation":Gf.Quatd(0.707, Gf.Vec3d(0,0,0.707))},
                              {"position":Gf.Vec3d(4.5,-0.5,0.25),"orientation":Gf.Quatd(0.707, Gf.Vec3d(0,0,0.707))},
                              {"position":Gf.Vec3d(5.5,-0.5,0.25),"orientation":Gf.Quatd(0.707, Gf.Vec3d(0,0,0.707))}]
        self.RM = RobotManager(spawning_pose_list, is_ROS2=True, max_robots=len(spawning_pose_list), robots_root="/Robots")

        self.create_subscription(String, "/Lunalab/Robots/Spawn", self.spawnRobot, 1)
        self.create_subscription(PoseStamped, "/Lunalab/Robots/Teleport", self.teleportRobot, 1)
        self.create_subscription(String, "/Lunalab/Robots/Reset", self.resetRobot, 1)
        self.create_subscription(String, "/Lunalab/Robots/ResetAll", self.resetRobots, 1)

        self.domain_id = 0

        self.modifications = []

    def clearModifications(self) -> None:
        """
        Clears the list of modifications to be applied to the robots."""

        self.modifications = []

    def applyModifications(self) -> None:
        """
        Applies the list of modifications to the robots."""

        for mod in self.modifications:
            mod[0](*mod[1])
        self.clearModifications()

    def reset(self) -> None:
        """
        Resets the robots to their initial state."""

        self.clearModifications()
        self.resetRobots(0)

    def spawnRobot(self, data:String) -> None:
        """
        Spawns a robot.
        
        Args:
            data (String): Name and path of the robot to spawn.
                           Must be in the format: robot_name:usd_path"""

        assert len(data.data.split(":")) == 2, "The data should be in the format: robot_name:usd_path"
        robot_name = data.data.split(":")[0]
        usd_path = data.data.split(":")[1]
        self.modifications.append([self.RM.addRobot, [usd_path, robot_name, self.domain_id]])

    def teleportRobot(self, data:PoseStamped) -> None:
        """
        Teleports a robot.
        
        Args:
            data (Pose): Pose in ROS2 Pose format."""
        
        robot_name = data.header.frame_id
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        self.modifications.append([self.RM.teleportRobot, [robot_name, p, q]])

    def resetRobot(self, data:String) -> None:
        """
        Resets a robot.
        
        Args:
            data (String): Name of the robot to reset."""
        
        robot_name = data.data
        self.modifications.append([self.RM.resetRobot, [robot_name]])

    def resetRobots(self, data:String):
        """
        Resets all the robots.

        Args:
            data (Int32): Dummy argument."""
        
        self.modifications.append([self.RM.resetRobots, []])

    def cleanRobots(self):
        """
        Cleans the robots."""

        self.destroy_node()

class SimulationManager:
    """"
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the lab manager thread
    - Running the robot manager thread
    - Running the simulation
    - Cleaning the simulation"""

    def __init__(self, cfg, simulation_app) -> None:
        self.simulation_app = simulation_app
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        self.physics_ctx = self.world.get_physics_context()
        self.physics_ctx.set_solver_type("PGS")
        # Lab manager thread
        self.ROSLabManager = ROS_LabManager(cfg["environment"], cfg["rendering"]["lens_flares"])
        exec1 = Executor()
        exec1.add_node(self.ROSLabManager)
        self.exec1_thread = Thread(target=exec1.spin, daemon=True, args=())
        self.exec1_thread.start()
        # Robot manager thread
        self.ROSRobotManager = ROS_RobotManager()
        exec2 = Executor()
        exec2.add_node(self.ROSRobotManager)
        self.exec2_thread = Thread(target=exec2.spin, daemon=True, args=())
        self.exec2_thread.start()
        self.world.reset()
        
    def run_simulation(self) -> None:
        """
        Runs the simulation."""

        self.timeline.play()
        while self.simulation_app.is_running():
            self.world.step(render=True)
            if self.world.is_playing():
                # Apply modifications to the lab only once the simulation step is finished
                if self.world.current_time_step_index == 0:
                    self.world.reset()
                    self.ROSLabManager.reset()
                    self.ROSRobotManager.reset()
                self.ROSLabManager.applyModifications()
                if self.ROSLabManager.trigger_reset:
                    self.ROSRobotManager.reset()
                    self.ROSLabManager.trigger_reset = False
                self.ROSRobotManager.applyModifications()

        self.timeline.stop()

if __name__ == "__main__":
    rclpy.init()
    SM = SimulationManager()
    SM.run_simulation()
