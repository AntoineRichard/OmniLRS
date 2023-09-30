__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

# System imports
from threading import Thread
import numpy as np
import time
import sys
import os

from omni.isaac.core import World
from pxr import Gf

# Custom libs
from src.environments.lunaryard import LunaryardController
from src.configurations.rendering_confs import FlaresConf

# Loads ROS2 dependent libraries
from std_msgs.msg import Bool, Float32, ColorRGBA, Int8, Int32, String, Empty
from rclpy.executors import SingleThreadedExecutor as Executor
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
import rclpy

class ROS_LunaryardManager(Node):
    """
    ROS2 node that manages the lab environment"""

    def __init__(self, environment_cfg: dict = None,
                       flares_cfg: FlaresConf = None,
                       **kwargs,
                       ) -> None:
        """
        Initializes the lab manager.
        
        Args:
            environment_cfg (dict): Environment configuration.
            flares_cfg (dict): Flares configuration.
            **kwargs: Additional arguments."""

        super().__init__("Lab_controller_node")
        self.LC = LunaryardController(**environment_cfg, flares_settings=flares_cfg)
        self.LC.load()
        self.trigger_reset = False

        self.create_subscription(Float32, "/LunarYard/Sun/Intensity", self.setSunIntensity, 1)
        self.create_subscription(Pose, "/LunarYard/Sun/Pose", self.setSunPose, 1)
        self.create_subscription(ColorRGBA, "/LunarYard/Sun/Color", self.setSunColor, 1)
        self.create_subscription(Int32, "/LunarYard/Terrain/Switch", self.switchTerrain, 1)
        self.create_subscription(Bool, "/LunarYard/Terrain/EnableRocks", self.enableRocks, 1)
        self.create_subscription(Int32, "/LunarYard/Terrain/RandomizeRocks", self.randomizeRocks, 1)
        self.create_subscription(Empty, "/LunarYard/Render/EnableRTXRealTime", self.useRTXRealTimeRender, 1)
        self.create_subscription(Empty, "/LunarYard/Render/EnableRTXInteractive", self.useRTXInteractiveRender, 1)
        self.create_subscription(Bool, "/LunarYard/LensFlare/EnableLensFlares", self.setLensFlareOn, 1)
        self.create_subscription(Int8, "/LunarYard/LensFlare/NumBlades", self.setLensFlareNumBlade, 1)
        self.create_subscription(Float32, "/LunarYard/LensFlare/Scale", self.setLensFlareScale, 1)
        self.create_subscription(Float32, "/LunarYard/LensFlare/ApertureRotation", self.setLensFlareApertureRotation, 1)
        self.create_subscription(Float32, "/LunarYard/LensFlare/FocalLength", self.setLensFlareFocalLength, 1)
        self.create_subscription(Float32, "/LunarYard/LensFlare/Fstop", self.setLensFlareFstop, 1)
        self.create_subscription(Float32, "/LunarYard/LensFlare/SensorAspectRatio", self.setLensFlareSensorAspectRatio, 1)
        self.create_subscription(Float32, "/LunarYard/LensFlare/SensorDiagonal", self.setLensFlareSensorDiagonal, 1)

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

    def setSunIntensity(self, data:Float32) -> None:
        """
        Sets the projector intensity.
        
        Args:
            data (Float32): Intensity in percentage."""
        
        self.modifications.append([self.LC.setSunIntensity, data.data])

    def setSunColor(self, data:ColorRGBA) -> None:
        """
        Sets the projector color.
        
        Args:
            data (ColorRGBA): Color in RGBA format."""
        
        color = [data.r, data.g, data.b]
        self.modifications.append([self.LC.setSunColor, color])

    def setSunPose(self, data:Pose) -> None:
        """
        Sets the projector pose.
        
        Args:
            data (Pose): Pose in ROS2 Pose format."""
        
        position = [data.position.x, data.position.y, data.position.z]
        quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.modifications.append([self.LC.setSunPose, (position, quaternion)])

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