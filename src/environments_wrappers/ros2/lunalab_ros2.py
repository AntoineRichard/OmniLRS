__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

# Custom libs
from src.environments.lunalab import LunalabController
from src.configurations.rendering_confs import FlaresConf

# Loads ROS2 dependent libraries
from std_msgs.msg import Bool, Float32, ColorRGBA, Int8, Int32, String, Empty
from rclpy.executors import SingleThreadedExecutor as Executor
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
import rclpy

class ROS_LunalabManager(Node):
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
        self.LC = LunalabController(**environment_cfg, flares_settings=flares_cfg)
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