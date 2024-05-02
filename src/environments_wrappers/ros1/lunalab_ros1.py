__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

# Custom libs
from src.environments.lunalab import LunalabController
from src.configurations.rendering_confs import FlaresConf
from src.robots.robot import RobotManager

# Loads ROS1 dependent libraries
import rospy
from std_msgs.msg import Bool, Float32, ColorRGBA, Int8, Int32, String, Empty
from geometry_msgs.msg import Pose, PoseStamped


class ROS_LunalabManager:
    """
    ROS1 node that manages the Lunalab and robots."""

    def __init__(
        self,
        environment_cfg: dict = None,
        flares_cfg: FlaresConf = None,
        **kwargs,
    ) -> None:
        """
        Initializes the ROS1 node.

        Args:
            environment_cfg (dict): Environment configuration dictionary.
            flares_cfg (dict): Lens flares configuration dictionary.
            **kwargs: Additional keyword arguments."""

        self.LC = LunalabController(**environment_cfg, flares_settings=flares_cfg)
        self.RM = RobotManager(
            environment_cfg["robots_settings"]
        )
        self.LC.load()

        self.projector_subs = []
        self.projector_subs.append(
            rospy.Subscriber(
                "/Lunalab/Projector/TurnOn", Bool, self.setProjectorOn, queue_size=1
            )
        )
        self.projector_subs.append(
            rospy.Subscriber(
                "/Lunalab/Projector/Intensity",
                Float32,
                self.setProjectorIntensity,
                queue_size=1,
            )
        )
        self.projector_subs.append(
            rospy.Subscriber(
                "/Lunalab/Projector/Radius",
                Float32,
                self.setProjectorRadius,
                queue_size=1,
            )
        )
        self.projector_subs.append(
            rospy.Subscriber(
                "/Lunalab/Projector/Pose", Pose, self.setProjectorPose, queue_size=1
            )
        )
        # self.projector_subs.append(rospy.Subscriber("/Lunalab/Projector/Color", ColorRGBA, self.setProjectorColor, queue_size=1))
        self.ceiling_subs = []
        self.ceiling_subs.append(
            rospy.Subscriber(
                "/Lunalab/CeilingLights/TurnOn", Bool, self.setCeilingOn, queue_size=1
            )
        )
        self.ceiling_subs.append(
            rospy.Subscriber(
                "/Lunalab/CeilingLights/Intensity",
                Float32,
                self.setCeilingIntensity,
                queue_size=1,
            )
        )
        # self.ceiling_subs.append(rospy.Subscriber("/Lunalab/CeilingLights/Radius", Float32, self.setCeilingRadius, queue_size=1))
        # self.ceiling_subs.append(rospy.Subscriber("/Lunalab/CeilingLights/FOV", Float32, self.setCeilingFOV, queue_size=1))
        # self.ceiling_subs.append(rospy.Subscriber("/Lunalab/CeilingLights/Color", ColorRGBA, self.setCeilingColor, queue_size=1))
        # self.curtains_subs = []
        # self.curtains_subs.append(rospy.Subscriber("/Lunalab/Curtains/Extend", Bool, self.setCurtainsMode, queue_size=1))
        self.terrains_subs = []
        self.terrains_subs.append(
            rospy.Subscriber(
                "/Lunalab/Terrain/Switch", Int8, self.switchTerrain, queue_size=1
            )
        )
        self.terrains_subs.append(
            rospy.Subscriber(
                "/Lunalab/Terrain/EnableRocks", Bool, self.enableRocks, queue_size=1
            )
        )
        self.terrains_subs.append(
            rospy.Subscriber(
                "/Lunalab/Terrain/RandomizeRocks",
                Int32,
                self.randomizeRocks,
                queue_size=1,
            )
        )
        # self.terrains_subs.append(rospy.Subscriber("/Lunalab/Terrain/PlaceRocks", String, self.placeRocks, queue_size=1))
        self.render_subs = []
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/Render/EnableRTXRealTime",
                Empty,
                self.useRTXRealTimeRender,
                queue_size=1,
            )
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/Render/EnableRTXInteractive",
                Empty,
                self.useRTXInteractiveRender,
                queue_size=1,
            )
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/LensFlare/EnableLensFlares",
                Bool,
                self.setLensFlareOn,
                queue_size=1,
            )
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/LensFlare/NumBlades",
                Int8,
                self.setLensFlareNumBlade,
                queue_size=1,
            )
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/LensFlare/Scale",
                Float32,
                self.setLensFlareScale,
                queue_size=1,
            )
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/LensFlare/ApertureRotation",
                Float32,
                self.setLensFlareApertureRotation,
                queue_size=1,
            )
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/LensFlare/FocalLength",
                Float32,
                self.setLensFlareFocalLength,
                queue_size=1,
            )
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/LensFlare/Fstop",
                Float32,
                self.setLensFlareFstop,
                queue_size=1,
            )
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/LensFlare/SensorAspectRatio",
                Float32,
                self.setLensFlareSensorAspectRatio,
                queue_size=1,
            )
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/LensFlare/SensorDiagonal",
                Float32,
                self.setLensFlareSensorDiagonal,
                queue_size=1,
            )
        )
        self.robot_subs = []
        self.robot_subs.append(
            rospy.Subscriber(
                "/Lunalab/Robots/Spawn", PoseStamped, self.spawnRobot, queue_size=1
            )
        )
        self.robot_subs.append(
            rospy.Subscriber(
                "/Lunalab/Robots/Teleport",
                PoseStamped,
                self.teleportRobot,
                queue_size=1,
            )
        )
        self.robot_subs.append(
            rospy.Subscriber(
                "/Lunalab/Robots/Reset", String, self.resetRobot, queue_size=1
            )
        )
        self.robot_subs.append(
            rospy.Subscriber(
                "/Lunalab/Robots/ResetAll", String, self.resetRobots, queue_size=1
            )
        )
        self.domain_id = 0
        self.modifications = []

    def clearModifications(self):
        """
        Clears the list of modifications to be applied to the lab."""

        self.modifications = []

    def applyModifications(self):
        """
        Applies the list of modifications to the lab."""

        for mod in self.modifications:
            mod[0](*mod[1])
        self.clearModifications()

    def reset(self):
        """
        Resets the lab to its initial state."""

        pass

    def setProjectorOn(self, data: Bool) -> None:
        """
        Turns the projector on or off.

        Args:
            data (Bool): True to turn the projector on, False to turn it off."""

        self.modifications.append([self.LC.turnProjectorOnOff, [data.data]])

    def setProjectorIntensity(self, data: Float32) -> None:
        """
        Sets the projector intensity.

        Args:
            data (Float32): Intensity in percentage."""

        default_intensity = 120000000.0
        data = default_intensity * float(data.data) / 100.0
        self.modifications.append([self.LC.setProjectorIntensity, [data]])

    def setProjectorRadius(self, data: Float32) -> None:
        """
        Sets the projector radius.

        Args:
            data (Float32): Radius in meters."""

        self.modifications.append([self.LC.setProjectorRadius, [data.data]])

    def setProjectorColor(self, data: ColorRGBA) -> None:
        """
        Sets the projector color.

        Args:
            data (ColorRGBA): Color in RGBA format."""

        color = [data.r, data.g, data.b]
        self.modifications.append([self.LC.setProjectorColor, [color]])

    def setProjectorPose(self, data: Pose) -> None:
        """
        Sets the projector pose.

        Args:
            data (Pose): Pose in ROS Pose format."""

        position = [data.position.x, data.position.y, data.position.z]
        quaternion = [
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w,
        ]
        self.modifications.append([self.LC.setProjectorPose, [(position, quaternion)]])

    def setCeilingOn(self, data: Bool) -> None:
        """
        Turns the ceiling lights on or off.

        Args:
            data (Bool): True to turn the lights on, False to turn them off."""

        self.modifications.append([self.LC.turnRoomLightsOnOff, [data.data]])

    def setCeilingIntensity(self, data: Float32) -> None:
        """
        Sets the ceiling lights intensity.

        Args:
            data (Float32): Intensity in percentage."""

        self.modifications.append([self.LC.setRoomLightsIntensity, [data.data]])

    def setCeilingRadius(self, data: Float32) -> None:
        """
        Sets the ceiling lights radius.

        Args:
            data (Float32): Radius in meters."""

        self.modifications.append([self.LC.setRoomLightsRadius, [data.data]])

    def setCeilingFOV(self, data: Float32) -> None:
        """
        Sets the ceiling lights field of view.

        Args:
            data (Float32): Field of view in degrees."""

        self.modifications.append([self.LC.setRoomLightsFOV, [data.data]])

    def setCeilingColor(self, data: ColorRGBA) -> None:
        """
        Sets the ceiling lights color.

        Args:
            data (ColorRGBA): Color in RGBA format."""

        color = [data.r, data.g, data.b]
        self.modifications.append([self.LC.setRoomLightsColor, [color]])

    def setCurtainsMode(self, data: Bool) -> None:
        """
        Sets the curtains mode.

        Args:
            data (Bool): True to extend the curtains, False to retract them."""

        self.modifications.append([self.LC.curtainsExtend, [data.data]])

    def switchTerrain(self, data: Bool) -> None:
        """
        Switches the terrain.

        Args:
            data (Int32): 0 for the first terrain, 1 for the second terrain."""

        self.modifications.append([self.LC.switchTerrain, [data.data]])

    def enableRocks(self, data: Bool) -> None:
        """
        Enables or disables the rocks.

        Args:
            data (Bool): True to enable the rocks, False to disable them."""

        self.modifications.append([self.LC.enableRocks, [data.data]])

    def randomizeRocks(self, data: Int32) -> None:
        """
        Randomizes the rocks.

        Args:
            data (Int32): Number of rocks to randomize."""

        data = int(data.data)
        self.modifications.append([self.LC.randomizeRocks, [data]])

    def placeRocks(self, data: String) -> None:
        """
        Places the rocks.

        Args:
            data (str): Path to the file containing the rocks positions."""

        self.modifications.append([self.LC.placeRocks, [data.data]])

    def useRTXRealTimeRender(self, data: Empty) -> None:
        """
        Enables or disables RTX real time rendering.

        Args:
            data (Empty): Empty message."""

        self.modifications.append([self.LC.enableRTXRealTime, [0]])

    def useRTXInteractiveRender(self, data: Empty) -> None:
        """
        Enables or disables RTX interactive rendering.

        Args:
            data (Empty): Empty message."""

        self.modifications.append([self.LC.enableRTXInteractive, [0]])

    def setLensFlareOn(self, data: Bool) -> None:
        """
        Enables or disables lens flares.

        Args:
            data (Bool): True to enable lens flares, False to disable them."""

        self.modifications.append([self.LC.enableLensFlare, [data.data]])

    def setLensFlareNumBlade(self, data: Int8) -> None:
        """
        Sets the number of blades of the lens flares.

        Args:
            data (Int8): Number of blades."""

        data = int(data.data)
        self.modifications.append([self.LC.setFlareNumBlades, [data]])

    def setLensFlareScale(self, data: Float32) -> None:
        """
        Sets the scale of the lens flares.

        Args:
            data (Float32): Scale."""

        data = float(data.data)
        self.modifications.append([self.LC.setFlareScale, [data]])

    def setLensFlareFstop(self, data: Float32) -> None:
        """
        Sets the fstop of the lens flares.

        Args:
            data (Float32): Fstop."""

        data = float(data.data)
        self.modifications.append([self.LC.setFlareFstop, [data]])

    def setLensFlareFocalLength(self, data: Float32) -> None:
        """
        Sets the focal length of the lens flares.

        Args:
            data (Float32): Focal length."""

        data = float(data.data)
        self.modifications.append([self.LC.setFlareFocalLength, [data]])

    def setLensFlareSensorAspectRatio(self, data: Float32) -> None:
        """
        Sets the sensor aspect ratio of the lens flares.

        Args:
            data (Float32): Sensor aspect ratio."""

        data = float(data.data)
        self.modifications.append([self.LC.setFlareSensorAspectRatio, [data]])

    def setLensFlareSensorDiagonal(self, data: Float32) -> None:
        """
        Sets the sensor diagonal of the lens flares.

        Args:
            data (Float32): Sensor diagonal."""

        data = float(data.data)
        self.modifications.append([self.LC.setFlareSensorDiagonal, [data]])

    def setLensFlareApertureRotation(self, data: Float32) -> None:
        """
        Sets the aperture rotation of the lens flares.

        Args:
            data (Float32): Aperture rotation."""

        data = float(data.data)
        self.modifications.append([self.LC.setFlareApertureRotation, [data]])

    def spawnRobot(self, data: PoseStamped) -> None:
        """
        Spawns a robot.

        Args:
            data (String): Name and path of the robot to spawn.
                           Must be in the format: robot_name:usd_path"""

        assert (
            len(data.header.frame_id.split(":")) == 2
        ), "The data should be in the format: robot_name:usd_path"
        robot_name, usd_path = data.header.frame_id.split(":")
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [
            data.pose.orientation.w,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.x,
        ]
        self.modifications.append([self.RM.addRobot, [usd_path, robot_name, p, q, self.domain_id]])

    def teleportRobot(self, data: PoseStamped) -> None:
        """
        Teleports a robot.

        Args:
            data (Pose): Pose in ROS2 Pose format."""

        robot_name = data.header.frame_id
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        ]
        self.modifications.append([self.RM.teleportRobot, [robot_name, p, q]])

    def resetRobot(self, data: String) -> None:
        """
        Resets a robot.

        Args:
            data (String): Name of the robot to reset."""

        robot_name = data.data
        self.modifications.append([self.RM.resetRobot, [robot_name]])

    def resetRobots(self, data: String):
        """
        Resets all the robots.

        Args:
            data (Int32): Dummy argument."""

        self.modifications.append([self.RM.resetRobots, []])

    def cleanScene(self):
        """
        Cleans the scene."""

        for sub in self.projector_subs:
            sub.unregister()
        for sub in self.ceiling_subs:
            sub.unregister()
        # for sub in self.curtains_subs:
        #    sub.unregister()
        for sub in self.terrains_subs:
            sub.unregister()
        for sub in self.render_subs:
            sub.unregister()
        for sub in self.robot_subs:
            sub.unregister()
        rospy.signal_shutdown("Shutting down")