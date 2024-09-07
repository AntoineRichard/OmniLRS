__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List

# Custom libs
from src.configurations.rendering_confs import FlaresConf
from src.environments.lunalab import LunalabController
import src.environments.rendering as rndr
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
            **kwargs: Additional keyword arguments.
        """

        self.LC = LunalabController(**environment_cfg, flares_settings=flares_cfg)
        self.RM = RobotManager(environment_cfg["robots_settings"])
        self.LC.load()

        self.projector_subs = []
        self.projector_subs.append(
            rospy.Subscriber("/Lunalab/Projector/TurnOn", Bool, self.set_projector_on, queue_size=1)
        )
        self.projector_subs.append(
            rospy.Subscriber("/Lunalab/Projector/Intensity", Float32, self.set_projector_intensity, queue_size=1)
        )
        self.projector_subs.append(
            rospy.Subscriber("/Lunalab/Projector/Radius", Float32, self.set_projector_radius, queue_size=1)
        )
        self.projector_subs.append(
            rospy.Subscriber("/Lunalab/Projector/Pose", Pose, self.set_projector_pose, queue_size=1)
        )
        # self.projector_subs.append(rospy.Subscriber("/Lunalab/Projector/Color", ColorRGBA, self.setProjectorColor, queue_size=1))
        self.ceiling_subs = []
        self.ceiling_subs.append(
            rospy.Subscriber("/Lunalab/CeilingLights/TurnOn", Bool, self.set_ceiling_on, queue_size=1)
        )
        self.ceiling_subs.append(
            rospy.Subscriber("/Lunalab/CeilingLights/Intensity", Float32, self.set_ceiling_intensity, queue_size=1)
        )
        # self.ceiling_subs.append(rospy.Subscriber("/Lunalab/CeilingLights/Radius", Float32, self.set_ceiling_radius, queue_size=1))
        # self.ceiling_subs.append(rospy.Subscriber("/Lunalab/CeilingLights/FOV", Float32, self.set_ceiling_FOV, queue_size=1))
        # self.ceiling_subs.append(rospy.Subscriber("/Lunalab/CeilingLights/Color", ColorRGBA, self.set_ceiling_color, queue_size=1))
        # self.curtains_subs = []
        # self.curtains_subs.append(rospy.Subscriber("/Lunalab/Curtains/Extend", Bool, self.set_curtains_mode, queue_size=1))
        self.terrains_subs = []
        self.terrains_subs.append(rospy.Subscriber("/Lunalab/Terrain/Switch", Int8, self.switch_terrain, queue_size=1))
        self.terrains_subs.append(
            rospy.Subscriber("/Lunalab/Terrain/EnableRocks", Bool, self.enable_rocks, queue_size=1)
        )
        self.terrains_subs.append(
            rospy.Subscriber("/Lunalab/Terrain/RandomizeRocks", Int32, self.randomize_rocks, queue_size=1)
        )
        self.render_subs = []
        self.render_subs.append(
            rospy.Subscriber("/Lunalab/Render/EnableRTXRealTime", Empty, self.use_RTX_real_time_render, queue_size=1)
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/Render/EnableRTXInteractive", Empty, self.use_RTX_interactive_Render, queue_size=1
            )
        )
        self.render_subs.append(
            rospy.Subscriber("/Lunalab/LensFlare/EnableLensFlares", Bool, self.set_lens_flare_on, queue_size=1)
        )
        self.render_subs.append(
            rospy.Subscriber("/Lunalab/LensFlare/NumBlades", Int8, self.set_lens_flare_num_blade, queue_size=1)
        )
        self.render_subs.append(
            rospy.Subscriber("/Lunalab/LensFlare/Scale", Float32, self.set_lens_flare_scale, queue_size=1)
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/LensFlare/ApertureRotation", Float32, self.set_lens_flare_aperture_rotation, queue_size=1
            )
        )
        self.render_subs.append(
            rospy.Subscriber("/Lunalab/LensFlare/FocalLength", Float32, self.set_lens_flare_focal_length, queue_size=1)
        )
        self.render_subs.append(
            rospy.Subscriber("/Lunalab/LensFlare/Fstop", Float32, self.set_lens_flare_fstop, queue_size=1)
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/LensFlare/SensorAspectRatio", Float32, self.set_lens_flare_sensor_aspect_ratio, queue_size=1
            )
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/Lunalab/LensFlare/SensorDiagonal", Float32, self.set_lens_flare_sensor_diagonal, queue_size=1
            )
        )
        self.robot_subs = []
        self.robot_subs.append(rospy.Subscriber("/Lunalab/Robots/Spawn", PoseStamped, self.spawn_robot, queue_size=1))
        self.robot_subs.append(
            rospy.Subscriber("/Lunalab/Robots/Teleport", PoseStamped, self.teleport_robot, queue_size=1)
        )
        self.robot_subs.append(rospy.Subscriber("/Lunalab/Robots/Reset", String, self.reset_robot, queue_size=1))
        self.robot_subs.append(rospy.Subscriber("/Lunalab/Robots/ResetAll", String, self.reset_robots, queue_size=1))
        self.domain_id = 0
        self.modifications = []

    def periodic_update(self, dt: float) -> None:
        pass

    def clear_modifications(self) -> None:
        """
        Clears the list of modifications to be applied to the lab.
        """

        self.modifications: List[callable, dict] = []

    def apply_modifications(self) -> None:
        """
        Applies the list of modifications to the lab.
        """

        for mod in self.modifications:
            mod[0](**mod[1])
        self.clear_modifications()

    def reset(self) -> None:
        """
        Resets the lab to its initial state.
        """
        pass

    def set_projector_on(self, data: Bool) -> None:
        """
        Turns the projector on or off.

        Args:
            data (Bool): True to turn the projector on, False to turn it off.
        """

        self.modifications.append([self.LC.turn_projector_on_off, {"flag": data.data}])

    def set_projector_intensity(self, data: Float32) -> None:
        """
        Sets the projector intensity.

        Args:
            data (Float32): Intensity in percentage.
        """

        assert 0.0 <= data.data <= 100.0, "Intensity must be between 0 and 100."
        default_intensity = 120000000.0
        data = default_intensity * float(data.data) / 100.0
        self.modifications.append([self.LC.set_projector_intensity, {"intensity": data}])

    def set_projector_radius(self, data: Float32) -> None:
        """
        Sets the projector radius.

        Args:
            data (Float32): Radius in meters.
        """

        assert data.data > 0.0, "Radius must be greater than 0.0"
        self.modifications.append([self.LC.set_projector_radius, {"radius": data.data}])

    def set_projector_color(self, data: ColorRGBA) -> None:
        """
        Sets the projector color.

        Args:
            data (ColorRGBA): Color in RGBA format.
        """

        color = [data.r, data.g, data.b]
        for c in color:
            assert 0.0 <= c <= 1.0, "Color values must be between 0 and 1."
        self.modifications.append([self.LC.set_projector_color, {"color": color}])

    def set_projector_pose(self, data: Pose) -> None:
        """
        Sets the projector pose.

        Args:
            data (Pose): Pose in ROS Pose format.
        """

        position = (data.position.x, data.position.y, data.position.z)
        orientation = (data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
        self.modifications.append([self.LC.set_projector_pose, {"position": position, "orientation": orientation}])

    def set_ceiling_on(self, data: Bool) -> None:
        """
        Turns the ceiling lights on or off.

        Args:
            data (Bool): True to turn the lights on, False to turn them off.
        """

        self.modifications.append([self.LC.turn_room_lights_on_off, {"flag": data.data}])

    def set_ceiling_intensity(self, data: Float32) -> None:
        """
        Sets the ceiling lights intensity.

        Args:
            data (Float32): Intensity in arbitrary units.
        """

        assert 0.0 <= data.data, "Intensity must be greater than 0."

        self.modifications.append([self.LC.set_room_lights_intensity, {"intensity": data.data}])

    def set_ceiling_radius(self, data: Float32) -> None:
        """
        Sets the ceiling lights radius.

        Args:
            data (Float32): Radius in meters.
        """

        assert data.data > 0.0, "Radius must be greater than 0."
        self.modifications.append([self.LC.set_room_lights_radius, {"radius": data.data}])

    def set_ceiling_FOV(self, data: Float32) -> None:
        """
        Sets the ceiling lights field of view.

        Args:
            data (Float32): Field of view in degrees.
        """

        assert 0.0 <= data.data <= 180.0, "Field of view must be between 0 and 180."
        self.modifications.append([self.LC.set_room_lights_FOV, {"FOV": data.data}])

    def set_ceiling_color(self, data: ColorRGBA) -> None:
        """
        Sets the ceiling lights color.

        Args:
            data (ColorRGBA): Color in RGBA format.
        """

        color = [data.r, data.g, data.b]
        for c in color:
            assert 0.0 <= c <= 1.0, "Color values must be between 0 and 1."
        self.modifications.append([self.LC.set_room_lights_color, {"color": color}])

    def set_curtains_mode(self, data: Bool) -> None:
        """
        Sets the curtains mode.

        Args:
            data (Bool): True to extend the curtains, False to retract them.
        """

        self.modifications.append([self.LC.curtains_extend, {"flag": data.data}])

    def switch_terrain(self, data: Bool) -> None:
        """
        Switches the terrain.

        Args:
            data (Int32): 0 for the first terrain, 1 for the second terrain.
        """

        self.modifications.append([self.LC.switch_terrain, {"flag": data.data}])

    def enable_rocks(self, data: Bool) -> None:
        """
        Enables or disables the rocks.

        Args:
            data (Bool): True to enable the rocks, False to disable them.
        """

        self.modifications.append([self.LC.enable_rocks, {"flag": data.data}])

    def randomize_rocks(self, data: Int32) -> None:
        """
        Randomizes the rocks.

        Args:
            data (Int32): Number of rocks to randomize.
        """

        data = int(data.data)
        assert data > 0, "Number of rocks must be greater than 0."
        self.modifications.append([self.LC.randomize_rocks, {"num": data}])

    def use_RTX_real_time_render(self, data: Empty) -> None:
        """
        Enables or disables RTX real time rendering.

        Args:
            data (Empty): Empty message.
        """

        self.modifications.append([rndr.enable_RTX_real_time, {}])

    def useRTX_interactive_render(self, data: Empty) -> None:
        """
        Enables or disables RTX interactive rendering.

        Args:
            data (Empty): Empty message.
        """

        self.modifications.append([rndr.enable_RTX_interactive, {}])

    def set_lens_flare_on(self, data: Bool) -> None:
        """
        Enables or disables lens flares.

        Args:
            data (Bool): True to enable lens flares, False to disable them.
        """

        self.modifications.append([rndr.enable_lens_flare, {"enable": data.data}])

    def set_lens_flare_num_blade(self, data: Int8) -> None:
        """
        Sets the number of blades of the lens flares.

        Args:
            data (Int8): Number of blades.
        """

        data = int(data.data)
        assert data > 2, "Number of blades must be greater than 2."
        self.modifications.append([rndr.set_flare_num_blades, {"value": data}])

    def set_lens_flare_scale(self, data: Float32) -> None:
        """
        Sets the scale of the lens flares.

        Args:
            data (Float32): Scale.
        """

        data = float(data.data)
        assert data >= 0.0, "Scale must be greater than or equal to 0."
        self.modifications.append([rndr.set_flare_scale, {"value": data}])

    def set_lens_flare_fstop(self, data: Float32) -> None:
        """
        Sets the fstop of the lens flares.

        Args:
            data (Float32): Fstop.
        """

        data = float(data.data)
        assert data > 0.0, "Fstop must be greater than 0."
        self.modifications.append([rndr.set_flare_fstop, {"value": data}])

    def set_lens_flare_focal_length(self, data: Float32) -> None:
        """
        Sets the focal length of the lens flares.

        Args:
            data (Float32): Focal length.
        """

        data = float(data.data)
        self.modifications.append([rndr.set_flare_focal_length, {"value": data}])

    def set_lens_flare_sensor_aspect_ratio(self, data: Float32) -> None:
        """
        Sets the sensor aspect ratio of the lens flares.

        Args:
            data (Float32): Sensor aspect ratio.
        """

        data = float(data.data)
        assert data > 0.0, "Sensor aspect ratio must be greater than 0."
        self.modifications.append([rndr.set_flare_sensor_aspect_ratio, {"value": data}])

    def set_lens_flare_sensor_diagonal(self, data: Float32) -> None:
        """
        Sets the sensor diagonal of the lens flares.

        Args:
            data (Float32): Sensor diagonal.
        """

        data = float(data.data)
        assert data > 0.0, "Sensor diagonal must be greater than 0."
        self.modifications.append([rndr.set_flare_sensor_diagonal, {"value": data}])

    def set_lens_flare_aperture_rotation(self, data: Float32) -> None:
        """
        Sets the aperture rotation of the lens flares.

        Args:
            data (Float32): Aperture rotation.
        """

        data = float(data.data)
        self.modifications.append([rndr.set_flare_aperture_rotation, {"value": data}])

    def spawn_robot(self, data: PoseStamped) -> None:
        """
        Spawns a robot.

        Args:
            data (String): Name and path of the robot to spawn.
                           Must be in the format: robot_name:usd_path
        """

        assert len(data.header.frame_id.split(":")) == 2, "The data should be in the format: robot_name:usd_path"
        robot_name, usd_path = data.header.frame_id.split(":")
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [
            data.pose.orientation.w,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.x,
        ]
        self.modifications.append([self.RM.add_robot, {"usd_path": usd_path, "robot_name": robot_name, "p": p, "q": q}])

    def teleport_robot(self, data: PoseStamped) -> None:
        """
        Teleports a robot.

        Args:
            data (Pose): Pose in ROS2 Pose format.
        """

        robot_name = data.header.frame_id
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        ]
        self.modifications.append([self.RM.teleport_robot, {"robot_name": robot_name, "position": p, "orientation": q}])

    def reset_robot(self, data: String) -> None:
        """
        Resets a robot.

        Args:
            data (String): Name of the robot to reset.
        """

        robot_name = data.data
        self.modifications.append([self.RM.reset_robot, {"robot_name": robot_name}])

    def reset_robots(self, data: String):
        """
        Resets all the robots.

        Args:
            data (Int32): Dummy argument.
        """

        self.modifications.append([self.RM.reset_robots, {}])

    def clean_scene(self):
        """
        Cleans the scene.
        """

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
