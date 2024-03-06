__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

# Custom libs
from src.environments.lunaryard import LunaryardController
from src.environments.lunaryard_deformable import LunaryardDeformableController
from src.configurations.rendering_confs import FlaresConf
from src.robots.robot import RobotManager
from src.robots.view import FourWheelRigidPrim, FourWheelRigidPrimView
from src.environments_wrappers.ros1.lunaryard_ros1 import ROS_LunaryardManager
from src.physics.terra_param import RobotParameter, TerrainMechanicalParameter
from src.physics.terramechanics_solver import TerramechanicsSolver

# Loads ROS1 dependent libraries
import rospy
from std_msgs.msg import Bool, Float32, ColorRGBA, Int8, Int32, String, Empty
from geometry_msgs.msg import Pose, PoseStamped
import os
import numpy as np


class ROS_LunaryardDeformableManager(ROS_LunaryardManager):
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

        self.LC = LunaryardDeformableController(**environment_cfg, flares_settings=flares_cfg)
        self.RM = RobotManager(
            uses_nucleus=False, is_ROS2=False, max_robots=5, robots_root="/Robots"
        )
        self.LC.load()

        self.TS = TerramechanicsSolver(
            robot_param=RobotParameter(),
            terrain_param=TerrainMechanicalParameter(),
        )
        
        self.scene = None
        self.robot_prim = FourWheelRigidPrim("/Robots")
        self.robot_prim_view = FourWheelRigidPrimView("/Robots")

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
                "/Lunalab/Projector/Pose", Pose, self.setProjectorPose, queue_size=1
            )
        )
        # self.projector_subs.append(rospy.Subscriber("/Lunalab/Projector/Color", ColorRGBA, self.setProjectorColor, queue_size=1))
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
                "/Lunalab/Robots/ResetAll", Empty, self.resetRobots, queue_size=1
            )
        )

        self.modifications = []
        self.world_poses = []
    
    def set_scene_asset(self):
        """
        Sets the scene asset."""
        #TODO: parse from hydra config.
        # usd_path = os.path.join(os.getcwd(), "assets/USD_Assets/robots/EX1_steer_ROS1.usd")
        # usd_path = os.path.join(os.getcwd(), "assets/USD_Assets/robots/EX1_steer_D435i_ROS1.usd")
        usd_path = os.path.join(os.getcwd(), "assets/USD_Assets/robots/ex1_camera.usd")
        robot_name = "ex1"
        p = [5.0, 5.0, 0.5]
        q = [0, 0, 0, 1]
        domain_id = "0"
        self.RM.addRobot(usd_path, robot_name, p, q, domain_id)
    
    def set_scene_view(self):
        """
        Sets the robot prim and rigid body view."""
        robot_name = "ex1"
        self.robot_prim.initialize(robot_name)
        self.robot_prim_view.initialize(robot_name, self.scene)

    def set_world_scene(self, scene):
        """
        Sets the world scene."""
        self.scene = scene
    
    ### Non ROS callback ####
    def deformTerrain(self) -> None:
        """
        Deforms the terrain."""
        world_pose = self.robot_prim.get_world_poses()
        self.LC.deformTerrain(world_pose)
        self.world_poses.append(world_pose)
        np.save("wheel_trajectory_lunaryard.npy", np.array(self.world_poses))

    
    def applyTerramechanics(self)->None:
        """
        Applies the terramechanics force and torque."""
        velocities, omega = self.robot_prim.get_velocities()
        sinkages = self.robot_prim.get_sinkages()
        force, torque = self.TS.compute_force_and_torque(velocities, omega, sinkages)
        self.robot_prim_view.apply_force_torque(force, torque)