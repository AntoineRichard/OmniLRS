__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import Dict, List, Tuple
import numpy as np
import warnings
import os

import omni
import omni.graph.core as og
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.transformations import (
    get_relative_transform,
    pose_from_tf_matrix,
)
from omni.isaac.core.utils.rotations import quat_to_rot_matrix
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.prims import RigidPrim, RigidPrimView
from pxr import Gf, UsdGeom, Usd

from WorldBuilders.pxr_utils import createXform, createObject, setDefaultOps
from src.configurations.robot_confs import RobotManagerConf


class RobotManager:
    """
    RobotManager class.
    It allows to spawn, reset, teleport robots. It also allows to automatically add namespaces to topics,
    and tfs to enable multi-robot operation."""

    def __init__(
        self, RM_conf:RobotManagerConf,
    ) -> None:
        """
        Args:
            uses_nucleus (bool, optional): Whether the robots are loaded from the nucleus or not. Defaults to False.
            is_ROS2 (bool, optional): Whether the robots are ROS2 enabled or not. Defaults to False.
            max_robots (int, optional): Maximum number of robots that can be spawned. Defaults to 5.
            robots_root (str, optional): The root path of the robots. Defaults to "/Robots".
        """

        self.stage = omni.usd.get_context().get_stage()
        self.RM_conf = RobotManagerConf(**RM_conf)
        self.robot_parameters = self.RM_conf.parameters
        self.uses_nucleus = self.RM_conf.uses_nucleus
        self.is_ROS2 = self.RM_conf.is_ROS2
        self.max_robots = self.RM_conf.max_robots
        self.robots_root = self.RM_conf.robots_root
        createXform(self.stage, self.robots_root)
        self.robots = {}
        self.robots_RG = {}
        self.num_robots = 0
    
    def preloadRobot(
        self, 
        world: Usd.Stage,
    ) -> None:
        if len(self.robot_parameters) > 0:
            for robot_parameter in self.robot_parameters:
                self.addRobot(
                    robot_parameter.usd_path, 
                    robot_parameter.robot_name, 
                    robot_parameter.pose.position, 
                    robot_parameter.pose.orientation, 
                    robot_parameter.domain_id, 
                )
                self.addRRG(
                    robot_parameter.robot_name, 
                    robot_parameter.target_links, 
                    world,
                )

    def addRobot(
        self,
        usd_path: str = None,
        robot_name: str = None,
        p: Tuple[float, float, float] = [0, 0, 0],
        q: Tuple[float, float, float, float] = [0, 0, 0, 1],
        domain_id: int = None,
    ) -> None:
        """
        Add a robot to the scene.

        Args:
            usd_path (str): The path of the robot's usd file.
            robot_name (str): The name of the robot.
            domain_id (int): The domain id of the robot."""

        if robot_name[0] != "/":
            robot_name = "/" + robot_name
        if self.num_robots >= self.max_robots:
            pass
        else:
            if robot_name in self.robots.keys():
                warnings.warn("Robot already exists. Ignoring request.")
            else:
                self.robots[robot_name] = Robot(
                    usd_path,
                    robot_name,
                    is_on_nucleus=self.uses_nucleus,
                    is_ROS2=self.is_ROS2,
                    domain_id=domain_id,
                    robots_root=self.robots_root,
                )
                self.robots[robot_name].load(p, q)
                self.num_robots += 1
    
    def addRRG(
        self, 
        robot_name: str = None, 
        target_links: List[str] = None,
        world=None,
    )-> None:
        """
        Add a robot rigid group to the scene.

        Args:
            robot_name (str): The name of the robot.
            target_links (List[str]): List of link names. 
            scene (Usd.Stage): usd stage scene."""
        rrg = RobotRigidGroup(
            self.robots_root, 
            robot_name, 
            target_links, 
        )
        rrg.initialize(world)
        self.robots_RG[robot_name] = rrg

    def resetRobots(self) -> None:
        """
        Reset all the robots to their original position."""

        for robot in self.robots.keys():
            self.robots[robot].reset()

    def resetRobot(self, robot_name: str) -> None:
        """
        Reset a specific robot to its original position.

        Args:
            robot_name (str): The name of the robot."""

        if robot_name in self.robots.keys():
            self.robots[robot_name].reset()
        else:
            warnings.warn("Robot does not exist. Ignoring request.")

    def teleportRobot(
        self, robot_name: str, position: np.ndarray, orienation: np.ndarray
    ) -> None:
        """
        Teleport a specific robot to a specific position and orientation.

        Args:
            robot_name (str): The name of the robot."""
        if robot_name in self.robots.keys():
            self.robots[robot_name].teleport(position, orienation)
        else:
            warnings.warn("Robot does not exist. Ignoring request.")


class Robot:
    """
    Robot class.
    It allows to spawn, reset, teleport a robot. It also allows to automatically add namespaces to topics,
    and tfs to enable multi-robot operation."""

    def __init__(
        self,
        usd_path: str,
        robot_name: str,
        robots_root: str = "/Robots",
        is_on_nucleus: bool = False,
        is_ROS2: bool = False,
        domain_id: int = 0,
    ) -> None:
        """
        Args:
            usd_path (str): The path of the robot's usd file.
            robot_name (str): The name of the robot.
            robots_root (str, optional): The root path of the robots. Defaults to "/Robots".
            is_on_nucleus (bool, optional): Whether the robots are loaded from the nucleus or not. Defaults to False.
            is_ROS2 (bool, optional): Whether the robots are ROS2 enabled or not. Defaults to False.
            domain_id (int, optional): The domain id of the robot. Defaults to 0."""

        self.stage = omni.usd.get_context().get_stage()
        self.usd_path = str(usd_path)
        self.robots_root = robots_root
        self.robot_name = robot_name
        self.robot_path = os.path.join(self.robots_root, self.robot_name.strip("/"))
        self.is_on_nucleus = is_on_nucleus
        self.is_ROS2 = is_ROS2
        self.domain_id = int(domain_id)
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.root_body_id = None

    def getRootRigidBodyPath(self) -> None:
        """
        Get the root rigid body path of the robot."""

        art = self.dc.get_articulation(self.robot_path)
        self.root_body_id = self.dc.get_articulation_root_body(art)

    def editGraphs(self) -> None:
        """
        Edit the graphs of the robot to add namespaces to topics and tfs."""

        selected_paths = []
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(self.robot_path)):
            l = [
                attr
                for attr in prim.GetAttributes()
                if attr.GetName().split(":")[0] == "graph"
            ]
            if l:
                selected_paths.append(prim.GetPath())

        for path in selected_paths:
            prim = self.stage.GetPrimAtPath(path)
            prim.GetAttribute("graph:variable:Namespace").Set(self.robot_name)
            if self.is_ROS2:
                prim.GetAttribute("graph:variable:Context").Set(self.domain_id)

    def load(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """
        Load the robot in the scene, and automatically edit its graphs.

        Args:
            position (np.ndarray): The position of the robot.
            orientation (np.ndarray): The orientation of the robot."""

        self.stage = omni.usd.get_context().get_stage()
        self.setResetPose(position, orientation)
        if self.is_on_nucleus:
            nucleus = get_assets_root_path()
            self.usd_path = os.path.join(nucleus, self.usd_path)
        createObject(
            self.robot_path,
            self.stage,
            self.usd_path,
            is_instance=False,
            position=Gf.Vec3d(*position),
            rotation=Gf.Quatd(*orientation),
        )
        self.editGraphs()

    def getPose(self) -> List[float]:
        """
        Get the pose of the robot."""

        source_prim = UsdGeom.Xformable(
            self.stage.GetPrimAtPath(self._robot_base_link_path)
        )
        target_prim = UsdGeom.Xformable(self.stage.GetPrimAtPath(self._root_path))
        relative_transform = get_relative_transform(source_prim, target_prim)
        translation, rotation = pose_from_tf_matrix(relative_transform)
        return [
            translation[0],
            translation[1],
            translation[2],
            rotation[1],
            rotation[2],
            rotation[3],
            rotation[0],
        ]

    def setResetPose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """
        Set the reset pose of the robot.

        Args:
            position (np.ndarray): The position of the robot.
            orientation (np.ndarray): The orientation of the robot."""

        self.reset_position = position
        self.reset_orientation = orientation

    def teleport(self, p: List[float], q: List[float]) -> None:
        """
        Teleport the robot to a specific position and orientation.

        Args:
            p (list): The position of the robot.
            q (list): The orientation of the robot."""

        self.getRootRigidBodyPath()
        transform = _dynamic_control.Transform(p, q)
        self.dc.set_rigid_body_pose(self.root_body_id, transform)
        self.dc.set_rigid_body_linear_velocity(self.root_body_id, [0, 0, 0])
        self.dc.set_rigid_body_angular_velocity(self.root_body_id, [0, 0, 0])

    def reset(self) -> None:
        """
        Reset the robot to its original position and orientation."""

        # w = self.reset_orientation.GetReal()
        # xyz = self.reset_orientation.GetImaginary()
        self.teleport(
            [self.reset_position[0], self.reset_position[1], self.reset_position[2]],
            [
                self.reset_orientation[1],
                self.reset_orientation[2],
                self.reset_orientation[3],
                self.reset_orientation[0],
            ],
        )

class RobotRigidGroup:
    """
    Class which deals with rigidprims and rigidprimview of a single robot.
    It is used to retrieve world pose, and contact forces, or apply force/torque. 
    """
    def __init__(self, 
                 root_path:str="/Robots", 
                 robot_name:str=None, 
                 target_links:List[str]=None):
        self.root_path = root_path
        self.robot_name = robot_name
        self.target_links = target_links
        self.prims = []
        self.prim_views = []

    def initialize(self, world)->None:
        world.reset()
        if len(self.target_links) > 0:
            for target_link in self.target_links:
                rigid_prim = RigidPrim(
                    prim_path=os.path.join(self.root_path, self.robot_name, target_link), 
                    name=f"{self.robot_name}/{target_link}")
                rigid_prim_view = RigidPrimView(prim_paths_expr=os.path.join(self.root_path, self.robot_name, target_link),
                                                name=f"{self.robot_name}/{target_link}_view",
                                                track_contact_forces=True,
                                                    )
                rigid_prim_view.initialize()
                self.prims.append(rigid_prim)
                self.prim_views.append(rigid_prim_view)
        world.reset()
    
    def get_world_poses(self)->np.ndarray:
        """
        Returns the world pose matrix of target links.
        """
        n_links = len(self.target_links)
        pose = np.zeros((n_links, 4, 4))
        for i, prim in enumerate(self.prims):
            position, orientation = prim.get_world_pose()
            orientation = quat_to_rot_matrix(orientation)
            pose[i, :3, 3] = 1
            pose[i, :3, :3] = orientation
            pose[i, :3, 3] = position
        return pose
    
    def get_velocities(self)->Tuple[np.ndarray, np.ndarray]:
        """
        Returns the linear/angular velocity of target links.
        """
        n_links = len(self.target_links)
        linear_velocities = np.zeros((n_links, 3))
        angular_velocities = np.zeros((n_links, 3))
        for i, prim in enumerate(self.prims):
            linear_velocity, angular_velocity = prim.get_velocities()
            linear_velocities[i, :] = linear_velocity
            angular_velocities[i, :] = angular_velocity
        return linear_velocities, angular_velocities
    
    def get_net_contact_forces(self)->np.ndarray:
        """
        Returns net contact forces on each target link.
        """
        n_links = len(self.target_links)
        contact_forces = np.zeros((n_links, 3))
        for i, prim_view in enumerate(self.prim_views):
            contact_force = prim_view.get_net_contact_forces().squeeze()
            contact_forces[i, :] = contact_force
        return contact_forces
    
    def apply_force_torque(self, forces:np.ndarray, torques:np.ndarray)->None:
        """
        Apply force and torque (defined in local body frame) to body frame of the four wheels.
        Args:
            forces (np.ndarray): The forces to apply to the body origin of the four wheels.
                                 (Fx, Fy, Fz) = (F_DP, F_S, F_N)
            torques (np.ndarray): The torques to apply to the body origin of the four wheels.
                                 (Mx, My, Mz0 = (M_O,-M_R, M_S)
        """
        n_links = len(self.target_links)
        assert forces.shape[0] == n_links, "given force does not have matching shape."
        assert torques.shape[0] == n_links, "given torque does not have matching shape."
        for i, prim_view in enumerate(self.prim_views):
            prim_view.apply_forces_and_torques_at_pos(forces=forces[i], torques=torques[i], is_global=False)