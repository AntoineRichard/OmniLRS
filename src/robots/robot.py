""""
  file: robot.py
  authors: antoine.richard@uni.lu
           kamohara.junnosuke.t6@dc.tohoku.ac.jp
  version: 0.1
  date: 2023-09-04
  
  copyright: University of Luxembourg | SnT | SpaceR 2023--2023
             Tohoku University | SRL 2023--2023
  brief: Robot manager file.
  details: This file implements a set of methods to manage robots inside Nvidia's Isaac Sim simulator.
           It allows to spawn, reset, teleport robots. It also allows to automatically add namespaces to topics,
           to enable multi-robot operation.
"""

from typing import Dict, List
import numpy as np
import os

import omni
import omni.graph.core as og
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.transformations import get_relative_transform, pose_from_tf_matrix
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.dynamic_control import _dynamic_control
from WorldBuilders.pxr_utils import createXform, createObject, setDefaultOps
from pxr import Gf, UsdGeom, Usd


class RobotManager:
    """
    RobotManager class.
    It allows to spawn, reset, teleport robots. It also allows to automatically add namespaces to topics,
    and tfs to enable multi-robot operation."""

    def __init__(self, spawning_poses:List[Dict[str,List[float]]], uses_nucleus=False, is_ROS2=False, max_robots=5, robots_root="/Robots") -> None:
        """
        Args:
            spawning_poses (list): List of dictionaries with coordinates containing the spawning poses of the robots.
            uses_nucleus (bool, optional): Whether the robots are loaded from the nucleus or not. Defaults to False.
            is_ROS2 (bool, optional): Whether the robots are ROS2 enabled or not. Defaults to False.
            max_robots (int, optional): Maximum number of robots that can be spawned. Defaults to 5.
            robots_root (str, optional): The root path of the robots. Defaults to "/Robots"."""
        
        self.stage = omni.usd.get_context().get_stage()
        self.spawning_poses = spawning_poses
        self.uses_nucleus = uses_nucleus
        self.is_ROS2 = is_ROS2
        self.max_robots = max_robots
        self.robots_root = robots_root
        createXform(self.stage, self.robots_root)
        self.robots = {}
        self.num_robots = 0
        assert(len(spawning_poses) >= max_robots)

    def addRobot(self, usd_path:str, robot_name:str, domain_id:int) -> None:
        """
        Add a robot to the scene.
        
        Args:
            usd_path (str): The path of the robot's usd file.
            robot_name (str): The name of the robot.
            domain_id (int): The domain id of the robot."""
        
        if robot_name[0] != "/":
            robot_name = "/"+robot_name
        if self.num_robots >= self.max_robots:
            pass
        else:
            self.robots[robot_name] = Robot(usd_path, robot_name, is_on_nucleus=self.uses_nucleus, is_ROS2=self.is_ROS2, domain_id=domain_id, robots_root=self.robots_root)
            self.robots[robot_name].load(self.spawning_poses[self.num_robots]["position"], self.spawning_poses[self.num_robots]["orientation"])
            self.num_robots += 1

    def resetRobots(self) -> None:
        """
        Reset all the robots to their original position."""

        for robot in self.robots.keys():
            self.robots[robot].reset()

    def resetRobot(self, robot_name:str) -> None:
        """
        Reset a specific robot to its original position.
        
        Args:
            robot_name (str): The name of the robot."""

        if robot_name in self.robots.keys():
            self.robots[robot_name].reset()

    def teleportRobot(self, robot_name:str, position:np.ndarray, orienation:np.ndarray) -> None:
        """
        Teleport a specific robot to a specific position and orientation.
        
        Args:
            robot_name (str): The name of the robot."""

        self.robots[robot_name].teleport(position, orienation)

class Robot:
    """
    Robot class.
    It allows to spawn, reset, teleport a robot. It also allows to automatically add namespaces to topics,
    and tfs to enable multi-robot operation."""
    
    def __init__(self, usd_path:str, robot_name:str, robots_root:str="/Robots", is_on_nucleus:bool=False, is_ROS2:bool=False, domain_id:int=0) -> None:
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
            l =  [attr for attr in prim.GetAttributes() if attr.GetName().split(':')[0] == "graph"]
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
            self.usd_path = os.path.join(nucleus,self.usd_path)
        print(self.robot_path)
        createObject(self.robot_path, self.stage, self.usd_path, is_instance=False, position=position, rotation=orientation)
        self.editGraphs()
    
    def getPose(self) -> List[float]:
        """
        Get the pose of the robot."""

        source_prim = UsdGeom.Xformable(self.stage.GetPrimAtPath(self._robot_base_link_path))
        target_prim = UsdGeom.Xformable(self.stage.GetPrimAtPath(self._root_path))
        relative_transform = get_relative_transform(source_prim, target_prim)
        translation, rotation = pose_from_tf_matrix(relative_transform)
        return [translation[0], translation[1], translation[2], rotation[1], rotation[2], rotation[3], rotation[0]]

    def setResetPose(self, position:np.ndarray, orientation:np.ndarray) -> None:
        """
        Set the reset pose of the robot.
        
        Args:
            position (np.ndarray): The position of the robot.
            orientation (np.ndarray): The orientation of the robot."""

        self.reset_position = position
        self.reset_orientation = orientation

    def teleport(self, p:List[float], q: List[float]) -> None:
        """
        Teleport the robot to a specific position and orientation.
        
        Args:
            p (list): The position of the robot.
            q (list): The orientation of the robot."""
        
        self.getRootRigidBodyPath()
        transform = _dynamic_control.Transform(p, q)
        self.dc.set_rigid_body_pose(self.root_body_id, transform)
        self.dc.set_rigid_body_linear_velocity(self.root_body_id, [0,0,0])
        self.dc.set_rigid_body_angular_velocity(self.root_body_id, [0,0,0])

    def reset(self) -> None:
        """
        Reset the robot to its original position and orientation."""

        w = self.reset_orientation.GetReal()
        xyz = self.reset_orientation.GetImaginary()
        self.teleport([self.reset_position[0], self.reset_position[1], self.reset_position[2]], [xyz[0],xyz[1],xyz[2],w])