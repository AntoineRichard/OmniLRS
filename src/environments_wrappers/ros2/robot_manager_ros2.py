__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from pxr import Gf

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from src.robots.robot import RobotManager


class ROS_RobotManager(Node):
    """
    ROS2 node that manages the robots."""

    def __init__(self, RM_conf:dict) -> None:
        super().__init__("Robot_spawn_manager_node")
        self.RM = RobotManager(RM_conf)

        self.create_subscription(
            PoseStamped, "/Lunalab/Robots/Spawn", self.spawnRobot, 1
        )
        self.create_subscription(
            PoseStamped, "/Lunalab/Robots/Teleport", self.teleportRobot, 1
        )
        self.create_subscription(String, "/Lunalab/Robots/Reset", self.resetRobot, 1)
        self.create_subscription(Empty, "/Lunalab/Robots/ResetAll", self.resetRobots, 1)

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
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
        ]
        self.modifications.append(
            [self.RM.addRobot, [usd_path, robot_name, p, q, self.domain_id]]
        )

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

    def resetRobots(self, data: Empty):
        """
        Resets all the robots.

        Args:
            data (Int32): Dummy argument."""

        self.modifications.append([self.RM.resetRobots, []])

    def cleanRobots(self):
        """
        Cleans the robots."""

        self.destroy_node()