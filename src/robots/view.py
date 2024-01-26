from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.utils.rotations import quat_to_rot_matrix
import os
import numpy as np

class FourWheelView:
    """
    FourWheelView is a class that provides a view of the four wheels of a vehicle.
    """
    def __init__(self, root_path:str="/Robots"):
        self.root_path = root_path

    def initialize(self, robot_name:str, scene)->None:
        self.left_front_wheel_path = os.path.join(self.root_path, robot_name, "left_front_wheel_link")
        self.left_rear_wheel_path = os.path.join(self.root_path, robot_name, "left_rear_wheel_link")
        self.right_front_wheel_path = os.path.join(self.root_path, robot_name, "right_front_wheel_link")
        self.right_rear_wheel_path = os.path.join(self.root_path, robot_name, "right_rear_wheel_link")

        self.left_front_wheel_view = RigidPrimView(prim_paths_expr=self.left_front_wheel_path,
                                                   name="left_front_wheel_view",
                                                   track_contact_forces=True,
                                                   )
        self.left_rear_wheel_view = RigidPrimView(prim_paths_expr=self.left_rear_wheel_path,
                                                    name="left_rear_wheel_view",
                                                    track_contact_forces=True,
                                                    )
        self.right_front_wheel_view = RigidPrimView(prim_paths_expr=self.right_front_wheel_path,
                                                    name="right_front_wheel_view",
                                                    track_contact_forces=True,
                                                    )
        self.right_rear_wheel_view = RigidPrimView(prim_paths_expr=self.right_rear_wheel_path,
                                                    name="right_rear_wheel_view",
                                                    track_contact_forces=True,
                                                    )
        self.left_front_wheel_view.initialize()
        self.left_rear_wheel_view.initialize()
        self.right_front_wheel_view.initialize()
        self.right_rear_wheel_view.initialize()
        
        scene.add(self.left_front_wheel_view)
        scene.add(self.left_rear_wheel_view)
        scene.add(self.right_front_wheel_view)
        scene.add(self.right_rear_wheel_view)
    
    def get_world_poses(self)->np.ndarray:
        """
        Returns the world pose matrix of the four wheels.
        """
        left_front_wheel_position, left_front_wheel_orientation = self.left_front_wheel_view.get_world_poses()
        left_rear_wheel_position, left_rear_wheel_orientation = self.left_rear_wheel_view.get_world_poses()
        right_front_wheel_position, right_front_wheel_orientation = self.right_front_wheel_view.get_world_poses()
        right_rear_wheel_position, right_rear_wheel_orientation = self.right_rear_wheel_view.get_world_poses()
        # squeeze
        left_front_wheel_position = left_front_wheel_position[-1]
        left_rear_wheel_position = left_rear_wheel_position[-1]
        right_front_wheel_position = right_front_wheel_position[-1]
        right_rear_wheel_position = right_rear_wheel_position[-1]
        # convert quaternion to rotation matrix
        left_front_wheel_orientation = quat_to_rot_matrix(left_front_wheel_orientation[-1])
        left_rear_wheel_orientation = quat_to_rot_matrix(left_rear_wheel_orientation[-1])
        right_front_wheel_orientation = quat_to_rot_matrix(right_front_wheel_orientation[-1])
        right_rear_wheel_orientation = quat_to_rot_matrix(right_rear_wheel_orientation[-1])
        # make 4x4 matrix
        left_front_wheel_pose = np.eye(4)
        left_rear_wheel_pose = np.eye(4)
        right_front_wheel_pose = np.eye(4)
        right_rear_wheel_pose = np.eye(4)
        left_front_wheel_pose[:3, :3] = left_front_wheel_orientation
        left_front_wheel_pose[:3, 3] = left_front_wheel_position
        left_rear_wheel_pose[:3, :3] = left_rear_wheel_orientation
        left_rear_wheel_pose[:3, 3] = left_rear_wheel_position
        right_front_wheel_pose[:3, :3] = right_front_wheel_orientation
        right_front_wheel_pose[:3, 3] = right_front_wheel_position
        right_rear_wheel_pose[:3, :3] = right_rear_wheel_orientation
        right_rear_wheel_pose[:3, 3] = right_rear_wheel_position
        return np.stack([left_front_wheel_pose, left_rear_wheel_pose, right_front_wheel_pose, right_rear_wheel_pose], axis=0)
    
    def get_net_contact_forces(self)->np.ndarray:
        """
        Returns the net contact forces of the four wheels.
        """
        left_front_wheel_contact_force = self.left_front_wheel_view.get_net_contact_forces()[-1]
        left_rear_wheel_contact_force = self.left_rear_wheel_view.get_net_contact_forces()[-1]
        right_front_wheel_contact_force = self.right_front_wheel_view.get_net_contact_forces()[-1]
        right_rear_wheel_contact_force = self.right_rear_wheel_view.get_net_contact_forces()[-1]
        return np.stack([left_front_wheel_contact_force, left_rear_wheel_contact_force, right_front_wheel_contact_force, right_rear_wheel_contact_force], axis=0)