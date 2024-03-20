from omni.isaac.core.prims import RigidPrim, RigidPrimView
from omni.isaac.core.utils.rotations import quat_to_rot_matrix
import os
import numpy as np
from dataclasses import dataclass

class FourWheelRigidPrim:
    """
    FourWheelView is a class that provides a view of the four wheels of a vehicle.
    """
    def __init__(self, root_path:str="/Robots"):
        self.root_path = root_path

    def initialize(self, robot_name:str)->None:
        self.left_front_wheel_path = os.path.join(self.root_path, robot_name, "left_front_wheel_link")
        self.left_rear_wheel_path = os.path.join(self.root_path, robot_name, "left_rear_wheel_link")
        self.right_front_wheel_path = os.path.join(self.root_path, robot_name, "right_front_wheel_link")
        self.right_rear_wheel_path = os.path.join(self.root_path, robot_name, "right_rear_wheel_link")

        self.left_front_wheel = RigidPrim(prim_path=self.left_front_wheel_path,
                                                   name="left_front_wheel_view",
                                                   )
        self.left_rear_wheel = RigidPrim(prim_path=self.left_rear_wheel_path,
                                                    name="left_rear_wheel_view",
                                                    )
        self.right_front_wheel = RigidPrim(prim_path=self.right_front_wheel_path,
                                                    name="right_front_wheel_view",
                                                    )
        self.right_rear_wheel = RigidPrim(prim_path=self.right_rear_wheel_path,
                                                    name="right_rear_wheel_view",
                                                    )
    
    def get_world_poses(self)->np.ndarray:
        """
        Returns the world pose matrix of the four wheels.
        """
        left_front_wheel_position, left_front_wheel_orientation = self.left_front_wheel.get_world_pose()
        left_rear_wheel_position, left_rear_wheel_orientation = self.left_rear_wheel.get_world_pose()
        right_front_wheel_position, right_front_wheel_orientation = self.right_front_wheel.get_world_pose()
        right_rear_wheel_position, right_rear_wheel_orientation = self.right_rear_wheel.get_world_pose()
        # convert quaternion to rotation matrix
        left_front_wheel_orientation = quat_to_rot_matrix(left_front_wheel_orientation)
        left_rear_wheel_orientation = quat_to_rot_matrix(left_rear_wheel_orientation)
        right_front_wheel_orientation = quat_to_rot_matrix(right_front_wheel_orientation)
        right_rear_wheel_orientation = quat_to_rot_matrix(right_rear_wheel_orientation)
        # make 4x4 matrix
        wheel_pose = np.zeros((4, 4, 4))
        wheel_pose[:, 3, 3] = 1
        wheel_pose[0, :3, :3] = left_front_wheel_orientation
        wheel_pose[0, :3, 3] = left_front_wheel_position
        wheel_pose[1, :3, :3] = left_rear_wheel_orientation
        wheel_pose[1, :3, 3] = left_rear_wheel_position
        wheel_pose[2, :3, :3] = right_front_wheel_orientation
        wheel_pose[2, :3, 3] = right_front_wheel_position
        wheel_pose[3, :3, :3] = right_rear_wheel_orientation
        wheel_pose[3, :3, 3] = right_rear_wheel_position
        return wheel_pose
    
    def get_velocities(self)->np.ndarray:
        """
        Return linear and angular velocties of four wheels.
        """
        #TODO: get from rigid prim api
        velocities = np.random.rand(4)
        omega = np.random.rand(4)
        return velocities, omega
    
    def get_sinkages(self)->np.ndarray:
        """
        Return sinkages of four wheels.
        """
        # TODO: get from terrain manager
        sinkages = np.random.rand(4)
        return sinkages

class FourWheelRigidPrimView:
    """
    FourWheelView is a class that provides a view of the four wheels of a vehicle.
    """
    def __init__(self, root_path:str="/Robots"):
        self.root_path = root_path

    def initialize(self, robot_name:str, scene)->None:
        # TODO: vectorize view (use idx to query wheel links instead of making four prims.)
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
    
    def get_net_contact_forces(self)->np.ndarray:
        """
        Returns the net contact forces of the four wheels.
        RigidBodyPrimView.get_net_contact_forces only considers force applied among rigid bodies (rigidbodyAPI is applied.)
        """
        left_front_wheel_contact_force = self.left_front_wheel_view.get_net_contact_forces()[-1]
        left_rear_wheel_contact_force = self.left_rear_wheel_view.get_net_contact_forces()[-1]
        right_front_wheel_contact_force = self.right_front_wheel_view.get_net_contact_forces()[-1]
        right_rear_wheel_contact_force = self.right_rear_wheel_view.get_net_contact_forces()[-1]
        return np.stack([left_front_wheel_contact_force, left_rear_wheel_contact_force, right_front_wheel_contact_force, right_rear_wheel_contact_force], axis=0)
    
    def apply_force_torque(self, forces:np.ndarray, torques:np.ndarray)->None:
        """
        Apply force and torque (defined in local body frame) to body frame of the four wheels.
        Args:
            forces (np.ndarray): The forces to apply to the body origin of the four wheels.
                                 (Fx, Fy, Fz) = (F_DP, F_S, F_N)
            torques (np.ndarray): The torques to apply to the body origin of the four wheels.
                                 (Mx, My, Mz0 = (M_O,-M_R, M_S)
        """
        assert len(forces) == 4
        assert len(torques) == 4
        self.left_front_wheel_view.apply_forces_and_torques_at_pos(forces=forces[0], torques=torques[0], is_global=False)
        self.left_rear_wheel_view.apply_forces_and_torques_at_pos(forces=forces[1], torques=torques[1], is_global=False)
        self.right_front_wheel_view.apply_forces_and_torques_at_pos(forces=forces[2], torques=torques[2], is_global=False)
        self.right_rear_wheel_view.apply_forces_and_torques_at_pos(forces=forces[3], torques=torques[3], is_global=False)