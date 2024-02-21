import numpy as np
import torch

from src.physics.terra_param import RobotParameter, TerrainMechanicalParameter

class TerramechanicsSolver:
    """
    Terramechanics solver class. This class computes the force and torque acting on the robot as a disturbance.
    Fx: Draw bar pull
    Fy: Side slip force
    Fz: Normal force
    Mx: Resistance torque
    My: Overturn torque
    Mz: Steering resistance torque"""
    def __init__(self, robot_param: RobotParameter=None, terrain_param: TerrainMechanicalParameter=None):
        self.robot_param = robot_param
        self.terrain_param = terrain_param
        self.num_wheels = robot_param.num_wheels
    
    def compute_force_and_torque(self)->np.ndarray:
        """
        Computes the force and torque."""
        force = 10 * np.random.rand(self.num_wheels, 3)
        torque = 10 * np.random.rand(self.num_wheels, 3)
        return force, torque