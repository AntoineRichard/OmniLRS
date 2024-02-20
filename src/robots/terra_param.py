from dataclasses import dataclass
import numpy as np

@dataclass
class RobotParameter:
    """
    Robot mechanical dimension parameters."""
    mass: float = 20.0
    wheel_radius: float = 0.09
    wheel_base: float = 0.1
    wheel_lug_height: float = 0.02

@dataclass
class TerrainMechanicalParameter:
    """
    Terrain mechanical parameters.
    - bearing_parameter: (k_c, k_phi, n_0, n_1) bearing parameters.
    - shearing_parameter: (c, phi, K) shearing parameters."""
    bearing_parameter: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0])
    shearing_parameter: np.ndarray = np.array([0.0, 0.0, 0.0])