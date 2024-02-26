__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple, Union, Dict
import omni.kit.actions.core
import numpy as np
import dataclasses
import omni
import carb
import cv2
import os

from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage
from pxr import UsdGeom, Sdf, UsdLux, Gf, UsdShade, Usd

from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.environments.lunalab import LunalabController
from src.terrain_management.terrain_manager import TerrainManager
from src.configurations.environments import LunalabConf
from src.configurations.rendering_confs import FlaresConf
from src.environments.rock_manager import RockManager
from WorldBuilders.pxr_utils import setDefaultOps
from assets import get_assets_path


class LunalabDeformableController(LunalabController):
    """
    This class is used to control the lab interactive elements."""

    def __init__(
        self,
        lunalab_settings: LunalabConf = None,
        rocks_settings: Dict = None,
        flares_settings: FlaresConf = None,
        terrain_manager: TerrainManagerConf = None,
        **kwargs,
    ) -> None:
        """
        Initializes the lab controller. This class is used to control the lab interactive elements.
        Including:
            - Projector position, intensity, radius, color.
            - Room lights intensity, radius, color.
            - Curtains open or closed.
            - Terrains randomization or using premade DEMs.
            - Rocks random placements.

        Args:
            lunalab_settings (LunalabLabConf): The settings of the lab.
            rocks_settings (Dict): The settings of the rocks.
            flares_settings (FlaresConf): The settings of the flares.
            terrain_manager (TerrainManagerConf): The settings of the terrain manager.
            **kwargs: Arbitrary keyword arguments."""
            
        super().__init__(lunalab_settings, rocks_settings, flares_settings, terrain_manager, **kwargs)
        self.terrain_managet_conf = terrain_manager
    
    def deformTerrain(self, world_poses) -> None:
        num_forces = world_poses.shape[0]
        contact_forces = np.zeros((num_forces, 3))
        contact_forces[:, 2] = np.ones(num_forces) * self.terrain_managet_conf.moon_yard.deformation_engine.static_normal_force
        contact_forces = self.get_contact_forces_in_world(contact_forces, world_poses)
        self.T.deformTerrain(body_transforms=world_poses, contact_forces=contact_forces)
        self.loadDEM()
        self.RM.updateImageData(self.dem, self.mask)


    @staticmethod
    def get_contact_forces_in_world(contact_forces_local:np.ndarray, world_poses:np.ndarray)->np.ndarray:
        """
        Returns the contact forces in world frame.

        Args:
            contact_forces_local (np.ndarray): The contact forces in local frame.
            world_poses (np.ndarray): The world poses of the contact points.

        Returns:
            np.ndarray: The contact forces in world frame.
        """
        return np.matmul(world_poses[:, :3, :3], contact_forces_local[:, :, None]).squeeze()