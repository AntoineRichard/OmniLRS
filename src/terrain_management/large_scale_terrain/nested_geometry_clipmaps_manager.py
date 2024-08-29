__author__ = "Antoine Richard"
__copyright__ = "Copyright 2024, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import Tuple
import dataclasses
import numpy as np
import math

import omni

from src.terrain_management.large_scale_terrain.geometry_clipmaps_manager import (
    GeoClipmapManagerConf,
    GeoClipmapManager,
)
from src.terrain_management.large_scale_terrain.geometry_clipmaps import (
    GeoClipmapSpecs,
)
from src.terrain_management.large_scale_terrain.pxr_utils import bindMaterial, loadMaterial


@dataclasses.dataclass
class NestedGeometryClipmapManagerCfg:
    """
    Args:
        num_texels_per_level (int): The number of texels per level.
        target_res (float): The target resolution. (in meters)
        fine_interpolation_method (str): The interpolation method for the fine clipmap. (bicubic or bilinear)
        coarse_interpolation_method (str): The interpolation method for the coarse clipmap. (bicubic or bilinear)
        fine_acceleration_mode (str): The acceleration mode for the fine clipmap. (hybrid or gpu)
        coarse_acceleration_mode (str): The acceleration mode for the coarse clipmap. (hybrid or gpu)
        profiling (bool): Whether to profile the clipmap manager.
    """

    num_texels_per_level: int = 256
    target_res: float = 0.01
    fine_interpolation_method: str = "bicubic"
    coarse_interpolation_method: str = "bilinear"
    fine_acceleration_mode: str = "hybrid"
    coarse_acceleration_mode: str = "hybrid"
    profiling: bool = False
    texture_name: str = "LunarRegolith8k"
    texture_path: str = "assets/Textures/LunarRegolith8k.mdl"


class NestedGeometryClipmapManager:
    """
    The nested geometry clipmap manager.
    It manages the fine and coarse clipmaps and makes sure that they are updated correctly.
    """

    def __init__(self, settings: NestedGeometryClipmapManagerCfg) -> None:
        """
        Args:
            settings (NestedGeometryClipMapManagerCfg): The settings for the nested geometry clipmap manager.
        """

        self.settings = settings
        self.stage = omni.usd.get_context().get_stage()

    def generate_geometry_clip_maps_configs(
        self,
        hr_dem_shape: Tuple[float, float],
        lr_dem_shape: Tuple[float, float],
        hr_dem_res: float,
        lr_dem_res: float,
    ) -> None:
        """
        Generate the geometry clipmap configurations using the shape and resolution of the DEMs.
        This code will ensure that the clipmaps are using the full extent of the DEMs.

        Args:
            hr_dem_shape (tuple): The shape of the high resolution DEM. (pixels)
            lr_dem_shape (tuple): The shape of the low resolution DEM. (pixels)
            hr_dem_res (float): The resolution of the high resolution DEM. (meters per pixel)
            lr_dem_res (float): The resolution of the low resolution DEM. (meters per pixel)
        """

        hr_dem_width = max(hr_dem_shape[0], hr_dem_shape[1]) * hr_dem_res
        lr_dem_width = max(lr_dem_shape[0], lr_dem_shape[1]) * lr_dem_res

        fine_clipmap_levels = int(
            math.log(hr_dem_width / (self.settings.num_texels_per_level * self.settings.target_res)) / math.log(2)
        )
        coarse_clipmap_res = self.settings.target_res * 2 ** (fine_clipmap_levels - 1)
        coarse_clipmap_levels = int(
            math.log(lr_dem_width / (self.settings.num_texels_per_level * coarse_clipmap_res)) / math.log(2)
        )

        self.fine_clipmap_specs = GeoClipmapSpecs(
            startingLODLevel=0,
            numMeshLODLevels=fine_clipmap_levels,
            meshBaseLODExtentHeightfieldTexels=self.settings.num_texels_per_level,
            meshBackBonePath="mesh_backbone_fine.npz",
            source_resolution=hr_dem_res,
            minimum_target_resolution=self.settings.target_res,
        )
        self.coarse_clipmap_specs = GeoClipmapSpecs(
            startingLODLevel=1,
            numMeshLODLevels=coarse_clipmap_levels + 1,
            meshBaseLODExtentHeightfieldTexels=self.settings.num_texels_per_level,
            meshBackBonePath="mesh_backbone_coarse.npz",
            source_resolution=lr_dem_res,
            minimum_target_resolution=coarse_clipmap_res,
        )
        self.fine_clipmap_manager_cfg = GeoClipmapManagerConf(
            root_path="/World",
            geo_clipmap_specs=self.fine_clipmap_specs,
            mesh_position=np.array([0, 0, 0]),
            mesh_orientation=np.array([0, 0, 0, 1]),
            mesh_scale=np.array([1, 1, 1]),
        )
        self.coarse_clipmap_manager_cfg = GeoClipmapManagerConf(
            root_path="/World",
            geo_clipmap_specs=self.coarse_clipmap_specs,
            mesh_position=np.array([0, 0, 0]),
            mesh_orientation=np.array([0, 0, 0, 1]),
            mesh_scale=np.array([1, 1, 1]),
        )

    def build(
        self,
        hr_dem: np.ndarray,
        lr_dem: np.ndarray,
        hr_dem_shape: Tuple[float, float],
        lr_dem_shape: Tuple[float, float],
        hr_dem_res: float,
        lr_dem_res: float,
    ) -> None:
        """
        Build the nested geometry clipmap manager.

        Args:
            hr_dem (np.ndarray): The high resolution DEM.
            lr_dem (np.ndarray): The low resolution DEM.
            hr_dem_shape (tuple): The shape of the high resolution DEM. (pixels)
            lr_dem_shape (tuple): The shape of the low resolution DEM. (pixels)
            hr_dem_res (float): The resolution of the high resolution DEM. (meters per pixel)
            lr_dem_res (float): The resolution of the low resolution DEM. (meters per pixel)
        """

        self.generate_geometry_clip_maps_configs(hr_dem_shape, lr_dem_shape, hr_dem_res, lr_dem_res)

        self.fine_clipmap_manager = GeoClipmapManager(
            self.fine_clipmap_manager_cfg,
            interpolation_method=self.settings.fine_interpolation_method,
            acceleration_mode=self.settings.fine_acceleration_mode,
            name_prefix="_fine",
            profiling=self.settings.profiling,
            stage=self.stage,
        )
        self.coarse_clipmap_manager = GeoClipmapManager(
            self.coarse_clipmap_manager_cfg,
            interpolation_method=self.settings.coarse_interpolation_method,
            acceleration_mode=self.settings.coarse_acceleration_mode,
            name_prefix="_coarse",
            profiling=self.settings.profiling,
            stage=self.stage,
        )
        self.fine_clipmap_manager.build(hr_dem, hr_dem_shape)
        self.coarse_clipmap_manager.build(lr_dem, lr_dem_shape)
        self.load_and_apply_material()

    def update_clipmaps(
        self,
        position_fine: Tuple[float, float],
        position_coarse: Tuple[float, float],
        mesh_position: Tuple[float, float],
    ) -> None:
        """
        Update the clipmaps geometry based on the given positions.

        Args:
            position_fine (Tuple[float, float]): The coordinates of the robot in the fine clipmap frame. (meters)
            position_coarse (Tuple[float, float]): The coordinates of the robot in the coarse clipmap frame. (meters)
            mesh_position (Tuple[float, float]): The position of the geometry clipmaps' meshes in the world frame. (meters)
        """

        position_fine = np.array(position_fine)
        position_coarse = np.array(position_coarse)
        mesh_position = np.array([mesh_position[0], mesh_position[1], 0])
        self.fine_clipmap_manager.updateGeoClipmap(position_fine, mesh_position)
        self.coarse_clipmap_manager.updateGeoClipmap(position_coarse, mesh_position)

    def get_height_and_random_scale(
        self,
        x: np.ndarray,
        y: np.ndarray,
        map_coordinates: Tuple[float, float],
        seed: int = 0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the height and random scale at the given coordinates.

        Args:
            x (np.ndarray): The x coordinates.
            y (np.ndarray): The y coordinates.
            map_coordinates (Tuple[float, float]): The coordinates in the map.
            seed (int): The seed for the random scale.
        """

        return self.fine_clipmap_manager.get_height_and_random_orientation(x, y, map_coordinates, seed=seed)

    def load_and_apply_material(self):
        """
        Load and apply the material to the clipmaps.
        """

        material_path = loadMaterial(self.settings.texture_name, self.settings.texture_path)
        bindMaterial(self.fine_clipmap_manager._stage, material_path, self.fine_clipmap_manager._mesh_path)
        bindMaterial(self.coarse_clipmap_manager._stage, material_path, self.coarse_clipmap_manager._mesh_path)
