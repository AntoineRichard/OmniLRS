from typing import Tuple
import dataclasses
import numpy as np
import math
import copy

from src.terrain_management.large_scale_terrain.map_manager import (
    MapManagerCfg,
    MapManager,
)
from src.terrain_management.large_scale_terrain.high_resolution_DEM_generator import (
    HighResDEMGenCfg,
)
from src.terrain_management.large_scale_terrain.geometric_clipmaps_manager import (
    GeoClipmapManagerConf,
    GeoClipmapManager,
)
from src.terrain_management.large_scale_terrain.geometric_clipmaps import (
    GeoClipmapSpecs,
)
from src.terrain_management.large_scale_terrain.rock_manager import (
    RockManagerCfg,
    RockManager,
)
from src.terrain_management.large_scale_terrain.rock_distribution import mock_call


@dataclasses.dataclass
class NestedGeometricClipMapManagerCfg:
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


class NestedGeoClipmapManager:
    """
    The nested geometric clipmap manager.
    It manages the fine and coarse clipmaps and makes sure that they are updated correctly.
    """

    def __init__(self, settings: NestedGeometricClipMapManagerCfg) -> None:
        """
        Args:
            settings (NestedGeometricClipMapManagerCfg): The settings for the nested geometric clipmap manager.
        """

        self.settings = settings

    def generate_geometric_clip_maps_configs(
        self,
        hr_dem_shape: Tuple[float, float],
        lr_dem_shape: Tuple[float, float],
        hr_dem_res: float,
        lr_dem_res: float,
    ) -> None:
        """
        Generate the geometric clipmap configurations using the shape and resolution of the DEMs.
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
            math.log(
                hr_dem_width
                / (self.settings.num_texels_per_level * self.settings.target_res)
            )
            / math.log(2)
        )
        coarse_clipmap_res = self.settings.target_res * 2 ** (fine_clipmap_levels - 1)
        coarse_clipmap_levels = int(
            math.log(
                lr_dem_width / (self.settings.num_texels_per_level * coarse_clipmap_res)
            )
            / math.log(2)
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
        Build the nested geometric clipmap manager.

        Args:
            hr_dem (np.ndarray): The high resolution DEM.
            lr_dem (np.ndarray): The low resolution DEM.
            hr_dem_shape (tuple): The shape of the high resolution DEM. (pixels)
            lr_dem_shape (tuple): The shape of the low resolution DEM. (pixels)
            hr_dem_res (float): The resolution of the high resolution DEM. (meters per pixel)
            lr_dem_res (float): The resolution of the low resolution DEM. (meters per pixel)
        """

        self.generate_geometric_clip_maps_configs(
            hr_dem_shape, lr_dem_shape, hr_dem_res, lr_dem_res
        )

        self.fine_clipmap_manager = GeoClipmapManager(
            self.fine_clipmap_manager_cfg,
            interpolation_method=self.settings.fine_interpolation_method,
            acceleration_mode=self.settings.fine_acceleration_mode,
            name_prefix="_fine",
            profiling=self.settings.profiling,
        )
        self.coarse_clipmap_manager = GeoClipmapManager(
            self.coarse_clipmap_manager_cfg,
            interpolation_method=self.settings.coarse_interpolation_method,
            acceleration_mode=self.settings.coarse_acceleration_mode,
            name_prefix="_coarse",
            profiling=self.settings.profiling,
        )
        self.fine_clipmap_manager.build(hr_dem, hr_dem_shape)
        self.coarse_clipmap_manager.build(lr_dem, lr_dem_shape)

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
            mesh_position (Tuple[float, float]): The position of the geometric clipmaps' meshes in the world frame. (meters)
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

        return self.fine_clipmap_manager.get_height_and_random_orientation(
            x, y, map_coordinates, seed=seed
        )


def is_map_done():
    return True


@dataclasses.dataclass
class LargeScaleTerrainManagerCfg:
    map_name: str = dataclasses.field(default_factory=str)
    pixel_coordinates: tuple = dataclasses.field(default_factory=tuple)
    ll_coordinates: tuple = dataclasses.field(default_factory=tuple)
    meters_coordinates: tuple = dataclasses.field(default_factory=tuple)
    coordinate_format: str = "meters"
    visual_mesh_update_threshold: float = 2.0


class LargeScaleTerrainManager:
    def __init__(
        self,
        settings: LargeScaleTerrainManagerCfg,
        nested_geometric_clipmap_manager_cfg: NestedGeometricClipMapManagerCfg,
        highresdemgen_cfg: HighResDEMGenCfg,
        mapmanager_cfg: MapManagerCfg,
        rock_manager_cfg: RockManagerCfg,
    ):
        self.nested_geometric_clipmap_manager_cfg = nested_geometric_clipmap_manager_cfg
        self.highresdemgen_cfg = highresdemgen_cfg
        self.mapmanager_cfg = mapmanager_cfg
        self.rock_manager_cfg = rock_manager_cfg
        self.settings = settings

        self.last_update_coordinates = None

    def cast_coordinates(self):
        """
        Cast the coordinates to the desired format.

        The coordinates are cast to meters with 0 at the center of the map.
        """

        # Cast coordinates to meters with 0 at the center of the map
        if self.settings.coordinate_format == "meters":
            self.initial_coordinates = self.settings.meters_coordinates
        elif self.settings.coordinate_format == "ll":
            raise NotImplementedError
        elif self.settings.coordinate_format == "pixels":
            lr_dem_shape = self.map_manager.get_lr_dem_shape()
            lr_dem_res = self.map_manager.get_lr_dem_res()
            self.initial_coordinates = (
                (self.settings.pixel_coordinates[0] - lr_dem_shape // 2) / lr_dem_res,
                (self.settings.pixel_coordinates[1] - lr_dem_shape // 2) / lr_dem_res,
            )

    def get_height_local(self, coordinates: Tuple[float, float]) -> float:
        """
        Get the height at the given coordinates in the local map.

        Args:
            coordinates (Tuple[float, float]): The coordinates in the local map.

        Returns:
            float: The height at the given coordinates.
        """

        global_coordinates = (
            coordinates[0] + self.initial_coordinates[0],
            coordinates[1] + self.initial_coordinates[1],
        )
        return self.map_manager.get_height(global_coordinates)

    def get_height_global(self, coordinates: Tuple[float, float]) -> float:
        """
        Get the height at the given coordinates in the global map.

        Args:
            coordinates (Tuple[float, float]): The coordinates in the global map.

        Returns:
            float: The height at the given coordinates.
        """

        return self.map_manager.get_height(coordinates)

    def get_normal_local(self, coordinates: Tuple[float, float]) -> np.ndarray:
        """
        Get the normal at the given coordinates in the local map.

        Args:
            coordinates (Tuple[float, float]): The coordinates in the local map.

        Returns:
            np.ndarray: The normal to the DEM surface at the given coordinates.
        """

        global_coordinates = (
            coordinates[0] + self.initial_coordinates[0],
            coordinates[1] + self.initial_coordinates[1],
        )
        return self.map_manager.get_normal(global_coordinates)

    def get_normal_global(self, coordinates: Tuple[float, float]) -> np.ndarray:
        """
        Get the normal at the given coordinates in the global map.

        Args:
            coordinates (Tuple[float, float]): The coordinates in the global map.

        Returns:
            np.ndarray: The normal to the DEM surface at the given coordinates.
        """

        return self.map_manager.get_normal(coordinates)

    def build(self):
        self.map_manager = MapManager(self.highresdemgen_cfg, self.mapmanager_cfg)
        self.nested_clipmap_manager = NestedGeoClipmapManager(
            self.nested_geometric_clipmap_manager_cfg
        )
        # self.rock_manager = RockManager(
        #    self.rock_manager_cfg,
        #    mock_call,
        #    is_map_done,
        # )
        self.rock_manager = RockManager(
            self.rock_manager_cfg,
            self.nested_clipmap_manager.get_height_and_random_scale,
            is_map_done,
        )

        self.map_manager.load_lr_dem_by_name(self.settings.map_name)
        self.cast_coordinates()
        self.mesh_position = (0, 0)
        self.map_manager.initialize_hr_dem(self.initial_coordinates)
        self.nested_clipmap_manager.build(
            self.map_manager.get_hr_dem(),
            self.map_manager.get_lr_dem(),
            self.map_manager.get_hr_dem_shape(),
            self.map_manager.get_lr_dem_shape(),
            self.map_manager.get_hr_dem_res(),
            self.map_manager.get_lr_dem_res(),
        )
        self.rock_manager.build()
        self.update_visual_mesh((0, 0))

    def update_visual_mesh(self, local_coordinates):
        # print("local_coordinates", local_coordinates)
        # print("last_update_coordinates", self.last_update_coordinates)
        if self.last_update_coordinates is None:
            self.last_update_coordinates = copy.copy(local_coordinates)
            delta = (0.0, 0.0)
            dist = self.settings.visual_mesh_update_threshold * 2
        else:
            # local coordinates are in meters
            delta = (
                local_coordinates[0] - self.last_update_coordinates[0],
                local_coordinates[1] - self.last_update_coordinates[1],
            )
            dist = math.sqrt(delta[0] ** 2 + delta[1] ** 2)
        if dist > self.settings.visual_mesh_update_threshold:
            # cast the coordinates so that they are a multiple of the threshold.
            x = (
                local_coordinates[0] // self.settings.visual_mesh_update_threshold
            ) * self.settings.visual_mesh_update_threshold
            y = (
                local_coordinates[1] // self.settings.visual_mesh_update_threshold
            ) * self.settings.visual_mesh_update_threshold
            corrected_coordinates = (x, y)
            # Update the visual mesh
            self.last_update_coordinates = local_coordinates

            # Get the global coordinates. The mesh initial position is in (0,0).
            # Thus, we need to add the initial coordinates to the local coordinates.
            global_coordinates = (
                corrected_coordinates[0] + self.initial_coordinates[0],
                corrected_coordinates[1] + self.initial_coordinates[1],
            )
            # print("initial_coordinates", self.initial_coordinates)
            # print("global_coordinates", global_coordinates)

            # Update the high resolution DEM
            # hr_dem_updated = self.map_manager.update_hr_dem(global_coordinates)
            self.map_manager.hr_dem_gen.update_terrain_data_blocking(global_coordinates)
            # if the DEM was updated, the high DEM inside the warp buffer of the nested clipmap manager
            # needs to be updated as well.

            # self.mesh_position = (
            #    self.mesh_position[0] + delta[0],
            #    self.mesh_position[1] + delta[1],
            # )
            print("terrain updated")
            fine_position = self.map_manager.get_hr_coordinates(global_coordinates)
            coarse_position = self.map_manager.get_lr_coordinates(global_coordinates)
            print("coordinates aquired")
            # print("fine_position", fine_position)
            # print("coarse_position", coarse_position)
            # print("min value lr dem", self.map_manager.get_lr_dem().min())
            # print("max value lr dem", self.map_manager.get_lr_dem().max())
            # print("min value hr dem", self.map_manager.get_hr_dem().min())
            # print("max value hr dem", self.map_manager.get_hr_dem().max())

            self.nested_clipmap_manager.update_clipmaps(
                fine_position, coarse_position, corrected_coordinates
            )
            print("clipmaps updated")
            self.rock_manager.sample(corrected_coordinates)
            print("rock manager sampled")
            update = True
        else:
            corrected_coordinates = (0, 0)
            update = False

        return update, corrected_coordinates
