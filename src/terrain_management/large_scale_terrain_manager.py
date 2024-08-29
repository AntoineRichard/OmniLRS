from typing import Tuple
import dataclasses
import numpy as np
import math
import copy


from src.terrain_management.large_scale_terrain.map_manager import (
    MapManagerCfg,
    MapManager,
)
from src.terrain_management.large_scale_terrain.rock_manager import (
    RockManagerCfg,
    RockManager,
)
from src.terrain_management.large_scale_terrain.nested_geometry_clipmaps_manager import (
    NestedGeometryClipmapManagerCfg,
    NestedGeometryClipmapManager,
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
        nested_geometric_clipmap_manager_cfg: NestedGeometryClipmapManagerCfg,
        mapmanager_cfg: MapManagerCfg,
        rock_manager_cfg: RockManagerCfg,
    ):
        self.nested_geometric_clipmap_manager_cfg = nested_geometric_clipmap_manager_cfg
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
        self.map_manager = MapManager(self.mapmanager_cfg)
        self.nested_clipmap_manager = NestedGeometryClipmapManager(self.nested_geometric_clipmap_manager_cfg)
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

            self.nested_clipmap_manager.update_clipmaps(fine_position, coarse_position, corrected_coordinates)
            print("clipmaps updated")
            self.rock_manager.sample(corrected_coordinates)
            print("rock manager sampled")
            update = True
        else:
            corrected_coordinates = (0, 0)
            update = False

        return update, corrected_coordinates
