import math

from src.terrain_management.map_manager import MapManagerCfg, MapManager
from src.terrain_management.high_res_dem_gen import HighResDEMGenCfg, HighResDEMGen
from src.terrain_management.geometric_clipmaps_manager_v2 import (
    GeoClipmapManagerConf,
    GeoClipmapManager,
)
from src.terrain_management.geometric_clipmaps_v2 import GeoClipmapSpecs, GeoClipmap

# This will have 2 clipmaps and a flat terrain.
# The first clipmap is a single level high resolution clipmap. Augments the high res-dem.
# The second clipmap is a multi-level clipmap with coarser resolution. It feeds off the high-res dem generator.


class NestedGeometricClipMapManagerCfg:
    num_texels_per_level: int = 256
    target_res: float = 0.01
    fine_interpolation_method: str = "bicubic"
    coarse_interpolation_method: str = "bilinear"
    fine_acceleration_mode: str = "hybrid"
    coarse_acceleration_mode: str = "hybrid"
    lr_dem_res: float = 5.0
    hr_dem_res: float = 0.01
    lr_dem_shape: tuple = (4000, 4000)
    hr_dem_shape: tuple = (15000, 15000)


class NestedGeoClipmapManager:
    def __init__(self, settings: NestedGeometricClipMapManagerCfg):
        self.settings = settings

    def generate_geometric_clip_maps_configs(self):
        hr_dem_width = (
            max(self.settings.hr_dem_shape[0], self.settings.hr_dem_shape[1])
            * self.settings.hr_dem_res
        )
        lr_dem_width = (
            max(self.settings.lr_dem_shape[0], self.settings.lr_dem_shape[1])
            * self.settings.lr_dem_res
        )
        fine_clipmap_levels = int(
            math.log(
                hr_dem_width
                / (self.settings.num_texels_per_level * self.settings.target_res)
            )
            / math.log(2)
        )
        coarse_clipmap_res = self.settings.target_res * 2**fine_clipmap_levels
        coarse_clipmap_levels = int(
            math.log(
                lr_dem_width / (self.settings.num_texels_per_level * coarse_clipmap_res)
            )
            / math.log(2)
        )

        self.fine_clipmap_specs = GeoClipmapSpecs(
            startingLODLevel=0,
            numMeshLODLevels=fine_clipmap_levels,
            numTexelsPerLevel=self.settings.num_texels_per_level,
            meshBackBonePath="mesh_backbone_fine.npy",
            source_resolution=self.settings.hr_dem_res,
            minimum_target_resolution=self.settings.target_res,
        )
        self.coarse_clipmap_specs = GeoClipmapSpecs(
            startingLODLevel=1,
            numMeshLODLevels=coarse_clipmap_levels,
            numTexelsPerLevel=self.settings.num_texels_per_level,
            meshBackBonePath="mesh_backbone_coarse.npy",
            source_resolution=self.settings.lr_dem_res,
            minimum_target_resolution=coarse_clipmap_res,
        )

    def build_geometric_clip_maps(self, hr_dem, lr_dem, hr_dem_shape, lr_dem_shape):
        self.fine_clipmap_manager = GeoClipmapManager(
            self.fine_clipmap_specs,
            interpolation_method=self.settings.fine_interpolation_method,
            acceleration_mode=self.settings.fine_acceleration_mode,
        )
        self.coarse_clipmap_manager = GeoClipmapManager(
            self.coarse_clipmap_specs,
            interpolation_method=self.settings.coarse_interpolation_method,
            acceleration_mode=self.settings.coarse_acceleration_mode,
        )
        self.fine_clipmap_manager.build(hr_dem, hr_dem_shape)
        self.coarse_clipmap_manager.build(lr_dem, lr_dem_shape)

    def build(self):
        self.generate_geometric_clip_maps_configs()
        self.build_geometric_clip_maps()

    def update_clipmaps(self, position_fine, position_coarse, mesh_position):
        self.fine_clipmap_manager.updateGeoClipmap(position_fine, mesh_position)
        self.coarse_clipmap_manager.updateGeoClipmap(position_coarse, mesh_position)


class LargeScaleTerrainManager:
    def __init__(self):
        self.map_manager = MapManager()
