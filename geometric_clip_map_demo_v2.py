import omni
import math
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})


if __name__ == "__main__":
    from omni.isaac.core import World
    from omni.usd import get_context
    from pxr import UsdLux
    from WorldBuilders.pxr_utils import setDefaultOps, addDefaultOps
    import numpy as np
    from src.terrain_management.geometric_clipmaps_manager_v2 import (
        GeoClipmapManager,
        GeoClipmapManagerConf,
    )

    # from src.terrain_management.map_manager import MapManager, MapManagerCfg, DemInfo
    # from src.terrain_management.high_res_dem_gen import HighResDEMGenCfg

    cfg = GeoClipmapManagerConf()

    world = World(stage_units_in_meters=1.0)
    stage = get_context().get_stage()

    # Let there be light
    light = UsdLux.DistantLight.Define(stage, "/sun")
    light.CreateIntensityAttr(3000.0)
    addDefaultOps(light.GetPrim())
    setDefaultOps(light.GetPrim(), (0, 0, 0), (0.383, 0, 0, 0.924), (1, 1, 1))

    # HRDEMGCfg_D = {
    #    "num_blocks": 7,  # int = dataclasses.field(default_factory=int)
    #    "block_size": 50,  # float = dataclasses.field(default_factory=float)
    #    "pad_size": 10.0,  # float = dataclasses.field(default
    #    "max_blocks": int(1e7),  # int = dataclasses.field(default_factory=int)
    #    "seed": 42,  # int = dataclasses.field(default_factory=int)
    #    "resolution": 0.05,  # float = dataclasses.field(default_factory=float)
    #    "z_scale": 1.0,  # float = dataclasses.field(default_factory=float)
    #    "radius": [
    #        [1.5, 2.5],
    #        [0.75, 1.5],
    #        [0.25, 0.5],
    #    ],  # List[Tuple[float, float]] = dataclasses.field(default_factory=list)
    #    "densities": [
    #        0.025,
    #        0.05,
    #        0.5,
    #    ],  # List[float] = dataclasses.field(default_factory=list)
    #    "num_repeat": 1,  # int = dataclasses.field(default_factory=int)
    #    "save_to_disk": False,  # bool = dataclasses.field(default_factory=bool)
    #    "write_to_disk_interval": 100,  # int = dataclasses.field(default_factory=int)
    #    "profiles_path": "assets/Terrains/crater_spline_profiles.pkl",  # str = dataclasses.field(default_factory=str)
    #    "min_xy_ratio": 0.85,  # float = dataclasses.field(default_factory=float)
    #    "max_xy_ratio": 1.0,  # float = dataclasses.field(default_factory=float)
    #    "random_rotation": True,  # bool = dataclasses.field(default_factory=bool)
    #    "num_unique_profiles": 10000,  # int = dataclasses.field(default_factory=int)
    #    "source_resolution": 5.0,  # float = dataclasses.field(default_factory=float)
    #    # "target_resolution": 0.05,  # float = dataclasses.field(default_factory=float)
    #    "interpolation_padding": 2,  # int = dataclasses.field(default_factory=int)
    #    "interpolation_method": "bicubic",  # str = dataclasses.field(default_factory=str)
    # }

    # hrdem_settings = HighResDEMGenCfg(**HRDEMGCfg_D)

    # MMCfg_D = {
    #    "folder_path": "assets/Terrains/SouthPole",
    #    "lr_dem_name": "crater",
    #    "initial_pixel_coordinates": (2000, 2000),
    # }

    # mm_settings = MapManagerCfg(**MMCfg_D)

    # MM = MapManager(hrdem_settings, mm_settings)
    # MM.load_lr_dem_by_name("NPD_final_adj_5mpp_surf")
    GCM = GeoClipmapManager(
        cfg, interpolation_method="bilinear", acceleration_mode="hybrid"
    )
    # T = GeoClipmapManager(cfg, interpolation_method="bilinear")
    dem = np.load("assets/Terrains/SouthPole/NPD_final_adj_5mpp_surf/dem.npy")
    GCM.build(dem, dem.shape)
    GCM.updateGeoClipmap(np.array([2000 * 5, 2000 * 5, 0]))

    while True:
        GCM.updateGeoClipmap(np.array([2000 * 5, 2000 * 5, 0]))
        world.step(render=True)
