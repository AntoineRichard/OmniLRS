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
    from src.terrain_management.large_scale_terrain.geometry_clipmaps import GeoClipmapSpecs
    from src.terrain_management.large_scale_terrain.geometry_clipmaps_manager import (
        GeoClipmapManager,
        GeoClipmapManagerConf,
    )

    specs = GeoClipmapSpecs(
        startingLODLevel=0,
        numMeshLODLevels=11,
        meshBaseLODExtentHeightfieldTexels=256,
        meshBackBonePath="terrain_mesh_backbone.npz",
        source_resolution=5.0,
        minimum_target_resolution=0.01,
    )

    cfg = GeoClipmapManagerConf(
        root_path="/World",
        geo_clipmap_specs=specs,
        mesh_position=np.array([0, 0, 0]),
        mesh_orientation=np.array([0, 0, 0, 1]),
        mesh_scale=np.array([1, 1, 1]),
    )

    world = World(stage_units_in_meters=1.0)
    stage = get_context().get_stage()

    # Let there be light
    light = UsdLux.DistantLight.Define(stage, "/sun")
    light.CreateIntensityAttr(3000.0)
    addDefaultOps(light.GetPrim())
    setDefaultOps(light.GetPrim(), (0, 0, 0), (0.383, 0, 0, 0.924), (1, 1, 1))

    GCM = GeoClipmapManager(cfg, interpolation_method="bicubic", acceleration_mode="hybrid")
    dem = np.load("assets/Terrains/SouthPole/NPD_final_adj_5mpp_surf/dem.npy")
    GCM.build(dem, dem.shape)
    C = 2000 * 5
    R = 0  # 500 * 5
    rotation_rate = 2048
    spiral_rate = 2.0 / rotation_rate
    theta = np.linspace(0, 2 * np.pi, rotation_rate)

    i = 0
    i2 = 1.0
    while True:
        GCM.updateGeoClipmap(
            np.array([C + i2 * R * math.cos(theta[i]), C + i2 * R * math.sin(theta[i]), 0]),
            np.array([i2 * R * math.cos(theta[i]), i2 * R * math.sin(theta[i]), 0]),
        )
        world.step(render=True)
        i = (i + 1) % rotation_rate
        i2 += spiral_rate
