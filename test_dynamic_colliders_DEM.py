import omni
import math
from omni.isaac.kit import SimulationApp


if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": False})

    from omni.isaac.core import World
    from WorldBuilders import pxr_utils as pu
    import math
    import numpy as np
    from pxr import UsdLux, UsdGeom
    from src.terrain_management.colliders_manager import CollidersManager
    from src.configurations.procedural_terrain_confs import (
        CollidersManagerConf,
    )
    

    CMC = CollidersManagerConf(
        root_path="/Main",
        sim_length=50.0,
        sim_width=50.0,
        resolution=0.1,
        build_radius=1.5,
        remove_radius=2.5,
        max_cache_size=6,
    )

    CM = CollidersManager(cfg=CMC)

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    # We are loading this DEM and using it as a collider.
    # The actual resolution of the DEM is 5m/pixel, but we are using it as if it had a resolution of 0.1m/pixel.
    DEM = np.load("assets/Terrains/SouthPole/LM1_final_adj_5mpp_surf/dem.npy") * (0.1/5.0)
    np.flip(DEM, axis=0) # Flip Y axis to match the real world.
    DEM_offset = (DEM.shape[0]//2, DEM.shape[1]//2)

    print("max:", np.max(DEM))
    print("min:", np.min(DEM))

    CM.update_DEM_data(DEM, DEM_offset)

    UsdLux.DistantLight.Define(stage, "/sun")
    UsdGeom.Sphere.Define(stage, "/sphere")
    sphere = stage.GetPrimAtPath("/sphere")
    pu.addDefaultOps(sphere)
    i = 0
    r = 60
    w = 0.001
    while True:
        world.step(render=True)
        pu.setDefaultOps(sphere, (math.cos(i*w)*r, math.sin(i*w)*r, 1.), (0.,0.,0.,1.), (1.,1.,1.))
        i += 1
        CM.update_blocks((math.cos(i*w)*r, math.sin(i*w)*r, 1.))
