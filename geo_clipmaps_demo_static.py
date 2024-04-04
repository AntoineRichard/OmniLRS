import omni
import math
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})


if __name__ == "__main__":
    from omni.isaac.core import World
    import numpy as np
    from src.terrain_management.geo_clipmaps_manager_static import (
        GeoClipmapManager,
        GeoClipmapManagerConf,
    )

    cfg = GeoClipmapManagerConf()

    world = World(stage_units_in_meters=1.0)
    T = GeoClipmapManager(cfg)
    T.updateGeoClipmap(np.array([20000 * 5, 0, 20000 * 5]))

    while True:
        world.step(render=True)
