import omni
import math
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})


if __name__ == "__main__":
    from omni.isaac.core import World
    import numpy as np
    from src.terrain_management.geo_clipmaps_manager import (
        GeoClipmapManager,
        GeoClipmapManagerConf,
    )

    cfg = GeoClipmapManagerConf()

    world = World(stage_units_in_meters=1.0)
    T = GeoClipmapManager(cfg)
    T.updateGeoClipmap(np.array([8192, 0, 8192]))

    R = 2048
    C = 8192
    theta = np.linspace(0, 2 * np.pi, 256)

    i = 0
    while True:
        for _ in range(2):
            world.step(render=True)

        x = math.cos(theta[i]) * R + C
        y = math.sin(theta[i]) * R + C
        T.updateGeoClipmap(np.array([x, 0, y]))
        i = (i + 1) % 256
