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
    from src.terrain_management.geometric_clipmaps_manager_gpu import (
        GeoClipmapManager,
        GeoClipmapManagerConf,
    )

    cfg = GeoClipmapManagerConf()

    world = World(stage_units_in_meters=1.0)
    stage = get_context().get_stage()

    # Let there be light
    light = UsdLux.DistantLight.Define(stage, "/sun")
    light.CreateIntensityAttr(3000.0)
    addDefaultOps(light.GetPrim())
    setDefaultOps(light.GetPrim(), (0, 0, 0), (0.383, 0, 0, 0.924), (1, 1, 1))

    T = GeoClipmapManager(cfg)
    T.updateGeoClipmap(np.array([2000 * 5, 2000 * 5, 0]))

    R = 500 * 5
    C = 2000 * 5
    theta = np.linspace(0, 2 * np.pi, 256)

    i = 0
    while True:
        for _ in range(2):
            world.step(render=True)

        x = math.cos(theta[i]) * R + C
        y = math.sin(theta[i]) * R + C
        T.updateGeoClipmap(np.array([x, y, 0]))
        i = (i + 1) % 256
