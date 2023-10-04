import omni
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})


if __name__ == "__main__":
    from omni.isaac.core import World
    import numpy as np
    from geo_clipmaps_manager import GeoClipmapManager, GeoClipmapManagerConf
    cfg = GeoClipmapManagerConf()

    world = World(stage_units_in_meters=1.0)
    T =  GeoClipmapManager(cfg)
    T.updateGeoClipmap(np.array([8192,0,8192]))

    while(True):
        world.step(render=True)
        