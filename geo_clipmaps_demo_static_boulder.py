from typing import Dict, List
import os
import hydra
from omegaconf import DictConfig, OmegaConf
import numpy as np
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from run import omegaconfToDict, instantiateConfigs

@hydra.main(config_name="config", config_path="cfg")
def run(cfg:DictConfig):
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import get_current_stage
    from pxr import UsdGeom
    from src.terrain_management.geo_clipmaps_manager_static import (
        GeoClipmapManager,
        GeoClipmapManagerConf,
    )
    from src.environments.rock_manager import RockManager
    
    world = World(stage_units_in_meters=1.0)
    
    ## Load GeoClipMap ##
    clipmap_cfg = GeoClipmapManagerConf()
    T = GeoClipmapManager(clipmap_cfg)
    T.updateGeoClipmap(np.array([20000 * 5, 0, 20000 * 5]))

    ## Load Boulders ##
    rock_cfg = cfg["environment"]["rocks_settings"]
    rock_cfg = omegaconfToDict(rock_cfg)
    rock_cfg = instantiateConfigs(rock_cfg)
    RM = RockManager(**rock_cfg)
    
    dem = T._geo_clipmap.dem
    mask = np.ones_like(dem)
    RM.build(dem, mask)
    RM.updateImageData(dem, mask)
    RM.randomizeInstancers(2844)
    
    ## Load Light ##
    stage = get_current_stage()
    light_prim = stage.DefinePrim(cfg["environment"]["lsp_settings"]["sun_path"], "DistantLight")
    UsdGeom.Xformable(light_prim).AddTranslateOp()
    UsdGeom.Xformable(light_prim).AddRotateXYZOp()
    light_prim.GetAttribute('xformOp:translate').Set((0.0, 0.0, 0.0))
    light_prim.GetAttribute('xformOp:rotateXYZ').Set((80, 0, 0))

    while True:
        world.step(render=True)
        
if __name__ == "__main__":
    run()