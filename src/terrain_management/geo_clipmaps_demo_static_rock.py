import omni
from omni.isaac.kit import SimulationApp

from typing import Dict, List
import hydra
from omegaconf import DictConfig, OmegaConf, ListConfig
from src.configurations import configFactory
import os

simulation_app = SimulationApp({"headless": False})


def resolve_tuple(*args):
    return tuple(args)
OmegaConf.register_new_resolver("as_tuple", resolve_tuple)


def omegaconfToDict(d: DictConfig) -> Dict:
    """Converts an omegaconf DictConfig to a python Dict, respecting variable interpolation.

    Args:
        d (DictConfig): OmegaConf DictConfig.

    Returns:
        Dict: Python dict."""

    if isinstance(d, DictConfig):
        ret = {}
        for k, v in d.items():
            if isinstance(v, DictConfig):
                ret[k] = omegaconfToDict(v)
            elif isinstance(v, ListConfig):
                ret[k] = [omegaconfToDict(i) for i in v]
            else:
                ret[k] = v
    elif isinstance(d, ListConfig):
        ret = [omegaconfToDict(i) for i in d]
    else:
        ret = d

    return ret


def instantiateConfigs(cfg: dict) -> dict:
    """"""

    instantiable_configs = configFactory.getConfigs()

    ret = {}
    for k, v in cfg.items():
        if isinstance(v, dict):
            if k in instantiable_configs:
                ret[k] = configFactory(k, **v)
            else:
                ret[k] = instantiateConfigs(v)
        else:
            ret[k] = v
    return ret

@hydra.main(config_name="config", config_path="../../cfg")
def run(cfg:DictConfig):
    from omni.isaac.core import World
    import numpy as np
    from geo_clipmaps_static import GeoClipmapSpecs
    from geo_clipmaps_manager_static import GeoClipmapManager, GeoClipmapManagerConf
    from src.environments.rock_manager import RockManager
    from omni.isaac.core.utils.stage import get_current_stage
    from pxr import UsdGeom

    # build terrain
    terrain_root = "assets/Terrains/SouthPole/Site_All"

    geo_spec = GeoClipmapSpecs(
        numMeshLODLevels= 7,
        demPath=os.path.join(terrain_root, "dem.npy"), 
    )
    clipmap_cfg = GeoClipmapManagerConf(geo_clipmap_specs=geo_spec)

    T = GeoClipmapManager(clipmap_cfg)
    T.updateGeoClipmap(np.array([20000*5, 0, 20000*5]))

    # spawn rocks
    rock_cfg = cfg["environment"]["rocks_settings"]
    rock_cfg = omegaconfToDict(rock_cfg)
    rock_cfg = instantiateConfigs(rock_cfg)
    RM = RockManager(**rock_cfg)
    
    dem = np.load(os.path.join(terrain_root, "dem.npy"))
    mask = np.load(os.path.join(terrain_root, "mask.npy"))
    RM.build(dem, mask)
    RM.updateImageData(dem, mask)
    RM.randomizeInstancers(2844)

    world = World(stage_units_in_meters=1.0)
    stage = get_current_stage()
    light_prim = stage.DefinePrim(cfg["environment"]["lsp_settings"]["projector_path"], "DistantLight")
    UsdGeom.Xformable(light_prim).AddTranslateOp()
    UsdGeom.Xformable(light_prim).AddRotateXYZOp()
    light_prim.GetAttribute('xformOp:translate').Set((0, 0, 1.0))
    light_prim.GetAttribute('xformOp:rotateXYZ').Set((80, 0, 0))

    while True:
        world.step(render=True)
if __name__ == "__main__":
    run()