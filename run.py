__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from omegaconf import DictConfig, OmegaConf, ListConfig
from src.configurations import configFactory
from src.environments_wrappers import startSim

from typing import Dict, List
import hydra


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


@hydra.main(config_name="config", config_path="cfg")
def run(cfg: DictConfig):
    cfg = omegaconfToDict(cfg)
    cfg = instantiateConfigs(cfg)
    SM, simulation_app = startSim(cfg)

    SM.run_simulation()
    simulation_app.close()


if __name__ == "__main__":
    run()
