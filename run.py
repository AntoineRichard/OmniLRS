from omegaconf import DictConfig, OmegaConf
from src.configurations import configFactory

from typing import Dict
import hydra

def resolve_tuple(*args):
    return tuple(args)

OmegaConf.register_new_resolver('as_tuple', resolve_tuple)

def omegaconfToDict(d: DictConfig) -> Dict:
    """Converts an omegaconf DictConfig to a python Dict, respecting variable interpolation.
    
    Args:
        d (DictConfig): OmegaConf DictConfig.
    
    Returns:
        Dict: Python dict."""

    ret = {}
    for k, v in d.items():
        if isinstance(v, DictConfig):
            ret[k] = omegaconfToDict(v)
        else:
            ret[k] = v
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


def startSim(cfg: dict):
    from omni.isaac.kit import SimulationApp
    # Start the simulation
    simulation_app = SimulationApp(cfg["rendering"]["renderer"].__dict__)
    if cfg["mode"] == "ROS2":
        from src.environments_wrappers.ros2 import enable_ros2
        enable_ros2(simulation_app)
        import rclpy
        rclpy.init()
    
    return simulation_app



@hydra.main(config_name="config", config_path="cfg")
def run(cfg: DictConfig):
    cfg = omegaconfToDict(cfg)
    cfg = instantiateConfigs(cfg)
    simulation_app = startSim(cfg)

    from src.environments_wrappers.ros2.lunalab_ros2 import SimulationManager

    SM = SimulationManager(cfg, simulation_app)
    SM.run_simulation()
    simulation_app.close()

if __name__ == '__main__':
    run()