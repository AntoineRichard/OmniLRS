__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

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
        from src.environments_wrappers.ros2.simulation_manager_ros2 import ROS2_SimulationManager
        SM = ROS2_SimulationManager(cfg, simulation_app)
    if cfg["mode"] == "ROS1":
        from src.environments_wrappers.ros1 import enable_ros1
        enable_ros1(simulation_app)
        import rospy
        rospy.init_node("omni_isaac_ros1")
        from src.environments_wrappers.ros1.simulation_manager_ros1 import ROS1_SimulationManager
        SM = ROS1_SimulationManager(cfg, simulation_app)
    
    return SM, simulation_app



@hydra.main(config_name="config", config_path="cfg")
def run(cfg: DictConfig):
    cfg = omegaconfToDict(cfg)
    cfg = instantiateConfigs(cfg)
    SM, simulation_app = startSim(cfg)

    SM.run_simulation()
    simulation_app.close()

if __name__ == '__main__':
    run()