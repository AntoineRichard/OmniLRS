__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import Any

from src.configurations.procedural_terrain_confs import TerrainManagerConf, MoonYardConf, CraterGeneratorConf, CraterDistributionConf, BaseTerrainGeneratorConf
from src.configurations.auto_labeling_confs import AutoLabelingConf
from src.configurations.rendering_confs import FlaresConf, RendererConf
from src.configurations.environments import LunalabLabConf

class ConfigFactory:
    def __init__(self):
        self.configs = {}

    def registerConfig(self, config_name:str, config: Any) -> None:
        self.configs[config_name] = config

    def __call__(self, config_name:str, **kwargs) -> Any:
        return self.configs[config_name](**kwargs)
    
    def getConfigs(self) -> list:
        return self.configs.keys()
    

configFactory = ConfigFactory()
configFactory.registerConfig("terrain_manager", TerrainManagerConf)
configFactory.registerConfig("moon_yard", MoonYardConf)
configFactory.registerConfig("crater_generator", CraterGeneratorConf)
configFactory.registerConfig("crater_distribution", CraterDistributionConf)
configFactory.registerConfig("base_terrain_generator", BaseTerrainGeneratorConf)
configFactory.registerConfig("auto_labeling", AutoLabelingConf)
configFactory.registerConfig("lens_flares", FlaresConf)
configFactory.registerConfig("renderer", RendererConf)
configFactory.registerConfig("lunalab_settings", LunalabLabConf)