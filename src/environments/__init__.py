__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import Any

#from src.environments.lunaryard_capture import LunarYardCapture
#from src.environments.lunaryard import LunarYard
#from src.environments.lunalab import 
#
#class EnvironmentFactory:
#    def __init__(self):
#        self.envs = {}
#
#    def registerEnvironment(self, env_name:str, env: Any) -> None:
#        self.envs[env_name] = env
#
#    def __call__(self, env_name:str, **kwargs) -> Any:
#        return self.envs[env_name](**kwargs)
#    
#    def getEnvironments(self) -> list:
#        return self.envs.keys()
#    
#environmentFactory = EnvironmentFactory()
#environmentFactory.registerEnvironment("LunarYardCapture", LunarYardCapture)
#environmentFactory.registerEnvironment("LunarYard", LunarYard)
#environmentFactory.registerEnvironment("LunarLab", LunarLab)
