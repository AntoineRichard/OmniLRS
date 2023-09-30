__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from threading import Thread
from typing import Any

from omni.isaac.core import World
import omni

from src.environments_wrappers.ros1.lunalab_ros1 import ROS_LunalabManager
from src.environments_wrappers.ros1.lunaryard_ros1 import ROS_LunaryardManager

class ROS1_LabManagerFactory:
    def __init__(self):
        self._lab_managers = {}

    def register(self, name, lab_manager):
        self._lab_managers[name] = lab_manager

    def __call__(self, cfg):
        return self._lab_managers[cfg["environment"]["name"]](cfg["environment"], cfg["rendering"]["lens_flares"])


ROS1_LMF = ROS1_LabManagerFactory()
ROS1_LMF.register("Lunalab", ROS_LunalabManager)
ROS1_LMF.register("Lunaryard", ROS_LunaryardManager)


class ROS1_SimulationManager:
    """"
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the lab and the robot manager thread
    - Running the simulation
    - Cleaning the simulation"""

    def __init__(self, cfg, simulation_app) -> None:

        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        self.physics_ctx = self.world.get_physics_context()
        self.physics_ctx.set_solver_type("PGS")
        self.ROSLabManager = ROS1_LMF(cfg)
        self.simulation_app = simulation_app
        
    def run_simulation(self) -> None:
        """
        Runs the simulation."""

        self.timeline.play()
        while self.simulation_app.is_running():
            self.world.step(render=True)
            if self.world.is_playing():
                if self.world.current_time_step_index == 0:
                    self.world.reset()
                    self.ROSLabManager.reset()
                self.ROSLabManager.applyModifications()

        self.timeline.stop()