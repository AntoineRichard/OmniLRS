__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from threading import Thread

from omni.isaac.core import World
from typing import Union
import omni

from src.environments_wrappers.ros2.robot_manager_ros2 import ROS_RobotManager
from src.environments_wrappers.ros2.lunalab_ros2 import ROS_LunalabManager
from src.environments_wrappers.ros2.lunaryard_ros2 import ROS_LunaryardManager
from rclpy.executors import SingleThreadedExecutor as Executor


class ROS2_LabManagerFactory:
    def __init__(self):
        self._lab_managers = {}

    def register(
        self,
        name: str,
        lab_manager: Union[ROS_LunalabManager, ROS_LunaryardManager],
    ) -> None:
        """
        Registers a lab manager.

        Args:
            name (str): Name of the lab manager.
            lab_manager (Union[ROS_LunalabManager, ROS_LunaryardManager]): Instance of the lab manager.
        """

        self._lab_managers[name] = lab_manager

    def __call__(
        self,
        cfg: dict,
    ) -> Union[ROS_LunalabManager, ROS_LunaryardManager]:
        """
        Returns an instance of the lab manager corresponding to the environment name.

        Args:
            cfg (dict): Configuration dictionary.

        Returns:
            Union[ROS_LunalabManager, ROS_LunaryardManager]: Instance of the lab manager.
        """

        return self._lab_managers[cfg["environment"]["name"]](
            environment_cfg=cfg["environment"],
            flares_cfg=cfg["rendering"]["lens_flares"],
        )


ROS2_LMF = ROS2_LabManagerFactory()
ROS2_LMF.register("Lunalab", ROS_LunalabManager)
ROS2_LMF.register("Lunaryard", ROS_LunaryardManager)


class ROS2_SimulationManager:
    """ "
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the lab manager thread
    - Running the robot manager thread
    - Running the simulation
    - Cleaning the simulation"""

    def __init__(
        self,
        cfg: dict,
        simulation_app,
    ) -> None:
        """
        Initializes the simulation.

        Args:
            cfg (dict): Configuration dictionary.
            simulation_app (SimulationApp): SimulationApp instance."""
        self.cfg  = cfg
        self.simulation_app = simulation_app
        # Setups the physics and acquires the different interfaces to talk with Isaac
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        self.physics_ctx = self.world.get_physics_context()
        self.physics_ctx.set_solver_type("PGS")
        # Lab manager thread
        self.ROSLabManager = ROS2_LMF(cfg)
        exec1 = Executor()
        exec1.add_node(self.ROSLabManager)
        self.exec1_thread = Thread(target=exec1.spin, daemon=True, args=())
        self.exec1_thread.start()
        # Robot manager thread
        self.ROSRobotManager = ROS_RobotManager(cfg["environment"]["robots_settings"])
        exec2 = Executor()
        exec2.add_node(self.ROSRobotManager)
        self.exec2_thread = Thread(target=exec2.spin, daemon=True, args=())
        self.exec2_thread.start()

        # Have you ever asked your self: "Is there a limit of topics one can subscribe to in ROS2?"
        # Yes "Josh" there is.
        # 24 topics. More than that and you won't reveive any messages.
        # Keep it in mind if you want to go crazy with the ROS2 calls to modify the sim...
        self.world.reset()
        
        self.terrain_manager_conf = cfg["environment"]["terrain_manager"]
        self.render_deform_inv = self.terrain_manager_conf.moon_yard.deformation_engine.render_deform_inv
        self.enable_deformation = self.terrain_manager_conf.moon_yard.deformation_engine.enable
        
        # Preload the assets
        self.ROSRobotManager.RM.preloadRobot(self.world)
        self.ROSLabManager.LC.addRobotManager(self.ROSRobotManager.RM)

    def run_simulation(self) -> None:
        """
        Runs the simulation."""

        self.timeline.play()
        while self.simulation_app.is_running():
            self.world.step(render=True)
            if self.world.is_playing():
                # Apply modifications to the lab only once the simulation step is finished
                # This is extremely important as modifying the stage during a simulation step
                # will lead to a crash.
                if self.world.current_time_step_index == 0:
                    self.world.reset()
                    self.ROSLabManager.reset()
                    self.ROSRobotManager.reset()
                self.ROSLabManager.applyModifications()
                if self.ROSLabManager.trigger_reset:
                    self.ROSRobotManager.reset()
                    self.ROSLabManager.trigger_reset = False
                self.ROSRobotManager.applyModifications()
                if self.enable_deformation:
                    if self.world.current_time_step_index % self.render_deform_inv == 0:
                        self.ROSLabManager.LC.deformTerrain()
                    # self.ROSLabManager.LC.applyTerramechanics()

        self.timeline.stop()
