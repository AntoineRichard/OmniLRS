__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from threading import Thread

from omni.isaac.core import World
from typing import Union
import omni
import time

from src.environments_wrappers.ros2.largescale_ros2 import ROS_LargeScaleManager
from src.environments_wrappers.ros2.lunaryard_ros2 import ROS_LunaryardManager
from src.environments_wrappers.ros2.robot_manager_ros2 import ROS_RobotManager
from src.environments_wrappers.ros2.lunalab_ros2 import ROS_LunalabManager
from src.configurations.procedural_terrain_confs import TerrainManagerConf
from rclpy.executors import SingleThreadedExecutor as Executor
from src.physics.physics_scene import PhysicsSceneManager


class Rate:
    """
    Creates a rate object that enables to sleep for a minimum amount
    of time between two iterations of a loop. If freq and dt are
    passed, the object will only use the information provided by dt.
    """

    def __init__(self, freq: float = None, dt: float = None, is_disabled: bool = False) -> None:
        """
        Args:
          freq (float): The frequency at which the loop should be executed.
          dt (float): The delta of time to be kept between two loop iterations.
        """

        self.is_disabled = is_disabled

        if not self.is_disabled:
            if dt is None:
                if freq is None:
                    raise ValueError("You must provide either a frequency or a delta time.")
                else:
                    self.dt = 1.0 / freq
            else:
                self.dt = dt

            self.last_check = time.time()

    def sleep(self) -> None:
        """
        Wait for a minimum amount of time between two iterations of a loop.
        """

        if not self.is_disabled:
            now = time.time()
            delta = now - self.last_check
            # If time delta is too low sleep, else carry on.
            if delta < self.dt:
                to_sleep = delta - self.dt
                time.sleep(to_sleep)
                # Update last check after sleep
                self.last_check = time.time()


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
ROS2_LMF.register("large_scale", ROS_LargeScaleManager)


class ROS2_SimulationManager:
    """
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the lab manager thread
    - Running the robot manager thread
    - Running the simulation
    - Cleaning the simulation
    """

    def __init__(
        self,
        cfg: dict,
        simulation_app,
    ) -> None:
        """
        Initializes the simulation.

        Args:
            cfg (dict): Configuration dictionary.
            simulation_app (SimulationApp): SimulationApp instance.
        """

        self.cfg = cfg
        self.simulation_app = simulation_app
        # Setups the physics and acquires the different interfaces to talk with Isaac
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=cfg["environment"]["physics_dt"],
            rendering_dt=cfg["environment"]["rendering_dt"],
        )
        PSM = PhysicsSceneManager(cfg["physics"]["physics_scene"])
        for i in range(100):
            self.world.step(render=True)
        self.world.reset()

        if cfg["environment"]["enforce_realtime"]:
            self.rate = Rate(dt=cfg["environment"]["physics_dt"])
        else:
            self.rate = Rate(is_disabled=True)

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
        self.terrain_manager_conf: TerrainManagerConf = cfg["environment"]["terrain_manager"]
        self.render_deform_inv = self.terrain_manager_conf.moon_yard.deformation_engine.render_deform_inv
        self.enable_deformation = self.terrain_manager_conf.moon_yard.deformation_engine.enable

        # Preload the assets
        self.ROSRobotManager.RM.preload_robot(self.world)
        self.ROSLabManager.LC.add_robot_manager(self.ROSRobotManager.RM)

        for i in range(100):
            self.world.step(render=True)
        self.world.reset()

    def run_simulation(self) -> None:
        """
        Runs the simulation.
        """

        self.timeline.play()
        while self.simulation_app.is_running():
            self.world.step(render=True)
            if self.world.is_playing():
                # Apply modifications to the lab only once the simulation step is finished
                # This is extremely important as modifying the stage during a simulation step
                # will lead to a crash.
                self.ROSLabManager.periodic_update(dt=self.world.get_physics_dt())
                if self.world.current_time_step_index == 0:
                    self.world.reset()
                    self.ROSLabManager.reset()
                    self.ROSRobotManager.reset()
                self.ROSLabManager.apply_modifications()
                if self.ROSLabManager.trigger_reset:
                    self.ROSRobotManager.reset()
                    self.ROSLabManager.trigger_reset = False
                self.ROSRobotManager.apply_modifications()
                if self.enable_deformation:
                    if self.world.current_time_step_index % self.render_deform_inv == 0:
                        self.ROSLabManager.LC.deform_terrain()
                    # self.ROSLabManager.LC.applyTerramechanics()
            self.rate.sleep()

        self.timeline.stop()
