__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"


def startSim(cfg: dict):
    from omni.isaac.kit import SimulationApp
    from src.environments.rendering import set_lens_flares, set_chromatic_aberrations, set_motion_blur

    # Starts the simulation and allows to import things related to Isaac and PXR
    renderer_cfg = cfg["rendering"]["renderer"]
    simulation_app = SimulationApp(renderer_cfg.__dict__)
    set_lens_flares(cfg)
    set_motion_blur(cfg)
    set_chromatic_aberrations(cfg)

    # Starts the ROS2 extension. Allows to import ROS2 related things.
    if cfg["mode"]["name"] == "ROS2":
        # ROS2 startup routine
        from src.environments_wrappers.ros2 import enable_ros2

        enable_ros2(simulation_app, bridge_name=cfg["mode"]["bridge_name"])
        import rclpy

        rclpy.init()
        # Call to the environment factory to load the correct environment.
        from src.environments_wrappers.ros2.simulation_manager_ros2 import (
            ROS2_SimulationManager,
        )

        SM = ROS2_SimulationManager(cfg, simulation_app)

    # Starts the ROS1 extension. Allows to import ROS1 related things.
    if cfg["mode"]["name"] == "ROS1":
        # ROS1 startup routine
        from src.environments_wrappers.ros1 import enable_ros1

        enable_ros1(simulation_app)
        import rospy

        rospy.init_node("omni_isaac_ros1")
        # Call to the environment factory to load the correct environment.
        from src.environments_wrappers.ros1.simulation_manager_ros1 import (
            ROS1_SimulationManager,
        )

        SM = ROS1_SimulationManager(cfg, simulation_app)

    # Starts the replicator stuff. Allows to acquire synthetic data.
    if cfg["mode"]["name"] == "SDG":
        # Call to the environment factory to load the correct environment.
        from src.environments_wrappers.sdg.simulation_manager_sdg import (
            SDG_SimulationManager,
        )

        SM = SDG_SimulationManager(cfg, simulation_app)

    return SM, simulation_app
