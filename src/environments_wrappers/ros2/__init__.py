__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

def enable_ros2(simulation_app, **kwargs) -> None:
    """
    Enables ROS2 in the simulation.
    
    Args:
        simulation_app (SimulationApp): SimulationApp instance."""

    from omni.isaac.core.utils.extensions import enable_extension
    enable_extension("omni.isaac.ros2_bridge")
    enable_extension("omni.kit.viewport.actions")

    simulation_app.update()