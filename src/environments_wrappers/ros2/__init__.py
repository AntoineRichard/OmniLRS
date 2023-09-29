def enable_ros2(simulation_app) -> None:
    """
    Enables ROS2 in the simulation.
    
    Args:
        simulation_app (SimulationApp): SimulationApp instance."""

    from omni.isaac.core.utils.extensions import enable_extension
    enable_extension("omni.isaac.ros2_bridge")
    enable_extension("omni.kit.viewport.actions")

    simulation_app.update()