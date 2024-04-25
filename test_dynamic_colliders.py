import omni
import math
from omni.isaac.kit import SimulationApp


if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": False})

    from omni.isaac.core import World
    import numpy as np
    from src.terrain_management.colliders_manager import TerrainManager
    from src.configurations.procedural_terrain_confs import (
        TerrainManagerConf,
        MoonYardConf,
        CraterDistributionConf,
        CraterGeneratorConf,
        BaseTerrainGeneratorConf,
    )

    CGC = CraterGeneratorConf(
        profiles_path="assets/Terrains/crater_spline_profiles.pkl",
        min_xy_ratio=0.85,
        max_xy_ratio=1.0,
        resolution=0.1,
        pad_size=500,
        random_rotation=True,
        z_scale=1.0,
        seed=42,
    )
    CDC = CraterDistributionConf(
        x_size=50.0,
        y_size=50.0,
        densities=[0.025, 0.05, 0.5],
        radius=[[1.5, 2.5], [0.75, 1.5], [0.25, 0.5]],
        num_repeat=0,
        seed=42,
    )
    BTGC = BaseTerrainGeneratorConf(
        x_size=50,
        y_size=50,
        resolution=0.1,
        max_elevation=0.5,
        min_elevation=0.5,
        z_scale=1.0,
        seed=42,
    )
    MYC = MoonYardConf(
        crater_generator=CGC,
        crater_distribution=CDC,
        base_terrain_generator=BTGC,
        is_yard=True,
        is_lab=False,
    )
    TMC = TerrainManagerConf(
        moon_yard=MYC,
        root_path="/Main",
        texture_path="/Main/Looks/Basalt",
        dems_path="Terrains/Lunalab",
        mesh_position=(0, 0, 0),
        mesh_orientation=(0, 0, 0, 1),
        mesh_scale=(1, 1, 1),
        sim_length=50.0,
        sim_width=50.0,
        resolution=0.1,
    )

    TM = TerrainManager(cfg=TMC)

    world = World(stage_units_in_meters=1.0)

    TM.build_initial_block()

    while True:
        world.step(render=True)
