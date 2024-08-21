# import omni
# import math
# from omni.isaac.kit import SimulationApp

# simulation_app = SimulationApp({"headless": False})

# RS_Cfg_D = {
#    "block_size": 50,
#    "rock_dist_cfg": {
#        "position_distribution": {
#            "name": "poisson",
#            "density": 10,
#            "seed": 42,
#        },
#        "scale_distribution": {
#            "name": "uniform",
#            "min": 0.5,
#            "max": 1.5,
#            "seed": 42,
#        },
#        "seed": 42,
#        "num_rock_id": 50,
#    },
# }
RS_Cfg_D = {
    "block_size": 50,
    "rock_dist_cfg": {
        "position_distribution": {
            "name": "thomas_point_process",
            "parent_density": 0.04,
            "child_density": 100,
            "sigma": 3,
            "seed": 42,
        },
        "scale_distribution": {
            "name": "uniform",
            "min": 0.5,
            "max": 1.5,
            "seed": 42,
        },
        "seed": 42,
        "num_rock_id": 50,
    },
}

RD_Cfg_D = {
    "block_size": 50,
    "max_blocks": int(1e7),
    "save_to_disk": False,
    "write_to_disk_interval": 1000,
}

if __name__ == "__main__":
    # from omni.isaac.core import World
    # from omni.usd import get_context
    # from pxr import UsdLux
    # from WorldBuilders.pxr_utils import setDefaultOps, addDefaultOps
    import numpy as np

    from src.terrain_management.large_scale_terrain.rock_distribution import (
        RockSamplerCfg,
        RockSampler,
    )
    from src.terrain_management.large_scale_terrain.rock_database import (
        RockDB,
        RockDBCfg,
    )
    from src.terrain_management.large_scale_terrain.utils import (
        BoundingBox,
        ScopedTimer,
    )

    RS_Cfg = RockSamplerCfg(**RS_Cfg_D)
    RD_Cfg = RockDBCfg(**RD_Cfg_D)

    RD = RockDB(RD_Cfg)
    RS = RockSampler(RS_Cfg, RD)

    # from src.terrain_management.large_scale_terrain_manager import (
    #    NestedGeometricClipMapManagerCfg,
    #    LargeScaleTerrainManagerCfg,
    #    LargeScaleTerrainManager,
    # )

    # world = World(stage_units_in_meters=1.0)
    # stage = get_context().get_stage()

    # Let there be light
    # light = UsdLux.DistantLight.Define(stage, "/sun")
    # light.CreateIntensityAttr(3000.0)
    # addDefaultOps(light.GetPrim())
    # setDefaultOps(light.GetPrim(), (0, 0, 0), (0.383, 0, 0, 0.924), (1, 1, 1))

    from matplotlib import pyplot as plt

    with ScopedTimer("Sampling rocks by block"):
        RS.sample_rocks_by_block((0, 0))
        RS.sample_rocks_by_block((50, 0))
        RS.sample_rocks_by_block((0, 50))
        RS.sample_rocks_by_block((50, 50))
    print("Number of rock blocks in the database: ", len(RD.rock_db))
    print("DB size in bytes: ", RD.get_memory_footprint())
    print("number of rocks in the region: ", RD.number_of_elements())

    region = BoundingBox(0, 100, 0, 100)

    RS.display_region(region)

    region = BoundingBox(0, 400, 0, 400)
    with ScopedTimer("Sampling rocks by region"):
        RS.sample_rocks_by_region(region)
    print("Number of rock blocks in the database: ", len(RD.rock_db))
    print("DB size in MB: ", RD.get_memory_footprint(unit="MB"))
    print("number of rocks in the region: ", RD.number_of_elements())
    RS.display_region(region)
    with ScopedTimer("Sampling rocks by region"):
        RS.sample_rocks_by_region(region)
    plt.show()
