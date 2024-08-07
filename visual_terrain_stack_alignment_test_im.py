# import omni
# import math
# from omni.isaac.kit import SimulationApp

# simulation_app = SimulationApp({"headless": False})
HRDEMCfg_D = {
    "num_blocks": 4,
    "block_size": 500,
    "pad_size": 50.0,
    "max_blocks": int(1e7),
    "seed": 42,
    "resolution": 0.05,
    "z_scale": 1.0,
    "source_resolution": 5.0,
    "resolution": 0.5,
    "interpolation_padding": 2,
    "generate_craters": False,
}
CWMCfg_D = {
    "num_workers": 8,
    "input_queue_size": 400,
    "output_queue_size": 16,
    "worker_queue_size": 2,
}
IWMCfg_D = {
    "num_workers": 1,
    "input_queue_size": 400,
    "output_queue_size": 30,
    "worker_queue_size": 200,
}
CraterDBCfg_D = {
    "block_size": 50,
    "max_blocks": 7,
    "save_to_disk": False,
    "write_to_disk_interval": 100,
}
CGCfg_D = {
    "profiles_path": "assets/Terrains/crater_spline_profiles.pkl",
    "min_xy_ratio": 0.85,
    "max_xy_ratio": 1.0,
    "random_rotation": True,
    "seed": 42,
    "num_unique_profiles": 10000,
}
CDDCfg_D = {
    "densities": [0.025, 0.05, 0.5],
    "radius": [[1.5, 2.5], [0.75, 1.5], [0.25, 0.5]],
    "num_repeat": 1,
    "seed": 42,
}
CraterSamplerCfg_D = {
    "block_size": 50,
    "crater_gen_cfg": CGCfg_D,
    "crater_dist_cfg": CDDCfg_D,
}
CraterBuilderCfg_D = {
    "block_size": 50,
    "pad_size": 10.0,
    "resolution": 0.5,
    "z_scale": 1.0,
}
ICfg = {
    "source_resolution": 5.0,
    "target_resolution": 0.5,
    "source_padding": 2,
    "method": "bicubic",
}
HRDEMGenCfg_D = {
    "high_res_dem_cfg": HRDEMCfg_D,
    "crater_db_cfg": CraterDBCfg_D,
    "crater_sampler_cfg": CraterSamplerCfg_D,
    "crater_builder_cfg": CraterBuilderCfg_D,
    "interpolator_cfg": ICfg,
    "crater_worker_manager_cfg": CWMCfg_D,
    "interpolator_worker_manager_cfg": IWMCfg_D,
}

MMCfg_D = {
    "folder_path": "assets/Terrains/SouthPole",
    "lr_dem_name": "crater",
}

NGCMMCfg_D = {
    "num_texels_per_level": 256,
    "target_res": 0.1,
    "fine_interpolation_method": "bilinear",
    "coarse_interpolation_method": "bilinear",
    "fine_acceleration_mode": "hybrid",
    "coarse_acceleration_mode": "hybrid",
}

LSTMCfg_D = {
    "map_name": "NPD_final_adj_5mpp_surf",
    "pixel_coordinates": (0, 0),
    "ll_coordinates": (0, 0),
    "meters_coordinates": (0, 0),
    "coordinate_format": "meters",
    "visual_mesh_update_threshold": 2.0,
}

if __name__ == "__main__":
    # from omni.isaac.core import World
    # from omni.usd import get_context
    # from pxr import UsdLux
    # from WorldBuilders.pxr_utils import setDefaultOps, addDefaultOps
    import numpy as np

    from src.terrain_management.map_manager import MapManagerCfg, MapManager
    from src.terrain_management.high_res_dem_gen import HighResDEMGenCfg

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

    C = 20000 * 5
    R = 500 * 5
    max_displacement = 2.0 / 30
    perimeter = 2 * np.pi * R
    rotation_rate = int(perimeter / max_displacement)
    spiral_rate = 0.0 / rotation_rate
    theta = np.linspace(0, 2 * np.pi, rotation_rate)
    print(rotation_rate)

    hrdem_settings = HighResDEMGenCfg(**HRDEMGenCfg_D)
    mm_settings = MapManagerCfg(**MMCfg_D)
    # ngcmm_settings = NestedGeometricClipMapManagerCfg(**NGCMMCfg_D)
    # lstm_settings = LargeScaleTerrainManagerCfg(**LSTMCfg_D)

    from matplotlib import pyplot as plt
    import matplotlib.colors as mcolors
    import cv2

    MM = MapManager(hrdem_settings, mm_settings)
    MM.load_lr_dem_by_name("NPD_final_adj_5mpp_surf")
    MM.initialize_hr_dem((0, 0))
    MM.hr_dem_gen.shutdown()
    norm = mcolors.Normalize(vmin=MM.lr_dem.min(), vmax=MM.lr_dem.max())
    hr_rs = cv2.resize(
        MM.hr_dem_gen.high_res_dem, (0, 0), fx=0.1, fy=0.1, interpolation=cv2.INTER_AREA
    )
    lr_shape = MM.get_lr_dem_shape()
    x = lr_shape[0] // 2 + 50
    y = lr_shape[1] // 2 + 50
    plt.figure()
    plt.imshow(hr_rs, cmap="terrain", norm=norm)
    plt.figure()
    plt.imshow(MM.lr_dem, cmap="terrain", norm=norm)
    plt.figure()
    plt.imshow(
        MM.lr_dem[x - 550 : x + 550, y - 550 : y + 550], cmap="terrain", norm=norm
    )
    diff = hr_rs - MM.lr_dem[x - 550 : x + 550, y - 550 : y + 550]
    plt.figure()
    plt.imshow(diff, cmap="jet")
    plt.show()

    # LSTM = LargeScaleTerrainManager(
    #    lstm_settings, ngcmm_settings, hrdem_settings, mm_settings
    # )

    # LSTM.build()
    # LSTM.update_visual_mesh((0, 0))

    # i = 0
    # i2 = 1.0
    # while True:
    # GCM.updateGeoClipmap(
    #    np.array(
    #        [C + i2 * R * math.cos(theta[i]), C + i2 * R * math.sin(theta[i]), 0]
    #    ),
    #    np.array([i2 * R * math.cos(theta[i]), i2 * R * math.sin(theta[i]), 0]),
    # )
    # world.step(render=True)
    # i = (i + 1) % rotation_rate
    # i2 += spiral_rate
