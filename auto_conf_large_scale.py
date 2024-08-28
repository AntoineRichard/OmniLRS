import dataclasses


@dataclasses.dataclass
class LargeScaleTerrainCfg:
    seed: int = 42
    hr_dem_resolution: float = 0.025


HRDEMCfg_D = {
    "num_blocks": 4,
    "block_size": 50,
    "pad_size": 10.0,
    "max_blocks": int(1e7),
    "seed": 42,
    "resolution": 0.05,
    "z_scale": 1.0,
    "source_resolution": 5.0,
    "resolution": 0.025,
    "interpolation_padding": 2,
    "generate_craters": True,
}
CWMCfg_D = {
    "num_workers": 8,
    "input_queue_size": 400,
    "output_queue_size": 30,
}
IWMCfg_D = {
    "num_workers": 1,
    "input_queue_size": 400,
    "output_queue_size": 30,
}
CraterDBCfg_D = {
    "block_size": 50,
    "max_blocks": int(1e7),
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
    "resolution": 0.025,
    "z_scale": 1.0,
}
ICfg = {
    "source_resolution": 5.0,
    "target_resolution": 0.025,
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
    "hrdem_settings": HRDEMCfg_D,
}

NGCMMCfg_D = {
    "num_texels_per_level": 384,
    "target_res": 0.02,
    "fine_interpolation_method": "bilinear",
    "coarse_interpolation_method": "bicubic",
    "fine_acceleration_mode": "hybrid",
    "coarse_acceleration_mode": "gpu",
    "profiling": True,
}
RDBCfg_D = {
    "block_size": 50,
    "max_blocks": int(1e7),
    "save_to_disk": False,
    "write_to_disk_interval": 1000,
}
RSCfg_1_D = {
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
            "min": 0.02,
            "max": 0.05,
            "seed": 42,
        },
        "seed": 42,
        "num_rock_id": 49,
    },
}
RSCfg_2_D = {
    "block_size": 50,
    "rock_dist_cfg": {
        "position_distribution": {
            "name": "thomas_point_process",
            "parent_density": 0.01,
            "child_density": 25,
            "sigma": 3,
            "seed": 42,
        },
        "scale_distribution": {
            "name": "uniform",
            "min": 0.05,
            "max": 0.2,
            "seed": 42,
        },
        "seed": 43,
        "num_rock_id": 49,
    },
}
RGCfg_1_D = {
    "rock_db_cfg": RDBCfg_D,
    "rock_sampler_cfg": RSCfg_1_D,
    "rock_assets_folder": "assets/USD_Assets/rocks/small",
    "instancer_name": "very_small_rock_instancer",
    "seed": 42,
    "block_span": 0,
    "block_size": 50,
}
RGCfg_2_D = {
    "rock_db_cfg": RDBCfg_D,
    "rock_sampler_cfg": RSCfg_2_D,
    "rock_assets_folder": "assets/USD_Assets/rocks/small",
    "instancer_name": "large_rock_instancer",
    "seed": 42,
    "block_span": 1,
    "block_size": 50,
}
RMCfg_D = {
    "rock_gen_cfgs": [RGCfg_1_D, RGCfg_2_D],
    "instancers_path": "/World/rock_instancers",
    "seed": 42,
    "block_size": 50,
}

LSTMCfg_D = {
    "map_name": "Site20_final_adj_5mpp_surf",
    "pixel_coordinates": (0, 0),
    "ll_coordinates": (0, 0),
    "meters_coordinates": initial_position,
    "coordinate_format": "meters",
    "visual_mesh_update_threshold": 2.0,
}

mm_settings = MapManagerCfg(**MMCfg_D)
ngcmm_settings = NestedGeometricClipMapManagerCfg(**NGCMMCfg_D)
rm_settings = RockManagerCfg(**RMCfg_D)
lstm_settings = LargeScaleTerrainManagerCfg(**LSTMCfg_D)

LSTM = LargeScaleTerrainManager(lstm_settings, ngcmm_settings, hrdem_settings, mm_settings, rm_settings)

LSTM.build()
height = LSTM.get_height_global(initial_position)
