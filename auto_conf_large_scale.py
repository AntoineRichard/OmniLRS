import dataclasses

from src.terrain_management.large_scale_terrain.high_resolution_DEM_generator import HighResDEMGenCfg

@dataclasses.dataclass
class SeedOverride:
    override: bool = False
    general_seed: int = dataclasses.field(default_factory=int)
    crater_gen_distribution_seed: int = dataclasses.field(default_factory=int)
    crater_gen_metadata_seed: int = dataclasses.field(default_factory=int)


    

@dataclasses.dataclass
class LargeScaleTerrainCfg:
    seed: int = 42
    profiling: bool = False
    update_every_n_meters: float = 2.0
    z_scale: float = 1.0
    block_size: int = 50

    max_elements_in_dbs = int(1e7)
    save_dbs_to_disk: bool = False
    dbs_write_interval: int = 1000

    hr_dem_resolution: float = 0.025
    hr_dem_generate_craters: bool = True
    hr_dem_num_blocks: int = 4

    num_workers_craters: int = 8
    num_workers_interpolation: int = 1
    input_queue_size: int = 400
    output_queue_size: int = 30

    crater_gen_densities: list = dataclasses.field(default_factory=list)
    crater_gen_radius: list = dataclasses.field(default_factory=list)
    crater_gen_profiles_path: str = "assets/Terrains/crater_spline_profiles.pkl"
    crater_gen_padding: float = 10
    crater_gen_min_xy_ratio: float = 0.85
    crater_gen_max_xy_ratio: float = 1.0
    crater_gen_random_rotation: bool = True
    ccrater_gen_rater_gen_num_unique_profiles: int = 10000

    hrdem_interpolation_method: str = "bicubic"
    hrdem_interpolator_name: str = "PIL"
    hrdem_interpolator_padding: int = 2

    lr_dem_folder_path: str = "assets/Terrains/SouthPole"
    lr_dem_name: str = "Site20_final_adj_5mpp_surf"

    num_texels_per_level: int = 384
    target_res: float = 0.02
    fine_interpolation_method: str = "bilinear"
    coarse_interpolation_method: str = "bicubic"
    fine_acceleration_mode: str = "hybrid"
    coarse_acceleration_mode: str = "gpu"
    apply_smooth_shading: bool = False

    rock_gen_cfgs: list = dataclasses.field(default_factory=list)

    def __post_init__(self):

        assert type(self.seed) == int, "Seed must be an integer."
        assert type(self.profiling) == bool, "Profiling must be a boolean."
        assert type(self.update_every_n_meters) == float, "Update every n meters must be a float."
        assert type(self.z_scale) == float, "Z scale must be a float."
        assert type(self.block_size) == int, "Block size must be an integer."
        assert type(self.max_elements_in_dbs) == int, "Max elements in dbs must be an integer."
        assert type(self.save_dbs_to_disk) == bool, "Save dbs to disk must be a boolean."
        assert type(self.dbs_write_interval) == int, "Dbs write interval must be an integer."
        assert type(self.hr_dem_resolution) == float, "HR DEM resolution must be a float."
        assert type(self.hr_dem_generate_craters) == bool, "HR DEM generate craters must be a boolean."
        assert type(self.hr_dem_num_blocks) == int, "HR DEM num blocks must be an integer."

        
        HRDEMCfg_D = {
            "num_blocks": self.hr_dem_num_blocks,
            "block_size": self.block_size,
            "pad_size": self.crater_gen_padding,
            "max_blocks": self.max_elements_in_dbs,
            "seed": 42,
            "z_scale": self.z_scale,
            "resolution": self.hr_dem_resolution,
            "interpolation_padding": self.hrdem_interpolator_padding,
            "generate_craters": self.hr_dem_generate_craters,
        }
        CWMCfg_D = {
            "num_workers": self.num_workers_craters,
            "input_queue_size": self.input_queue_size,
            "output_queue_size": self.output_queue_size,
        }
        IWMCfg_D = {
            "num_workers": self.num_workers_interpolation,
            "input_queue_size": self.input_queue_size,
            "output_queue_size": self.output_queue_size,
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
