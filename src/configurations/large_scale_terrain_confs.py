import dataclasses
import os


@dataclasses.dataclass
class LargeScaleTerrainConf:
    seed: int = 42
    crater_gen_seed: int = None
    crater_gen_distribution_seed: int = None
    crater_gen_metadata_seed: int = None
    rock_gen_main_seed: int = None

    profiling: bool = False
    update_every_n_meters: float = 2.0
    z_scale: float = 1.0
    block_size: int = 50

    dbs_max_elements: int = int(1e7)
    dbs_save_to_disk: bool = False
    dbs_write_interval: int = 1000

    hr_dem_resolution: float = 0.025
    hr_dem_generate_craters: bool = True
    hr_dem_num_blocks: int = 4

    crater_gen_densities: list = dataclasses.field(default_factory=list)
    crater_gen_radius: list = dataclasses.field(default_factory=list)
    crater_gen_profiles_path: str = "assets/Terrains/crater_spline_profiles.pkl"
    crater_gen_padding: float = 10
    crater_gen_min_xy_ratio: float = 0.85
    crater_gen_max_xy_ratio: float = 1.0
    crater_gen_random_rotation: bool = True
    crater_gen_num_unique_profiles: int = 10000

    num_workers_craters: int = 8
    num_workers_interpolation: int = 1
    input_queue_size: int = 400
    output_queue_size: int = 30

    hrdem_interpolation_method: str = "bicubic"
    hrdem_interpolator_name: str = "PIL"
    hrdem_interpolator_padding: int = 2

    lr_dem_folder_path: str = "assets/Terrains/SouthPole"
    lr_dem_name: str = "Site20_final_adj_5mpp_surf"
    starting_position: tuple = (0, 0)

    geo_cm_num_texels_per_level: int = 384
    geo_cm_target_res: float = 0.02
    geo_cm_fine_interpolation_method: str = "bilinear"
    geo_cm_coarse_interpolation_method: str = "bicubic"
    geo_cm_fine_acceleration_mode: str = "hybrid"
    geo_cm_coarse_acceleration_mode: str = "gpu"
    geo_cm_apply_smooth_shading: bool = False
    geo_cm_semantic_label: str = "terrain"
    geo_cm_texture_name: str = "LunarRegolith8k"
    geo_cm_texture_path: str = "assets/Textures/LunarRegolith8k.mdl"

    rock_gen_cfgs: list = dataclasses.field(default_factory=list)

    def __post_init__(self):

        assert type(self.seed) == int, "seed must be an integer."
        assert self.seed > 0, "seed must be a positive integer."
        assert type(self.profiling) == bool, "profiling must be a boolean."
        assert type(self.update_every_n_meters) == float, "update_every_n_meters must be a float."
        assert self.update_every_n_meters > 0, "update_every_n_meters must be a positive float."
        assert type(self.z_scale) == float, "z_scale must be a float."
        assert type(self.block_size) == int, "block_size must be an integer."
        assert self.block_size > 0, "block_size must be a positive integer."
        assert type(self.dbs_max_elements) == int, "dbs_max_elements must be an integer."
        assert self.dbs_max_elements > 0, "dbs_max_elements must be a positive integer."
        assert type(self.dbs_save_to_disk) == bool, "dbs_save_to_disk must be a boolean."
        assert type(self.dbs_write_interval) == int, "dbs_write_interval must be an integer."
        assert self.dbs_write_interval > 0, "dbs_write_interval must be a positive integer."
        assert type(self.hr_dem_resolution) == float, "hr_dem_resolution must be a float."
        assert self.hr_dem_resolution > 0, "hr_dem_resolution must be a positive float."
        assert type(self.hr_dem_generate_craters) == bool, "hr_dem_generate_craters must be a boolean."
        assert type(self.hr_dem_num_blocks) == int, "hr_dem_num_blocks must be an integer."
        assert self.hr_dem_num_blocks > 0, "hr_dem_num_blocks must be a positive integer."

        if self.crater_gen_seed is None:
            self.crater_gen_seed = self.seed + 1
        else:
            assert type(self.crater_gen_seed) == int, "crater_gen_seed must be an integer."

        HRDEMCfg_D = {
            "num_blocks": self.hr_dem_num_blocks,
            "block_size": self.block_size,
            "pad_size": self.crater_gen_padding,
            "max_blocks": self.dbs_max_elements,
            "seed": 42,
            "z_scale": self.z_scale,
            "resolution": self.hr_dem_resolution,
            "interpolation_padding": self.hrdem_interpolator_padding,
            "generate_craters": self.hr_dem_generate_craters,
        }
        CraterDBCfg_D = {
            "block_size": self.block_size,
            "max_blocks": self.dbs_max_elements,
            "save_to_disk": self.dbs_save_to_disk,
            "write_to_disk_interval": self.dbs_write_interval,
        }

        assert type(self.crater_gen_densities) == list, "crater_gen_densities must be a list."
        assert len(self.crater_gen_densities) > 0, "crater_gen_densities must have at least one element."
        assert all(
            [type(d) == float for d in self.crater_gen_densities]
        ), "crater_gen_densities must be a list of floats."
        assert all(
            [d > 0 for d in self.crater_gen_densities]
        ), "crater_gen_densities must be a list of positive floats."
        assert type(self.crater_gen_radius) == list, "crater_gen_radius must be a list."
        assert len(self.crater_gen_radius) > 0, "crater_gen_radius must have at least one element."
        assert all(type(r) == list for r in self.crater_gen_radius), "crater_gen_radius must be a list of list."
        assert all(
            len(r) == 2 for r in self.crater_gen_radius
        ), "crater_gen_radius must be a list of tuples of length 2."
        assert all(
            r[0] < r[1] for r in self.crater_gen_radius
        ), "crater_gen_radius must be a list of tuples with the first element smaller than the second."
        assert all(
            (r[0] > 0) for r in self.crater_gen_radius
        ), "crater_gen_radius must be a list of list with the first element greater than 0."
        assert len(self.crater_gen_radius) == len(
            self.crater_gen_densities
        ), "crater_gen_radius and crater_gen_densities must have the same length."
        assert type(self.crater_gen_profiles_path) == str, "crater_gen_profiles_path must be a string."
        assert os.path.exists(self.crater_gen_profiles_path), "crater_gen_profiles_path does not exist."
        assert type(self.crater_gen_padding) == float, "crater_gen_padding must be a float."
        assert self.crater_gen_padding > 0, "crater_gen_padding must be a positive float."
        assert type(self.crater_gen_min_xy_ratio) == float, "crater_gen_min_xy_ratio must be a float."
        assert type(self.crater_gen_max_xy_ratio) == float, "crater_gen_max_xy_ratio must be a float."
        assert self.crater_gen_min_xy_ratio > 0, "crater_gen_min_xy_ratio must be a positive float."
        assert self.crater_gen_max_xy_ratio <= 1, "crater_gen_max_xy_ratio must be smaller or equal to 1."
        assert (
            self.crater_gen_max_xy_ratio > self.crater_gen_min_xy_ratio
        ), "crater_gen_max_xy_ratio must be greater than crater_gen_min_xy_ratio."
        assert type(self.crater_gen_random_rotation) == bool, "crater_gen_random_rotation must be a boolean."
        assert (
            type(self.crater_gen_num_unique_profiles) == int
        ), "crater_gen_rater_gen_num_unique_profiles must be an integer."
        assert (
            self.crater_gen_num_unique_profiles > 0
        ), "crater_gen_rater_gen_num_unique_profiles must be a positive integer."

        if self.crater_gen_metadata_seed is None:
            self.crater_gen_metadata_seed = self.seed + 2
        else:
            assert type(self.crater_gen_metadata_seed) == int, "crater_gen_metadata_seed must be an integer."

        CGCfg_D = {
            "profiles_path": self.crater_gen_profiles_path,
            "min_xy_ratio": self.crater_gen_min_xy_ratio,
            "max_xy_ratio": self.crater_gen_max_xy_ratio,
            "random_rotation": self.crater_gen_random_rotation,
            "seed": self.crater_gen_metadata_seed,
            "num_unique_profiles": self.crater_gen_num_unique_profiles,
        }

        if self.crater_gen_distribution_seed is None:
            self.crater_gen_distribution_seed = self.seed + 3
        else:
            assert type(self.crater_gen_distribution_seed) == int, "crater_gen_distribution_seed must be an integer."

        CDDCfg_D = {
            "densities": self.crater_gen_densities,
            "radius": self.crater_gen_radius,
            "seed": self.crater_gen_distribution_seed,
        }
        CraterSamplerCfg_D = {
            "block_size": self.block_size,
            "crater_gen_cfg": CGCfg_D,
            "crater_dist_cfg": CDDCfg_D,
        }
        CraterBuilderCfg_D = {
            "block_size": self.block_size,
            "pad_size": self.crater_gen_padding,
            "resolution": self.hr_dem_resolution,
            "z_scale": self.z_scale,
        }

        assert type(self.hrdem_interpolation_method) == str, "hrdem_interpolation_method must be a string."
        assert self.hrdem_interpolation_method in [
            "nearest",
            "bilinear",
            "bicubic",
        ], "hrdem_interpolation_method must be either 'nearest', 'bilinear' or 'bicubic'."
        assert type(self.hrdem_interpolator_name) == str, "hrdem_interpolator_name must be a string."
        assert self.hrdem_interpolator_name in [
            "PIL",
            "OpenCV",
        ], "hrdem_interpolator_name must be either 'PIL' or 'OpenCV'."
        assert type(self.hrdem_interpolator_padding) == int, "hrdem_interpolator_padding must be an integer."
        assert self.hrdem_interpolator_padding >= 0, "hrdem_interpolator_padding must be a non-negative integer."

        ICfg = {
            "target_resolution": self.hr_dem_resolution,
            "source_padding": self.hrdem_interpolator_padding,
            "method": self.hrdem_interpolation_method,
        }

        assert type(self.num_workers_craters) == int, "num_workers_craters must be an integer."
        assert self.num_workers_craters > 0, "num_workers_craters must be a positive integer."
        assert type(self.num_workers_interpolation) == int, "num_workers_interpolation must be an integer."
        assert self.num_workers_interpolation > 0, "num_workers_interpolation must be a positive integer."
        assert type(self.input_queue_size) == int, "input_queue_size must be an integer."
        assert self.input_queue_size > 0, "input_queue_size must be a positive integer."
        assert type(self.output_queue_size) == int, "output_queue_size must be an integer."
        assert self.output_queue_size > 0, "output_queue_size must be a positive integer."

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

        HRDEMGenCfg_D = {
            "high_res_dem_cfg": HRDEMCfg_D,
            "crater_db_cfg": CraterDBCfg_D,
            "crater_sampler_cfg": CraterSamplerCfg_D,
            "crater_builder_cfg": CraterBuilderCfg_D,
            "interpolator_cfg": ICfg,
            "crater_worker_manager_cfg": CWMCfg_D,
            "interpolator_worker_manager_cfg": IWMCfg_D,
        }

        assert type(self.lr_dem_folder_path) == str, "lr_dem_folder_path must be a string."
        assert os.path.exists(self.lr_dem_folder_path), "lr_dem_folder_path does not exist."
        assert type(self.lr_dem_name) == str, "lr_dem_name must be a string."

        self.MMConf_D = {
            "folder_path": self.lr_dem_folder_path,
            "hrdem_settings": HRDEMGenCfg_D,
        }

        assert type(self.geo_cm_num_texels_per_level) == int, "geo_cm_num_texels_per_level must be an integer."
        assert self.geo_cm_num_texels_per_level > 0, "geo_cm_num_texels_per_level must be a positive integer."
        assert type(self.geo_cm_target_res) == float, "geo_cm_target_res must be a float."
        assert self.geo_cm_target_res > 0, "geo_cm_target_res must be a positive float."
        assert type(self.geo_cm_fine_interpolation_method) == str, "geo_cm_fine_interpolation_method must be a string."
        assert self.geo_cm_fine_interpolation_method in [
            "bilinear",
            "bicubic",
        ], "geo_cm_fine_interpolation_method must be either 'bilinear' or 'bicubic'."
        assert (
            type(self.geo_cm_coarse_interpolation_method) == str
        ), "geo_cm_coarse_interpolation_method must be a string."
        assert self.geo_cm_coarse_interpolation_method in [
            "bilinear",
            "bicubic",
        ], "geo_cm_coarse_interpolation_method must be either 'bilinear' or 'bicubic'."
        assert type(self.geo_cm_fine_acceleration_mode) == str, "geo_cm_fine_acceleration_mode must be a string."
        assert self.geo_cm_fine_acceleration_mode in [
            "hybrid",
            "gpu",
        ], "geo_cm_fine_acceleration_mode must be either 'hybrid' or 'gpu'."
        assert type(self.geo_cm_coarse_acceleration_mode) == str, "geo_cm_coarse_acceleration_mode must be a string."
        assert self.geo_cm_coarse_acceleration_mode in [
            "hybrid",
            "gpu",
        ], "geo_cm_coarse_acceleration_mode must be either 'hybrid' or 'gpu'."
        assert type(self.geo_cm_apply_smooth_shading) == bool, "geo_cm_apply_smooth_shading must be a boolean."

        self.NGCMMConf_D = {
            "num_texels_per_level": self.geo_cm_num_texels_per_level,
            "target_res": self.geo_cm_target_res,
            "fine_interpolation_method": self.geo_cm_fine_interpolation_method,
            "coarse_interpolation_method": self.geo_cm_coarse_interpolation_method,
            "fine_acceleration_mode": self.geo_cm_fine_acceleration_mode,
            "coarse_acceleration_mode": self.geo_cm_coarse_acceleration_mode,
            "profiling": self.profiling,
            "semantic_label": self.geo_cm_semantic_label,
            "texture_name": self.geo_cm_texture_name,
            "texture_path": self.geo_cm_texture_path,
        }

        self.RDBConf_D = {
            "block_size": self.block_size,
            "max_blocks": self.dbs_max_elements,
            "save_to_disk": self.dbs_save_to_disk,
            "write_to_disk_interval": self.dbs_write_interval,
        }

        if self.rock_gen_main_seed is None:
            self.rock_gen_main_seed = self.seed + 4
        else:
            assert type(self.rock_gen_main_seed) == int, "rock_gen_main_seed must be an integer."

        self.RMConf_D = {
            "rock_gen_cfgs": self.rock_gen_cfgs,
            "rock_dbs_cfg": self.RDBConf_D,
            "instancers_path": "/World/rock_instancers",
            "seed": self.rock_gen_main_seed,
            "profiling": self.profiling,
            "block_size": self.block_size,
        }

        assert type(self.starting_position) == tuple, "starting_position must be a tuple."
        assert len(self.starting_position) == 2, "starting_position must be a tuple of length 2."

        self.LSTMCfg_D = {
            "map_name": self.lr_dem_name,
            "meters_coordinates": self.starting_position,
            "visual_mesh_update_threshold": self.update_every_n_meters,
            "profiling": self.profiling,
        }
