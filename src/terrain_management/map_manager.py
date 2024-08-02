# The map manager is handling the LR and HR maps.
# The map manager is responsible for loading and unloading the maps.

# The map manager can handle fully prodedural maps or DEM-based maps.

# The map manager applies curvatrue e.g. for the moon based on the radius of the moon.
# The map manager is used by the terrain renderer to generate the Geoclipmaps.
# The map manager is used by the terrain collider to generate the collider meshes.

import dataclasses
import numpy as np
import warnings
import yaml
import time
import os

from high_res_dem_gen import HighResDEMGen
from high_res_dem_gen import HighResDEMGenCfg


@dataclasses.dataclass
class DemInfo:
    size: tuple = dataclasses.field(default_factory=tuple)
    pixel_size: tuple = dataclasses.field(default_factory=tuple)
    center_coordinates: tuple = dataclasses.field(default_factory=tuple)


@dataclasses.dataclass
class MapManagerCfg:
    folder_path: str = dataclasses.field(default_factory=str)
    lr_dem_name: str = dataclasses.field(default_factory=str)
    initial_pixel_coordinates: tuple = dataclasses.field(default_factory=tuple)


class MapManager:
    def __init__(
        self, hrdem_settings: HighResDEMGenCfg, map_manager_settings: MapManagerCfg
    ):
        self.hr_dem_settings = hrdem_settings
        self.settings = map_manager_settings
        self.lr_dem = None

        self.fetch_pregenerated_lr_dems()

    def fetch_pregenerated_lr_dems(self):
        assert os.path.isdir(self.settings.folder_path), "Folder path does not exist"
        self.dem_paths = {}
        self.dem_infos = {}
        for folder in os.listdir(self.settings.folder_path):
            if os.path.isdir(os.path.join(self.settings.folder_path, folder)):
                dem_path = os.path.join(self.settings.folder_path, folder, "dem.npy")
                dem_info_path = os.path.join(
                    self.settings.folder_path, folder, "dem.yaml"
                )
                if os.path.exists(dem_path):
                    self.dem_paths[folder] = dem_path
                else:
                    warnings.warn(
                        f"DEM {dem_path} does not exist. Expected to find dem.npy in the folder but could not find it."
                    )
                if os.path.exists(dem_info_path):
                    with open(dem_info_path, "r") as file:
                        self.dem_infos[folder] = DemInfo(
                            **yaml.load(file, Loader=yaml.Loader)
                        )
                else:
                    warnings.warn(
                        f"DEM info {dem_info_path} does not exist. Expected to find dem.yaml in the folder but could not find it."
                    )
            else:
                warnings.warn(f"Folder {folder} is not a directory.")
        pass

    def load_lr_dem_by_name(self, name: str):
        if name in self.dem_paths:
            self.lr_dem = np.load(self.dem_paths[name])
            if name in self.dem_infos:
                self.lr_dem_info = self.dem_infos[name]
            else:
                raise ValueError(
                    f"DEM info {name} does not exist in the folder path {self.settings.folder_path}"
                )
        else:
            warnings.warn(
                f"DEM {name} does not exist in the folder path {self.settings.folder_path}"
            )
        self.hr_dem_gen = HighResDEMGen(self.lr_dem, self.hr_dem_settings)

    def load_lr_dem_by_path(self, path: str):
        if os.path.exists(path):
            dem_path = os.path.join(path, "dem.npy")
            dem_info_path = os.path.join(path, "dem.yaml")
            if os.path.exists(dem_path) and os.path.exists(dem_info_path):
                self.lr_dem = np.load(dem_path)
                self.lr_dem_info = DemInfo(**yaml.load(open(dem_info_path, "r")))
            else:
                raise ValueError(
                    f"DEM {dem_path} or DEM info {dem_info_path} does not exist in the folder path {path}"
                )
        self.hr_dem_gen = HighResDEMGen(self.lr_dem, self.hr_dem_settings)

    def load_lr_dem_by_id(self, id: int):
        if id in list(range(len(self.dem_paths.keys()))):
            key = list(self.dem_paths.keys())[id]
            self.hr_dem = np.load(self.dem_paths[key])
            self.hr_dem_info = self.dem_infos[key]
        elif id == -1:
            self.generate_procedural_lr_dem()
        else:
            raise ValueError(
                f"id {id} is not in the range of the number of DEMs in the folder path {self.settings.folder_path}"
            )
        self.hr_dem_gen = HighResDEMGen(self.lr_dem, self.hr_dem_settings)

    def generate_procedural_lr_dem(self):
        raise NotImplementedError

    def get_lr_dem(self):
        return self.lr_dem

    def get_hr_dem(self):
        return self.hr_dem_gen.high_res_dem

    def get_hr_dem_mask(self):
        raise NotImplementedError

    def update_hr_dem(self, coordinates):
        self.hr_dem_gen.update_high_res_dem(coordinates)

    def initialize_hr_dem(self, coordinates):
        print("Initializing HR DEM")
        start = time.time()
        self.update_hr_dem(coordinates)
        while not self.is_hr_dem_updated():
            time.sleep(0.1)
        end = time.time()
        print(f"HR DEM initialized in {end - start} seconds")

    def is_hr_dem_updated(self):
        return self.hr_dem_gen.is_map_done()


if __name__ == "__main__":
    HRDEMCfg_D = {
        "num_blocks": 4,
        "block_size": 50,
        "pad_size": 10.0,
        "max_blocks": int(1e7),
        "seed": 42,
        "resolution": 0.05,
        "z_scale": 1.0,
        "source_resolution": 5.0,
        "resolution": 0.05,
        "interpolation_padding": 2,
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
        "resolution": 0.05,
        "z_scale": 1.0,
    }
    ICfg = {
        "source_resolution": 5.0,
        "target_resolution": 0.05,
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

    hrdem_settings = HighResDEMGenCfg(**HRDEMGenCfg_D)

    MMCfg_D = {
        "folder_path": "assets/Terrains/SouthPole",
        "lr_dem_name": "crater",
        "initial_pixel_coordinates": (2000, 2000),
    }

    mm_settings = MapManagerCfg(**MMCfg_D)
    from matplotlib import pyplot as plt

    low_res_dem = np.load("assets/Terrains/SouthPole/NPD_final_adj_5mpp_surf/dem.npy")
    HRDEMGen = HighResDEMGen(low_res_dem, hrdem_settings)

    MM = MapManager(hrdem_settings, mm_settings)
    MM.load_lr_dem_by_name("NPD_final_adj_5mpp_surf")
    MM.initialize_hr_dem((0, 0))
    plt.figure()
    plt.imshow(MM.hr_dem_gen.high_res_dem, cmap="jet")
    plt.figure()
    plt.imshow(MM.lr_dem, cmap="jet")
    plt.show()
    MM.hr_dem_gen.shutdown()
