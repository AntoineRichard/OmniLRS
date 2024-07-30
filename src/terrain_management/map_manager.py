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
    resolution: tuple = dataclasses.field(default_factory=tuple)
    coordinates: tuple = dataclasses.field(default_factory=tuple)


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
            if os.path.isdir(folder):
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
                    self.dem_infos[folder] = DemInfo(
                        **yaml.load(open(dem_info_path, "r"))
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

    def load_hr_dem_by_id(self, id: int):
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
        pass

    def update_hr_dem(self, coordinates):
        self.hr_dem_gen.update_high_res_dem(coordinates)

    def initialize_hr_dem(self, coordinates):
        print("Initializing HR DEM")
        start = time.time()
        while not self.is_hr_dem_updated():
            self.update_hr_dem(coordinates)
        end = time.time()
        print(f"HR DEM initialized in {end - start} seconds")

    def is_hr_dem_updated(self):
        return self.hr_dem_gen.is_map_done()


if __name__ == "__main__":
    HRDEMGCfg_D = {
        "num_blocks": 7,  # int = dataclasses.field(default_factory=int)
        "block_size": 50,  # float = dataclasses.field(default_factory=float)
        "pad_size": 10.0,  # float = dataclasses.field(default
        "max_blocks": int(1e7),  # int = dataclasses.field(default_factory=int)
        "seed": 42,  # int = dataclasses.field(default_factory=int)
        "resolution": 0.05,  # float = dataclasses.field(default_factory=float)
        "z_scale": 1.0,  # float = dataclasses.field(default_factory=float)
        "radius": [
            [1.5, 2.5],
            [0.75, 1.5],
            [0.25, 0.5],
        ],  # List[Tuple[float, float]] = dataclasses.field(default_factory=list)
        "densities": [
            0.025,
            0.05,
            0.5,
        ],  # List[float] = dataclasses.field(default_factory=list)
        "num_repeat": 1,  # int = dataclasses.field(default_factory=int)
        "save_to_disk": False,  # bool = dataclasses.field(default_factory=bool)
        "write_to_disk_interval": 100,  # int = dataclasses.field(default_factory=int)
        "profiles_path": "assets/Terrains/crater_spline_profiles.pkl",  # str = dataclasses.field(default_factory=str)
        "min_xy_ratio": 0.85,  # float = dataclasses.field(default_factory=float)
        "max_xy_ratio": 1.0,  # float = dataclasses.field(default_factory=float)
        "random_rotation": True,  # bool = dataclasses.field(default_factory=bool)
        "num_unique_profiles": 10000,  # int = dataclasses.field(default_factory=int)
        "source_resolution": 5.0,  # float = dataclasses.field(default_factory=float)
        # "target_resolution": 0.05,  # float = dataclasses.field(default_factory=float)
        "interpolation_padding": 2,  # int = dataclasses.field(default_factory=int)
        "interpolation_method": "bicubic",  # str = dataclasses.field(default_factory=str)
    }

    hrdem_settings = HighResDEMGenCfg(**HRDEMGCfg_D)

    MMCfg_D = {
        "folder_path": "assets/Terrains",
        "lr_dem_name": "crater",
        "initial_pixel_coordinates": (2000, 2000),
    }

    mm_settings = MapManagerCfg(**MMCfg_D)

    MM = MapManager(hrdem_settings, mm_settings)
    MM.load_lr_dem_by_name("NPD_final_adj_5mpp_surf")

    MM.