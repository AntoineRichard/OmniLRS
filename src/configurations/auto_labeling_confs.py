__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

import dataclasses
from typing import Any

@dataclasses.dataclass
class AutoLabelingConf:
    prim_path: str = dataclasses.field(default_factory=str)
    camera_name: str = dataclasses.field(default_factory=str)
    camera_resolution: tuple = dataclasses.field(default_factory=tuple)
    data_dir: str = dataclasses.field(default_factory=str)
    annotator_list: list = dataclasses.field(default_factory=list)
    image_format: str = dataclasses.field(default_factory=str)
    annot_format: str = dataclasses.field(default_factory=str)
    element_per_folder: int = dataclasses.field(default_factory=int)
    add_noise_to_rgb: bool = dataclasses.field(default_factory=bool)
    sigma: float = dataclasses.field(default_factory=float)
    seed: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        assert self.prim_path is str, "prim_path must be a string"
        assert self.camera_name is str, "camera_name must be a string"
        assert self.camera_resolution is tuple, "camera_resolution must be a tuple"
        assert len(self.camera_resolution) == 2, "camera_resolution must be a tuple of length 2"
        assert self.data_dir is str, "data_dir must be a string"
        assert self.annotator_list is list, "annotator_list must be a list"
        assert self.image_format is str, "image_format must be a string"
        assert self.annot_format is str, "annot_format must be a string"
        assert self.element_per_folder > 0, "element_per_folder must be greater than 0"
        assert self.element_per_folder is int, "element_per_folder must be an integer"
        assert self.add_noise_to_rgb is bool, "add_noise_to_rgb must be a boolean"
        assert self.sigma > 0, "sigma must be greater than 0"
        assert self.sigma is float, "sigma must be a float"
        assert self.seed is int, "seed must be an integer"