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
class CraterGeneratorConf:
    profiles_path: str = dataclasses.field(default_factory=str)
    min_xy_ratio: float = dataclasses.field(default_factory=float)
    max_xy_ratio: float = dataclasses.field(default_factory=float)
    resolution: float = dataclasses.field(default_factory=float)
    pad_size: int = dataclasses.field(default_factory=int)
    random_rotation: bool = dataclasses.field(default_factory=bool)
    z_scale: float = dataclasses.field(default_factory=float)
    seed: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        print(self.__dict__)
        assert type(self.profiles_path) is str, "profile_path must be a string"
        assert type(self.min_xy_ratio) is float, "min_xy_ratio must be a float"
        assert type(self.max_xy_ratio) is float, "max_xy_ratio must be a float"
        assert type(self.resolution) is float, "resolution must be a float"
        assert type(self.pad_size) is int, "pad_size must be an integer"
        assert type(self.random_rotation) is bool, "random_rotation must be a boolean"
        assert type(self.z_scale) is float, "z_scale must be a float"
        assert type(self.seed) is int, "seed must be an integer"

        assert self.min_xy_ratio <= self.max_xy_ratio, "min_xy_ratio must be smaller than max_xy_ratio"
        assert self.min_xy_ratio > 0, "min_xy_ratio must be greater than 0"
        assert self.max_xy_ratio > 0, "max_xy_ratio must be greater than 0"
        assert self.min_xy_ratio <= 1, "min_xy_ratio must be smaller than 1"
        assert self.max_xy_ratio <= 1, "max_xy_ratio must be smaller than 1"
        assert self.resolution > 0, "resolution must be greater than 0"
        assert self.pad_size >= 0, "pad_size must be greater or equal to 0"
        assert self.z_scale > 0, "z_scale must be greater than 0"

@dataclasses.dataclass
class CraterDistributionConf:
    x_size: float = dataclasses.field(default_factory=float)
    y_size: float = dataclasses.field(default_factory=float)
    densities: list = dataclasses.field(default_factory=list)
    radius: list = dataclasses.field(default_factory=list)
    num_repeat: int = dataclasses.field(default_factory=int)
    seed: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        assert type(self.x_size) is float, "x_size must be a float"
        assert type(self.y_size) is float, "y_size must be a float"
        assert type(self.num_repeat) is int, "num_repeat must be an integer"
        assert type(self.seed) is int, "seed must be an integer"

        assert self.x_size > 0, "x_size must be greater than 0"
        assert self.y_size > 0, "y_size must be greater than 0"
        assert len(self.densities) == len(self.radius), "densities and radius must have the same length"
        assert self.num_repeat >= 0, "num_repeat must be greater or equal to 0"

@dataclasses.dataclass
class BaseTerrainGeneratorConf:
    x_size: float = dataclasses.field(default_factory=float)
    y_size: float = dataclasses.field(default_factory=float)
    resolution: float = dataclasses.field(default_factory=float)
    max_elevation: float = dataclasses.field(default_factory=float)
    min_elevation: float = dataclasses.field(default_factory=float)
    seed: int = dataclasses.field(default_factory=int)
    z_scale: float = dataclasses.field(default_factory=float)

    def __post_init__(self):
        assert type(self.x_size) is float, "x_size must be a float"
        assert type(self.y_size) is float, "y_size must be a float"
        assert type(self.resolution) is float, "resolution must be a float"
        assert type(self.max_elevation) is float, "max_elevation must be a float"
        assert type(self.min_elevation) is float, "min_elevation must be a float"
        assert type(self.seed) is int, "seed must be an integer"
        assert type(self.z_scale) is float, "z_scale must be a float"

        assert self.x_size > 0, "x_size must be greater than 0"
        assert self.y_size > 0, "y_size must be greater than 0"
        assert self.resolution > 0, "resolution must be greater than 0"
        assert self.max_elevation > self.min_elevation, "max_elevation must be greater than min_elevation"
        assert self.z_scale > 0, "z_scale must be greater than 0"

@dataclasses.dataclass
class MoonYardConf:
    crater_generator: CraterGeneratorConf = None
    crater_distribution: CraterDistributionConf = None
    base_terrain_generator: BaseTerrainGeneratorConf = None
    is_yard: bool = dataclasses.field(default_factory=bool)
    is_lab: bool = dataclasses.field(default_factory=bool)

    def __post_init__(self):
        self.crater_generator = CraterGeneratorConf(**self.crater_generator)
        self.crater_distribution = CraterDistributionConf(**self.crater_distribution)
        self.base_terrain_generator = BaseTerrainGeneratorConf(**self.base_terrain_generator)

        assert type(self.is_yard) is bool, "is_yard must be a boolean"
        assert type(self.is_lab) is bool, "is_lab must be a boolean"

@dataclasses.dataclass
class TerrainManagerConf:
    moon_yard: MoonYardConf = None
    root_path: str = dataclasses.field(default_factory=str)
    texture_path: str = dataclasses.field(default_factory=str)
    dems_path: str = dataclasses.field(default_factory=str)
    mesh_position: tuple = dataclasses.field(default_factory=tuple)
    mesh_orientation: tuple = dataclasses.field(default_factory=tuple)
    mesh_scale: tuple = dataclasses.field(default_factory=tuple)
    sim_length: int = dataclasses.field(default_factory=int)
    sim_width: int = dataclasses.field(default_factory=int)
    resolution: float = dataclasses.field(default_factory=float)

    def __post_init__(self):
        self.moon_yard = MoonYardConf(**self.moon_yard)

        assert type(self.root_path) is str, "root_path must be a string"
        assert type(self.texture_path) is str, "texture_path must be a string"
        assert type(self.dems_path) is str, "dems_path must be a string"
        assert type(self.sim_length) is float, "sim_length must be a float"
        assert type(self.sim_width) is float, "sim_width must be a float"
        assert type(self.resolution) is float, "resolution must be a float"

        assert len(self.mesh_position) == 3, "mesh_position must be a tuple of length 3"
        assert len(self.mesh_orientation) == 4, "mesh_orientation must be a tuple of length 4"
        assert len(self.mesh_scale) == 3, "mesh_scale must be a tuple of length 3"
        assert self.sim_length > 0, "sim_length must be greater than 0"
        assert self.sim_width > 0, "sim_width must be greater than 0"
        assert self.resolution > 0, "resolution must be greater than 0"
        