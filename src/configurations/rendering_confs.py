__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

import dataclasses

@dataclasses.dataclass
class FlaresConf:
    enable: bool = dataclasses.field(default_factory=bool)
    scale: float = dataclasses.field(default_factory=float)
    blades: int = dataclasses.field(default_factory=int)
    aperture_rotation: float = dataclasses.field(default_factory=float)
    sensor_diagonal: float = dataclasses.field(default_factory=float)
    sensor_aspect_ratio: float = dataclasses.field(default_factory=float)
    fstop: float = dataclasses.field(default_factory=float)
    focal_length: float = dataclasses.field(default_factory=float)

    def __post_init__(self):
        assert type(self.enable) is bool, "enalbe must be a boolean"
        assert type(self.scale) is float, "scale must be a float"
        assert type(self.blades) is int, "blades must be an integer"
        assert type(self.aperture_rotation) is float, "aperture_rotation must be a float"
        assert type(self.sensor_diagonal) is float, "sensor_diagonal must be a float"
        assert type(self.sensor_aspect_ratio) is float, "sensor_aspect_ratio must be a float"
        assert type(self.fstop) is float, "fstop must be a float"
        assert type(self.focal_length) is float, "focal_length must be a float"

        assert self.scale > 0, "scale must be greater than 0"
        assert self.blades > 0, "blades must be greater than 0"
        assert self.aperture_rotation >= 0, "aperture_rotation must be greater or equal to 0"
        assert self.aperture_rotation <= 360, "aperture_rotation must be smaller or equal to 360"
        assert self.sensor_diagonal > 0, "sensor_diagonal must be greater than 0"
        assert self.sensor_aspect_ratio > 0, "sensor_aspect_ratio must be greater than 0"
        assert self.fstop > 0, "fstop must be greater than 0"
        assert self.focal_length > 0, "focal_length must be greater than 0"

@dataclasses.dataclass
class RendererConf:
    samples_per_pixel_per_frame: int = dataclasses.field(default_factory=int)
    max_bounces: int = dataclasses.field(default_factory=int)
    max_specular_transmission_bounces: int = dataclasses.field(default_factory=int)
    max_volume_bounces: int = dataclasses.field(default_factory=int)
    subdiv_refinement_level: int = dataclasses.field(default_factory=int)
    renderer: str = dataclasses.field(default_factory=str)
    headless: bool = dataclasses.field(default_factory=bool)

    def __post_init__(self):
        assert type(self.samples_per_pixel_per_frame) is int, "samples_per_pixel_per_frame must be an integer"
        assert type(self.max_bounces) is int, "max_bounces must be an integer"
        assert type(self.max_specular_transmission_bounces) is int, "max_specular_transmission_bounces must be an integer"
        assert type(self.max_volume_bounces) is int, "max_volume_bounces must be an integer"
        assert type(self.subdiv_refinement_level) is int, "subdiv_refinement_level must be an integer"
        assert type(self.headless) is bool, "headless must be a boolean"
        assert type(self.renderer) is str, "renderer must be a string"

        assert self.samples_per_pixel_per_frame > 0, "samples_per_pixel_per_frame must be greater than 0"
        assert self.max_bounces > 0, "max_bounces must be greater than 0"
        assert self.max_specular_transmission_bounces > 0, "max_specular_transmission_bounces must be greater than 0"
        assert self.max_volume_bounces > 0, "max_volume_bounces must be greater than 0"
        assert self.subdiv_refinement_level >= 0, "subdiv_refinement_level must be greater or equal to 0"
        assert self.renderer in ["PathTracing", "RayTracedLighting"], "renderer must be PATH, PathTracing or RayTracedLighting"