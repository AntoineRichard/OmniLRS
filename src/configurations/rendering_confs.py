import dataclasses

@dataclasses.dataclass
class FlaresConf:
    scale: float = dataclasses.field(default_factory=float)
    blades: int = dataclasses.field(default_factory=int)
    aperture_rotation: float = dataclasses.field(default_factory=float)
    sensor_diagonal: float = dataclasses.field(default_factory=float)
    sensor_aspect_ratio: float = dataclasses.field(default_factory=float)
    fstop: float = dataclasses.field(default_factory=float)
    focal_length: float = dataclasses.field(default_factory=float)

@dataclasses.dataclass
class PathTracedRenderConf:
    samples_per_pixel_per_frame: int = dataclasses.field(default_factory=int)
    max_bounces: int = dataclasses.field(default_factory=int)
    max_specular_transmission_bounces: int = dataclasses.field(default_factory=int)
    max_volume_bounces: int = dataclasses.field(default_factory=int)
    subdiv_refinement_level: int = dataclasses.field(default_factory=int)
    renderer: str = "PathTracing"
    headless: bool = dataclasses.field(default_factory=bool)

@dataclasses.dataclass
class RayTracedRenderConf:
    renderer: str = "RayTracedLighting"
    headless: bool = dataclasses.field(default_factory=bool)