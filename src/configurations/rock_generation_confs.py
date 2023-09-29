import dataclasses
from WorldBuilders.Types import Sampler_T, Layer_T


@dataclasses.dataclass
class RockGenerationConf:
    instancers_path: str = dataclasses.field(default_factory=str)
    rocks_settings: dict = dataclasses.field(default_factory=dict)