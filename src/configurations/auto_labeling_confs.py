import dataclasses

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