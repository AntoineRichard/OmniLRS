__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

import dataclasses


@dataclasses.dataclass
class LunalabConf:
    lab_length: float = 10.0
    lab_width: float = 6.5
    resolution: float = 0.01
    projector_path: str = "/Lunalab/Projector"
    projector_shader_path: str = "/Lunalab/Looks/Light_12000K/Shader"
    room_lights_path: str = "/Lunalab/CeilingLights"
    curtains_path: dict = dataclasses.field(default_factory=dict)

    def __post_init__(self):
        assert type(self.lab_length) == float, "The lab length must be a float."
        assert type(self.lab_width) == float, "The lab width must be a float."
        assert type(self.resolution) == float, "The resolution must be a float."
        assert type(self.projector_path) == str, "The projector path must be a string."
        assert (
            type(self.projector_shader_path) == str
        ), "The projector shader path must be a string."
        assert (
            type(self.room_lights_path) == str
        ), "The room lights path must be a string."

        assert self.lab_length > 0.0, "The lab length must be greater than 0."
        assert self.lab_width > 0.0, "The lab width must be greater than 0."
        assert self.resolution > 0.0, "The resolution must be greater than 0."
        assert self.projector_path != "", "The projector path must not be empty."
        assert (
            self.projector_shader_path != ""
        ), "The projector shader path must not be empty."
        assert self.room_lights_path != "", "The room lights path must not be empty."
        self.curtains_path = {
            "left": "/Lunalab/CurtainLeft",
            "right": "/Lunalab/CurtainRight",
        }


@dataclasses.dataclass
class LunaryardConf:
    lab_length: float = 20.0
    lab_width: float = 20.0
    resolution: float = 0.025
    projector_path: str = "/Lunaryard/Sun"
    earth_path: str = "/Lunaryard/Earth"

    def __post_init__(self):
        assert type(self.lab_length) == float, "The lab length must be a float."
        assert type(self.lab_width) == float, "The lab width must be a float."
        assert type(self.resolution) == float, "The resolution must be a float."
        assert type(self.projector_path) == str, "The projector path must be a string."

        assert self.lab_length > 0.0, "The lab length must be greater than 0."
        assert self.lab_width > 0.0, "The lab width must be greater than 0."
        assert self.resolution > 0.0, "The resolution must be greater than 0."
        assert self.projector_path != "", "The projector path must not be empty."
