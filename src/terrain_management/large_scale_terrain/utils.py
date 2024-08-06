__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from scipy.interpolate import CubicSpline
from typing import Tuple
import dataclasses
import time


@dataclasses.dataclass
class BoundingBox:
    x_min: float = 0
    x_max: float = 0
    y_min: float = 0
    y_max: float = 0


@dataclasses.dataclass
class CraterMetadata:
    radius: float = 0.0
    coordinates: Tuple[int, int] = (0, 0)
    deformation_spline_id: CubicSpline = None
    marks_spline_id: CubicSpline = None
    marks_intensity: float = 0
    crater_profile_id: int = 0
    xy_deformation_factor: Tuple[float, float] = (0, 0)
    rotation: float = 0

    def get_memory_footprint(self) -> int:
        return self.size


class ScopedTimer:
    def __init__(self, name="", color=0xFFFF5733, active=True):
        self.active = active
        self.name = name
        if color:
            self.rgb_color = self.argb_to_rgb(color)
            self.ansi_color = self.rgb_to_ansi(self.rgb_color)
        else:
            self.ansi_color = ""

    def __enter__(self):
        if self.active:
            self.start_time = time.time()
        return self

    def argb_to_rgb(self, argb):
        # Extract RGB components from ARGB value
        rgb = (argb >> 16) & 0xFFFFFF
        return (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF

    def rgb_to_ansi(self, rgb):
        # Convert RGB to an ANSI escape code for colored text
        return f"\033[38;2;{rgb[0]};{rgb[1]};{rgb[2]}m"

    def __exit__(self, exc_type, exc_value, traceback):
        if self.active:
            self.end_time = time.time()
            elapsed_time = self.end_time - self.start_time
            reset_color = "\033[0m"
            print(
                f"{self.ansi_color}{self.name} took: {elapsed_time:.4f}s.{reset_color}"
            )
