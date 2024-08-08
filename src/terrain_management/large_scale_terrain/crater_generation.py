__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from matplotlib import pyplot as plt
from scipy.ndimage import rotate
from typing import Tuple, List
import numpy as np
import dataclasses
import cv2

from src.terrain_management.large_scale_terrain.crater_distribution import (
    CraterSampler,
    CraterSamplerCfg,
)
from src.terrain_management.large_scale_terrain.crater_database import (
    CraterDB,
    CraterDBCfg,
)
from src.terrain_management.large_scale_terrain.utils import CraterMetadata, BoundingBox


@dataclasses.dataclass
class CraterBuilderCfg:
    """
    Args:
        block_size (int): size of the block in meters.
        pad_size (int): size of the padding in meters.
        resolution (float): resolution of the DEM in pixel per meters.
        z_scale (float): scale of the z axis.
    """

    block_size: int = 50
    pad_size: int = 10
    resolution: float = dataclasses.field(default_factory=float)
    z_scale: float = dataclasses.field(default_factory=float)


class CraterBuilder:
    """
    Class to generate an image containing craters.
    It can be added to a DEM to create a more realistic terrain.
    """

    def __init__(self, settings: CraterBuilderCfg, db: CraterDB):
        self.settings = settings
        self.db = db

    @staticmethod
    def sat_gaussian(x: np.ndarray, mu1: float, mu2: float, std: float) -> np.ndarray:
        """
        Saturates a gaussian function to its maximum between mu1 and mu2 with a standard deviation
        of std.

        Args:
            x (np.ndarray): input array.
            mu1 (float): gaussian mu lower bound.
            mu2 (float): gaussian mu upper bound.
            std (float): standard deviation.

        Returns:
            np.ndarray: saturated gaussian.
        """

        shape = x.shape
        x = x.flatten()
        x[x < mu1] = np.exp(-0.5 * ((x[x < mu1] - mu1) / std) ** 2)
        x[x > mu2] = np.exp(-0.5 * ((x[x > mu2] - mu2) / std) ** 2)
        x[(x >= mu1) & (x <= mu2)] = 1.0
        x = x / (std * np.sqrt(2 * np.pi))
        x = x.reshape(shape)
        return x

    def centeredDistanceMatrix(self, crater_metadata: CraterMetadata) -> np.ndarray:
        """
        Generates a distance matrix centered at the center of the matrix.

        Args:
            crater_metadata (CraterMetadata): data used to generate the crater.

        Returns:
            np.ndarry: distance matrix.
        """

        size = int(crater_metadata.radius * 2 / self.settings.resolution)
        size = size + size % 2

        deformation_spline = self.db.get_deformation_spline(
            crater_metadata.deformation_spline_id
        )
        marks_spline = self.db.get_marks_spline(crater_metadata.marks_spline_id)

        # Generates the deformation matrix
        m = np.zeros([size, size])
        x, y = np.meshgrid(np.linspace(-1, 1, size), np.linspace(-1, 1, size))
        theta = np.arctan2(y, x)
        fac = deformation_spline(theta / (2 * np.pi) + 0.5)

        # Generates the marks matrix
        marks = (
            marks_spline(theta / (2 * np.pi) + 0.5)
            * size
            / 2
            * crater_metadata.marks_intensity
        )

        # Generates the distance matrix
        x, y = np.meshgrid(range(size), range(size))
        m = np.sqrt(
            ((x - (size / 2) + 1) / crater_metadata.xy_deformation_factor[0]) ** 2
            + (y - (size / 2) + 1) ** 2
        )

        # Deforms the distance matrix
        m = m * fac

        # Smoothes the marks such that they are not present on the outer edge of the crater
        sat = self.sat_gaussian(
            m,
            0.15 * size / 2,
            0.45 * size / 2,
            0.05 * size / 2,
        )
        sat = (sat - sat.min()) / (sat.max() - sat.min())

        # Adds the marks to the distance matrix
        m = m + marks * sat

        # Rotate the matrix
        m = rotate(m, crater_metadata.rotation, reshape=False, cval=size / 2)

        # Saturate the matrix such that the distance is not greater than the radius of the crater
        m[m > size / 2] = size / 2
        return m, size

    def applyProfile(
        self, distance: np.ndarray, crater_metadata: CraterMetadata, size: float
    ) -> np.ndarray:
        """
        Applies a profile to the distance matrix.

        Args:
            distance (np.ndarray): distance matrix.
            crater_metadata (CraterMetadata): data used to generate the crater
            size (float): size of the matrix in meters.

        Returns:
            np.ndarray: crater DEM.
        """

        profile_spline = self.db.get_crater_profile_spline(
            crater_metadata.crater_profile_id
        )
        crater = profile_spline(2 * distance / size)
        return crater

    def generateCrater(self, crater_metadata: CraterMetadata) -> np.ndarray:
        """
        Generates a crater DEM.

        Args:
            crater_metadata (CraterMetadata): data used to generate the crater.

        Returns:
            tuple: crater DEM, an image containing all the craters from the metadata.
        """

        distance, size = self.centeredDistanceMatrix(crater_metadata)

        crater = (
            self.applyProfile(distance, crater_metadata, size)
            * size
            / 2.0
            * self.settings.z_scale
            * self.settings.resolution
        )
        return crater

    def generateCraters(
        self,
        craters_data: List[CraterMetadata],
        coords: Tuple[int, int],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generates a DEM with craters.

        Args:
            craters_data (List[CraterMetadata]): list of craters metadata.
            coords (Tuple[int, int]): coordinates of the block.

        Returns:
            Tuple[np.ndarray, np.ndarray]: DEM with craters, mask.
        """

        # Creates a padded DEM and mask
        dem_size = int(self.settings.block_size / self.settings.resolution)
        pad_size = int(self.settings.pad_size / self.settings.resolution)
        DEM_padded = np.zeros((pad_size * 2 + dem_size, pad_size * 2 + dem_size))
        mask_padded = np.ones_like(DEM_padded)
        coords_np = np.array(coords)
        # Adds the craters to the DEM
        for crater_data in craters_data:
            # rad = int(crater_data.size * 2 / self._resolution)
            # coord = coord / self._resolution
            c = self.generateCrater(crater_data)
            size = c.shape[0]
            coord = (
                np.array(crater_data.coordinates - coords_np) / self.settings.resolution
            )
            coord_padded = (coord + pad_size).astype(np.int64)
            coord_offset = (coord_padded - size / 2).astype(np.int64)
            DEM_padded[
                coord_offset[0] : coord_offset[0] + size,
                coord_offset[1] : coord_offset[1] + size,
            ] += c
            mask_padded = cv2.circle(
                mask_padded,
                (coord_padded[1], coord_padded[0]),
                int(size / 4),
                0,
                -1,
            )

        return (
            DEM_padded[pad_size:-pad_size, pad_size:-pad_size],
            mask_padded[pad_size:-pad_size, pad_size:-pad_size],
        )


if __name__ == "__main__":

    CDB_CFG_D = {
        "block_size": 50,
        "max_blocks": int(1e7),
        "save_to_disk": False,
        "write_to_disk_interval": 1000,
    }
    CDB_CFG = CraterDBCfg(**CDB_CFG_D)
    CDB = CraterDB(CDB_CFG)

    CDDC_CFG_D = {
        "densities": [0.025, 0.05, 0.5],
        "radius": [[1.5, 2.5], [0.75, 1.5], [0.25, 0.5]],
        "num_repeat": 1,
        "seed": 42,
    }
    CG_CFG_D = {
        "profiles_path": "assets/Terrains/crater_spline_profiles.pkl",
        "min_xy_ratio": 0.85,
        "max_xy_ratio": 1.0,
        "random_rotation": True,
        "num_unique_profiles": 10000,
        "seed": 42,
    }
    CS_CFG_D = {
        "block_size": 50,
        "crater_gen_cfg": CG_CFG_D,
        "crater_dist_cfg": CDDC_CFG_D,
    }

    CS_CFG = CraterSamplerCfg(**CS_CFG_D)
    CS = CraterSampler(crater_sampler_cfg=CS_CFG, db=CDB)
    CB_CFG_D = {
        "resolution": 0.05,
        "block_size": 50,
        "z_scale": 10,
    }
    CB_CFG = CraterBuilderCfg(**CB_CFG_D)
    CB = CraterBuilder(settings=CB_CFG, db=CDB)

    out = CS.sample_craters_by_block((0, 0))
    out = CS.sample_craters_by_block((50, 0))
    out = CS.sample_craters_by_block((50, 50))
    out = CS.sample_craters_by_block((100, 0))
    out = CS.sample_craters_by_block((100, 100))
    out = CS.sample_craters_by_block((150, 50))
    out = CS.sample_craters_by_block((100, 150))
    CS.display_block_set(
        [
            (0, 0),
            (50, 0),
            (50, 50),
            (100, 0),
            (100, 100),
            (150, 50),
            (100, 150),
        ]
    )
    out = CS.sample_craters_by_region(BoundingBox(0, 200, 0, 200))
    CS.display_region(BoundingBox(0, 200, 0, 200))

    out = CDB.get_block_data((50, 50))
    crater = CB.generateCrater(out[0])
    plt.figure()
    plt.imshow(crater, cmap="terrain")
    craters = CB.generateCraters(out, (50, 50))
    plt.figure()
    plt.imshow(craters[0], cmap="terrain")

    plt.show()