# We have the low-res DEM. We need to generate the high-res DEM from it.
# We will use bi-cubic interpolation to generate the high-res DEM.
# We will use Perlin noise to generate the high-res DEM.

# --> So we would fetch the low-res DEM block from the DEM manager.
# --> We would generate the high-res DEM block from the low-res DEM block using bi-cubic interpolation.
# --> We would generate the high-res DEM block from the low-res DEM block using Perlin noise.


# I will store objects by chunks. This way I don't need to build a QuadTree for quick lookups. --> Or I can build a QuadTree of the chunks.
# If I use a KdTree, and I initialize it with empty chunks, I don't need to dynamically update the tree. I can just query the tree and update the chunks.

# I will generate objects in the main thread. And offload the application of these objects to worker threads.

# Using a given context window, we would collect all the chunks that are within the context window and their neighbors.
# --> We then check if they have been initialized or not.

from scipy.interpolate import CubicSpline
from matplotlib import pyplot as plt
from scipy.ndimage import rotate
from typing import Tuple, List
import numpy as np
import dataclasses
import colorsys
import pickle
import sys
import cv2


@dataclasses.dataclass
class CraterGeneratorCfg:
    profiles_path: str = dataclasses.field(default_factory=str)
    min_xy_ratio: float = dataclasses.field(default_factory=float)
    max_xy_ratio: float = dataclasses.field(default_factory=float)
    random_rotation: bool = dataclasses.field(default_factory=bool)
    seed: int = dataclasses.field(default_factory=int)
    num_unique_profiles: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        print(self.__dict__)
        assert type(self.profiles_path) is str, "profile_path must be a string"
        assert type(self.min_xy_ratio) is float, "min_xy_ratio must be a float"
        assert type(self.max_xy_ratio) is float, "max_xy_ratio must be a float"
        assert type(self.random_rotation) is bool, "random_rotation must be a boolean"
        assert type(self.seed) is int, "seed must be an integer"

        assert (
            self.min_xy_ratio <= self.max_xy_ratio
        ), "min_xy_ratio must be smaller than max_xy_ratio"
        assert self.min_xy_ratio > 0, "min_xy_ratio must be greater than 0"
        assert self.max_xy_ratio > 0, "max_xy_ratio must be greater than 0"
        assert self.min_xy_ratio <= 1, "min_xy_ratio must be smaller than 1"
        assert self.max_xy_ratio <= 1, "max_xy_ratio must be smaller than 1"


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


@dataclasses.dataclass
class CraterDBCfg:
    block_size: int = 50
    max_blocks: int = int(1e7)
    save_to_disk: bool = False
    write_to_disk_interval: int = 1000


@dataclasses.dataclass
class BoundingBox:
    x_min: float = 0
    x_max: float = 0
    y_min: float = 0
    y_max: float = 0


class CraterDB:
    def __init__(self, cfg: CraterDBCfg):
        self.crater_db = {}
        self.profile_db = {}
        self.crater_db_configs = cfg

    def add_deformation_profiles(self, profiles: List[CubicSpline]) -> None:
        self.profile_db["deformations"] = {}
        for i, profile in enumerate(profiles):
            self.profile_db["deformations"][i] = profile

    def add_marks_profiles(self, profiles: List[CubicSpline]) -> None:
        self.profile_db["markings"] = {}
        for i, profile in enumerate(profiles):
            self.profile_db["markings"][i] = profile

    def add_crater_profiles(self, profiles: List[CubicSpline]) -> None:
        self.profile_db["craters"] = {}
        for i, profile in enumerate(profiles):
            self.profile_db["craters"][i] = profile

    def get_deformation_spline(self, id: int) -> CubicSpline:
        return self.profile_db["deformations"][id]

    def get_marks_spline(self, id: int) -> CubicSpline:
        return self.profile_db["markings"][id]

    def get_crater_profile_spline(self, id: int) -> CubicSpline:
        return self.profile_db["craters"][id]

    def add_block_data(
        self, block_data: List[CraterMetadata], block_coordinates: Tuple[float, float]
    ) -> None:

        assert (
            block_coordinates[0] % self.crater_db_configs.block_size == 0
        ), "Block x-coordinate must be a multiple of the block size."
        assert (
            block_coordinates[1] % self.crater_db_configs.block_size == 0
        ), "Block y-coordinate must be a multiple of the block size."

        self.crater_db[block_coordinates] = block_data

    def is_valid(self, block_coordinates) -> bool:
        assert (
            block_coordinates[0] % self.crater_db_configs.block_size == 0
        ), "Block x-coordinate must be a multiple of the block size."
        assert (
            block_coordinates[1] % self.crater_db_configs.block_size == 0
        ), "Block y-coordinate must be a multiple of the block size."
        return True

    def get_block_data(
        self, block_coordinates: Tuple[float, float]
    ) -> List[CraterMetadata]:
        return self.crater_db[block_coordinates]

    def get_block_data_with_neighbors(
        self, block_coordinates: Tuple[float, float]
    ) -> List[CraterMetadata]:
        blocks = []
        for x in range(-1, 2, 1):
            xc = block_coordinates[0] + x * self.crater_db_configs.block_size
            for y in range(-1, 2, 1):
                yc = block_coordinates[1] + y * self.crater_db_configs.block_size
                if self.check_block_exists((xc, yc)):
                    blocks += self.get_block_data((xc, yc))
        return blocks

    def check_block_exists(self, block_coordinates: Tuple[float, float]) -> bool:
        return block_coordinates in self.crater_db

    def get_missing_blocks(self, region: BoundingBox) -> List[Tuple[float, float]]:
        missing_blocks = []
        x_min_rem = region.x_min % self.crater_db_configs.block_size
        y_min_rem = region.y_min % self.crater_db_configs.block_size
        x_max_rem = region.x_max % self.crater_db_configs.block_size
        y_max_rem = region.y_max % self.crater_db_configs.block_size

        x_min = region.x_min - x_min_rem
        y_min = region.y_min - y_min_rem
        x_max = region.x_max - x_max_rem
        y_max = region.y_max - y_max_rem

        for x in range(x_min, x_max, self.crater_db_configs.block_size):
            for y in range(y_min, y_max, self.crater_db_configs.block_size):
                if not self.check_block_exists((x, y)):
                    missing_blocks.append((x, y))
        return missing_blocks

    def get_blocks_within_region(self, region: BoundingBox) -> List[CraterMetadata]:
        blocks = []

        x_min_rem = region.x_min % self.crater_db_configs.block_size
        y_min_rem = region.y_min % self.crater_db_configs.block_size
        x_max_rem = region.x_max % self.crater_db_configs.block_size
        y_max_rem = region.y_max % self.crater_db_configs.block_size

        x_min = region.x_min - x_min_rem
        y_min = region.y_min - y_min_rem
        x_max = region.x_max - x_max_rem
        y_max = region.y_max - y_max_rem

        new_region = BoundingBox(x_min, x_max, y_min, y_max)

        occupied_block_matrix = np.zeros(
            (
                (int((x_max - x_min) / self.crater_db_configs.block_size)),
                (int((y_max - y_min) / self.crater_db_configs.block_size)),
            ),
            dtype=int,
        )

        for x in range(x_min, x_max, self.crater_db_configs.block_size):
            for y in range(y_min, y_max, self.crater_db_configs.block_size):
                if self.check_block_exists((x, y)):
                    blocks += self.get_block_data((x, y))

                    occupied_block_matrix[
                        int((x - x_min) / self.crater_db_configs.block_size),
                        int((y - y_min) / self.crater_db_configs.block_size),
                    ] = 1
        return (
            blocks,
            new_region,
            occupied_block_matrix,
        )

    def get_blocks_within_region_with_neighbors(
        self, region: BoundingBox
    ) -> List[CraterMetadata]:
        blocks = []

        x_min_rem = region.x_min % self.crater_db_configs.block_size
        y_min_rem = region.y_min % self.crater_db_configs.block_size
        x_max_rem = region.x_max % self.crater_db_configs.block_size
        y_max_rem = region.y_max % self.crater_db_configs.block_size

        x_min = region.x_min - x_min_rem - self.crater_db_configs.block_size
        y_min = region.y_min - y_min_rem - self.crater_db_configs.block_size
        x_max = region.x_max - x_max_rem + self.crater_db_configs.block_size
        y_max = region.y_max - y_max_rem + self.crater_db_configs.block_size

        new_region = BoundingBox(x_min, x_max, y_min, y_max)

        occupied_block_matrix = np.zeros(
            (
                (int((x_max - x_min) / self.crater_db_configs.block_size)),
                (int((y_max - y_min) / self.crater_db_configs.block_size)),
            ),
            dtype=int,
        )

        for x in range(x_min, x_max, self.crater_db_configs.block_size):
            for y in range(y_min, y_max, self.crater_db_configs.block_size):
                if self.check_block_exists((x, y)):
                    blocks += self.get_block_data((x, y))
                    occupied_block_matrix[
                        int((x - x_min) / self.crater_db_configs.block_size),
                        int((y - y_min) / self.crater_db_configs.block_size),
                    ] = 1

        return (
            blocks,
            new_region,
            occupied_block_matrix[1:-1, 1:-1],
        )

    def get_all_blocks(self) -> List[CraterMetadata]:
        blocks = []
        for block in self.crater_db.values():
            blocks += block
        return blocks

    def get_memory_footprint(self) -> int:
        return sys.getsizeof(self.crater_db), sys.getsizeof(self.profile_db)


@dataclasses.dataclass
class CraterDynamicDistributionCfg:
    densities: List[float] = dataclasses.field(default_factory=list)
    radius: List[float] = dataclasses.field(default_factory=list)
    num_repeat: int = 1
    seed: int = 42

    def __post_init__(self):
        assert len(self.densities) == len(self.radius), "The ..."
        self.num_repeat = int(self.num_repeat)
        assert self.num_repeat > 0, "Num ..."


class DynamicDistribute:
    """
    Distributes craters on a DEM using a Poisson process with hardcore rejection."""

    def __init__(
        self,
        settings: CraterDynamicDistributionCfg,
    ) -> None:
        """
        Args:
        """

        self.settings = settings
        self._rng = np.random.default_rng(self.settings.seed)

    def sampleFromPoisson(
        self,
        region: BoundingBox,
        l: float,
        r_minmax: Tuple[float, float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Samples from a Poisson process.

        Args:
            l (float): density of the Poisson process (in units per square meters).
            r_minmax (tuple): minimum and maximum radius of the craters (in meters).

        Returns:
            tuple: coordinates and radius of the craters.
        """

        area = (region.x_max - region.x_min) * (region.y_max - region.y_min)
        num_points = self._rng.poisson(area * l)
        radius = self._rng.uniform(r_minmax[0], r_minmax[1], num_points)
        x_coords = self._rng.uniform(region.x_min, region.x_max, num_points)
        y_coords = self._rng.uniform(region.y_min, region.y_max, num_points)
        return np.stack([x_coords, y_coords]).T, radius

    def hardcoreRejection(
        self, coords: np.ndarray, radius: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Performs hardcore rejection on the craters.

        Args:
            coords (np.ndarray): coordinates of the craters (in meters).
            radius (np.ndarray): radii of the craters (in meters).

        Returns:
            tuple: coordinates of the craters (in meters).
        """
        print(coords.shape)
        mark_age = self._rng.uniform(0, 1, coords.shape[0])
        boole_keep = np.zeros(mark_age.shape[0], dtype=bool)
        for i in range(mark_age.shape[0]):
            dist_tmp = np.linalg.norm(coords[i] - coords, axis=-1)
            in_disk = (dist_tmp < radius[i]) & (dist_tmp > 0)
            if len(mark_age[in_disk]) == 0:
                boole_keep[i] = True
            else:
                boole_keep[i] = all(mark_age[i] < mark_age[in_disk])
        return coords[boole_keep], radius[boole_keep]

    def checkPrevious(
        self, new_coords: np.ndarray, radius: np.ndarray, prev_coords: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Checks if the new craters are not in the previous craters.

        Args:
            new_coords (np.ndarray): coordinates of the new craters.
            radius (np.ndarray): radii of the new craters.
            prev_coords (np.ndarray): coordinates of the previous craters.

        Returns:
            tuple: the coordinates of the new craters, the radii of the new craters.
        """

        boole_keep = np.ones(new_coords.shape[0], dtype=bool)
        if prev_coords is None:
            boole_keep = np.ones(new_coords.shape[0], dtype=bool)
        else:
            for i in range(prev_coords[0].shape[0]):
                dist_tmp = np.linalg.norm(prev_coords[0][i] - new_coords, axis=-1)
                in_disk = (dist_tmp < prev_coords[1][i]) & (dist_tmp > 0)
                boole_keep[in_disk] = False
        return new_coords[boole_keep], radius[boole_keep]

    def simulateHCPoissonProcess(
        self,
        region: BoundingBox,
        l: float,
        r_minmax: Tuple[float],
        prev_coords: np.ndarray = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simulates a hardcore Poisson process.

        Args:
            l (float): density of the Poisson process (in units per square meters).
            r_minmax (tuple): minimum and maximum radius of the craters (in meters).
            prev_coords (np.ndarray): coordinates of the previous craters (in meters).

        Returns:
            tuple: coordinates of the craters, radii of the craters.
        """

        coords, radius = self.sampleFromPoisson(region, l, r_minmax)
        for _ in range(self.settings.num_repeat):
            coords, radius = self.hardcoreRejection(coords, radius)
            new_coords, new_radius = self.sampleFromPoisson(region, l, r_minmax)
            coords = np.concatenate([coords, new_coords])
            radius = np.concatenate([radius, new_radius])
            self.checkPrevious(coords, radius, prev_coords)
        coords, radius = self.hardcoreRejection(coords, radius)
        coords, radius = self.checkPrevious(coords, radius, prev_coords)
        return coords, radius

    def simulatePoissonProcess(
        self,
        region: BoundingBox,
        l: float,
        r_minmax: Tuple[float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simulates a Poisson process.

        Args:
            l (float): density of the Poisson process (in units per square meters).
            r_minmax (tuple): minimum and maximum radius of the craters (in meters).
            prev_coords (np.ndarray): coordinates of the previous craters (in meters).

        Returns:
            tuple: coordinates of the craters, radii of the craters.
        """

        coords, radius = self.sampleFromPoisson(region, l, r_minmax)
        return coords, radius

    def run_HC(
        self, region: BoundingBox, prev_coords: Tuple[np.ndarray, np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Runs the hardcore Poisson process for all the densities and radius in order.

        Returns:
            tuple: coordinates of the craters, radii of the craters."""

        if prev_coords[1].shape[0] == 0:
            prev_coords = None

        coords_to_save = []
        rads_to_save = []
        for d, r_minmax in zip(self.settings.densities, self.settings.radius):
            new_coords, new_radius = self.simulateHCPoissonProcess(
                region, d, r_minmax, prev_coords
            )
            coords_to_save.append(new_coords)
            rads_to_save.append(new_radius)
            if prev_coords is not None:
                prev_coords = (
                    np.concatenate([prev_coords[0], new_coords], axis=0),
                    np.concatenate([prev_coords[1], new_radius], axis=0),
                )
            else:
                prev_coords = (new_coords, new_radius)

        to_save = (
            np.concatenate(coords_to_save, axis=0),
            np.concatenate(rads_to_save, axis=0),
        )
        return to_save

    def run_NHC(
        self, region: BoundingBox, prev_coords: Tuple[np.ndarray, np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Runs the hardcore Poisson process for all the densities and radius in order.

        Returns:
            tuple: coordinates of the craters, radii of the craters."""

        if prev_coords[1].shape[0] == 0:
            prev_coords = None

        coords_to_save = []
        rads_to_save = []
        for d, r_minmax in zip(self.settings.densities, self.settings.radius):
            new_coords, new_radius = self.simulatePoissonProcess(region, d, r_minmax)
            coords_to_save.append(new_coords)
            rads_to_save.append(new_radius)
            if prev_coords is not None:
                prev_coords = (
                    np.concatenate([prev_coords[0], new_coords], axis=0),
                    np.concatenate([prev_coords[1], new_radius], axis=0),
                )
            else:
                prev_coords = (new_coords, new_radius)

        to_save = (
            np.concatenate(coords_to_save, axis=0),
            np.concatenate(rads_to_save, axis=0),
        )
        return to_save

    def run(
        self,
        region: BoundingBox,
        prev_coords: Tuple[np.ndarray, np.ndarray] = None,
        use_hc: bool = False,
    ) -> Tuple[np.ndarray, np.ndarray]:

        if use_hc:
            return self.run_HC(region, prev_coords)
        else:
            return self.run_NHC(region, prev_coords)


class CraterMetadataGenerator:
    def __init__(self, settings: CraterGeneratorCfg):
        self.settings = settings
        self.deformation_profiles = []
        self.marking_profiles = []
        self._rng = np.random.default_rng(self.settings.seed)
        self.build()

    def build(self):
        print("Warming up crater generation...")
        self.generate_deformation_profiles()
        self.generate_marking_profiles()
        self.loadProfiles()

    def get_crater_profiles(self) -> List[CubicSpline]:
        return self.crater_profiles

    def get_deformation_profiles(self) -> List[CubicSpline]:
        return self.deformation_profiles

    def get_marking_profiles(self) -> List[CubicSpline]:
        return self.marking_profiles

    def generate_deformation_profiles(self):
        print("Pre-generating crater deformation profiles")
        for i in range(self.settings.num_unique_profiles):
            deformation_profile = self._rng.uniform(0.95, 1, 9)
            deformation_profile = np.concatenate(
                [deformation_profile, [deformation_profile[0]]], axis=0
            )
            tmp_x = np.linspace(0, 1, deformation_profile.shape[0])
            self.deformation_profiles.append(
                CubicSpline(tmp_x, deformation_profile, bc_type=((1, 0.0), (1, 0.0)))
            )

    def generate_marking_profiles(self):
        print("Pre-generating crater marking profiles")
        for i in range(self.settings.num_unique_profiles):
            # Generates a profile to add marks that converges toward the center of the crater
            marks_profile = self._rng.uniform(0.0, 0.01, 45)
            marks_profile = np.concatenate([marks_profile, [marks_profile[0]]], axis=0)
            tmp_x = np.linspace(0, 1, marks_profile.shape[0])
            self.marking_profiles.append(
                CubicSpline(tmp_x, marks_profile, bc_type=((1, 0.0), (1, 0.0)))
            )

    def loadProfiles(self) -> None:
        """
        Loads the half crater spline profiles from a pickle file.
        """

        print("Loading crater profiles")
        with open(self.settings.profiles_path, "rb") as handle:
            self.crater_profiles = pickle.load(handle)

    def randomizeCraterParameters(
        self, coordinates: Tuple[float, float], radius: float
    ) -> CraterMetadata:
        """
        Randomizes the parameters of a crater.

        Args:

        Returns:
            CraterData: data regarding the crater generation."""

        # Generates the metadata for a random crater
        crater_data = CraterMetadata()

        # Makes sure the matrix size is odd
        crater_data.radius = radius
        crater_data.coordinates = coordinates
        crater_data.deformation_spline_id = self._rng.integers(
            0, self.settings.num_unique_profiles, 1
        )[0]
        crater_data.marks_spline_id = self._rng.integers(
            0, self.settings.num_unique_profiles, 1
        )[0]
        crater_data.marks_intensity = self._rng.uniform(0, 1)
        crater_data.crater_profile_id = self._rng.integers(
            0, len(self.crater_profiles), 1
        )[0]
        crater_data.xy_deformation_factor = (
            self._rng.uniform(self.settings.min_xy_ratio, self.settings.max_xy_ratio),
            1.0,
        )
        crater_data.rotation = int(self._rng.uniform(0, 360))
        return crater_data

    def run(self, coordinates: np.ndarray, radius: np.ndarray) -> List[CraterMetadata]:
        metadatas = []
        for i in range(coordinates.shape[0]):
            metadatas.append(self.randomizeCraterParameters(coordinates[i], radius[i]))
        return metadatas

    def castMetadata(
        self, metadatas=List[CraterMetadata]
    ) -> Tuple[np.ndarray, np.ndarray]:
        coordinates = np.array([metadata.coordinates for metadata in metadatas])
        radius = np.array([metadata.radius for metadata in metadatas])

        return coordinates, radius


@dataclasses.dataclass
class CraterSamplerCfg:
    block_size: int = 50
    crater_gen_cfg: CraterGeneratorCfg = dataclasses.field(default_factory=dict)
    crater_dist_cfg: CraterDynamicDistributionCfg = dataclasses.field(
        default_factory=dict
    )

    def __post_init__(self):
        assert (
            self.crater_gen_cfg is not None
        ), "Crater generator configuration must be provided."
        assert (
            self.crater_dist_cfg is not None
        ), "Crater distribution configuration must be provided."

        self.crater_gen_cfg = CraterGeneratorCfg(**self.crater_gen_cfg)
        self.crater_dist_cfg = CraterDynamicDistributionCfg(**self.crater_dist_cfg)


class CraterSampler:
    def __init__(self, crater_sampler_cfg: CraterSamplerCfg, db: CraterDB):
        self.settings = crater_sampler_cfg
        self.crater_dist_gen = DynamicDistribute(self.settings.crater_dist_cfg)
        self.crater_metadata_gen = CraterMetadataGenerator(self.settings.crater_gen_cfg)
        self.crater_db = db
        self.add_profiles_to_db()

    def add_profiles_to_db(self):
        self.crater_db.add_deformation_profiles(
            self.crater_metadata_gen.get_deformation_profiles()
        )
        self.crater_db.add_marks_profiles(
            self.crater_metadata_gen.get_marking_profiles()
        )
        self.crater_db.add_crater_profiles(
            self.crater_metadata_gen.get_crater_profiles()
        )

    @staticmethod
    def compute_largest_rectangle(matrix: np.ndarray) -> Tuple[int, int]:
        """
        Computes the largest rectangle in a binary matrix.

        Args:
            matrix (np.ndarray): binary matrix.

        Returns:
            tuple: the coordinates of the largest rectangle.
        """
        m, n = matrix.shape
        left = np.zeros((n), dtype=int)
        right = np.ones((n), dtype=int) * n
        height = np.zeros((n), dtype=int)

        max_area = 0
        s_i = -1
        s_height = -1
        s_left = -1
        s_right = -1

        for i in range(m):
            cur_left, cur_right = 0, n

            for j in range(n - 1, -1, -1):
                if matrix[i, j] == 0:
                    right[j] = min(right[j], cur_right)
                else:
                    right[j] = n
                    cur_right = j

            for j in range(n):
                if matrix[i, j] == 0:
                    height[j] += 1
                    left[j] = max(left[j], cur_left)
                else:
                    height[j] = 0
                    left[j] = 0
                    cur_left = j + 1

                v = max(max_area, (right[j] - left[j]) * height[j])
                if v > max_area:
                    max_area = v
                    s_i = i
                    s_left = left[j]
                    s_right = right[j]
                    s_height = height[j]

        coords = ((s_i - s_height + 1, s_i + 1), (s_left, s_right))
        return max_area, coords

    def sample_craters_by_block(self, block_coordinates: Tuple[int, int]) -> None:
        self.crater_db.is_valid(block_coordinates)
        if self.crater_db.check_block_exists(block_coordinates):
            block = self.crater_db.get_block_data(block_coordinates)
        else:
            block = self.crater_db.get_block_data_with_neighbors(block_coordinates)
            prev_coords = self.crater_metadata_gen.castMetadata(block)
            bb = BoundingBox(
                block_coordinates[0],
                block_coordinates[0] + self.settings.block_size,
                block_coordinates[1],
                block_coordinates[1] + self.settings.block_size,
            )
            coord, rad = self.crater_dist_gen.run(bb, prev_coords=prev_coords)
            block = self.crater_metadata_gen.run(coord, rad)
            self.crater_db.add_block_data(block, block_coordinates)

    def dissect_region_blocks(
        self, blocks: Tuple[np.ndarray, np.ndarray], region: BoundingBox
    ) -> Tuple[List[Tuple[np.ndarray, np.ndarray]], List[Tuple[float, float]]]:
        block_list = []
        coordinate_list = []
        for x in range(region.x_min, region.x_max, self.settings.block_size):
            for y in range(region.y_min, region.y_max, self.settings.block_size):
                idx = np.where(
                    (blocks[0][:, 0] >= x)
                    & (blocks[0][:, 0] < x + self.settings.block_size)
                    & (blocks[0][:, 1] >= y)
                    & (blocks[0][:, 1] < y + self.settings.block_size)
                )
                if len(idx[0]) > 0:
                    block_list.append((blocks[0][idx], blocks[1][idx]))
                    coordinate_list.append((x, y))

        return block_list, coordinate_list

    def sample_craters_by_region(self, region: BoundingBox) -> None:
        process_by_regions = True
        print("Sampling craters by region")
        print("original region", region)
        while process_by_regions:
            _, _, matrix = self.crater_db.get_blocks_within_region_with_neighbors(
                region
            )
            area, coords = self.compute_largest_rectangle(matrix)
            # print(area)
            # print(coords)
            if area <= 1:
                process_by_regions = False
                break
            new_region = BoundingBox(
                region.x_min + coords[0][0] * self.settings.block_size,
                region.x_min + coords[0][1] * self.settings.block_size,
                region.y_min + coords[1][0] * self.settings.block_size,
                region.y_min + coords[1][1] * self.settings.block_size,
            )

            print("selected region", new_region)
            # print("get blocks within region")
            blocks, _, _ = self.crater_db.get_blocks_within_region_with_neighbors(
                new_region
            )
            # print("found", len(blocks), "blocks")
            # print("casting from metadata to arrays")
            prev_coords = self.crater_metadata_gen.castMetadata(blocks)
            # print("running crater distribution")
            new_blocks = self.crater_dist_gen.run(new_region, prev_coords=prev_coords)
            print("new blocks", new_blocks[0].shape)
            # print("dissecting region blocks")
            new_blocks_list, block_coordinates_list = self.dissect_region_blocks(
                new_blocks, region
            )
            print("coordinates", block_coordinates_list)
            for (coordinates, radius), block_coordinates in zip(
                new_blocks_list, block_coordinates_list
            ):
                metadata = self.crater_metadata_gen.run(coordinates, radius)
                self.crater_db.add_block_data(metadata, block_coordinates)
        coordinates = self.crater_db.get_missing_blocks(region)
        for coord in coordinates:
            self.sample_craters_by_block(coord)

    def display_block(self, coordinates: Tuple[float, float]):
        fig = plt.figure(figsize=(10, 10), dpi=300)
        if self.crater_db.check_block_exists(coordinates):
            block = self.crater_db.get_block_data(coordinates)
            coordinates, radius = self.crater_metadata_gen.castMetadata(block)
            ppm = 3000 / self.settings.block_size / 5
            print(radius)
            print(np.max(radius))
            print(np.min(radius))
            plt.scatter(coordinates[:, 0], coordinates[:, 1], s=radius * ppm)
            plt.axis("equal")
        # plt.show()

    def display_block_set(self, coords: List[Tuple[float, float]]):
        fig = plt.figure(figsize=(10, 10), dpi=300)
        blocks = [self.crater_db.get_block_data(coord) for coord in coords]

        color_interp = np.linspace(0, 1, len(blocks), endpoint=False)
        colors = [colorsys.hsv_to_rgb(i, 1, 1) for i in color_interp]
        for i, block in enumerate(blocks):
            coordinates, radius = self.crater_metadata_gen.castMetadata(block)
            ppm = 3000 / self.settings.block_size / 5
            plt.scatter(
                coordinates[:, 0], coordinates[:, 1], s=radius * ppm, c=[colors[i]]
            )

        plt.axis("equal")
        # plt.show()

    def display_region(self, region: BoundingBox):
        fig = plt.figure(figsize=(10, 10), dpi=300)
        blocks, _, _ = self.crater_db.get_blocks_within_region(region)
        coordinates, radius = self.crater_metadata_gen.castMetadata(blocks)
        blocks, block_coordinates = self.dissect_region_blocks(
            (coordinates, radius), region
        )
        color_interp = np.linspace(0, 1, len(blocks), endpoint=False)
        colors = [colorsys.hsv_to_rgb(i, 1, 1) for i in color_interp]
        for i in range(len(blocks)):
            coordinates, radius = blocks[i]
            ppm = 3000 / self.settings.block_size / 5
            plt.scatter(
                coordinates[:, 0], coordinates[:, 1], s=radius * ppm, c=[colors[i]]
            )
        plt.axis("equal")
        # plt.show()


@dataclasses.dataclass
class CraterBuilderCfg:
    block_size: int = 50
    pad_size: int = 10
    resolution: float = dataclasses.field(default_factory=float)
    z_scale: float = dataclasses.field(default_factory=float)


class CraterBuilder:
    def __init__(self, settings: CraterBuilderCfg, db: CraterDB):
        self.settings = settings
        self.db = db

    @staticmethod
    def sat_gaussian(x: np.ndarray, mu1: float, mu2: float, std: float) -> np.ndarray:
        """
        Saturates a gaussian function to its maximum between mu1 and mu2 with a standard deviation of std.

        Args:
            x (np.ndarray): input array.
            mu1 (float): gaussian mu lower bound.
            mu2 (float): gaussian mu upper bound.
            std (float): standard deviation.

        Returns:
            np.ndarray: saturated gaussian."""

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
            n (int): size of the matrix

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
            index (int): index of the profile to apply.
            distance (np.ndarray): distance matrix.
            size (int): size of the matrix.

        Returns:
            np.ndarray: crater DEM."""

        profile_spline = self.db.get_crater_profile_spline(
            crater_metadata.crater_profile_id
        )
        crater = profile_spline(2 * distance / size)
        return crater

    def generateCrater(self, crater_metadata: CraterMetadata) -> np.ndarray:
        """
        Generates a crater DEM.

        Args:
            size (float): size of the crater.
            index (int): index of the profile to use.

        Returns:
            tuple: crater DEM, the whole the of the data regarding the crater generation.
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
            DEM (np.ndarray): DEM to add craters to.
            coords (np.ndarray): coordinates of the craters (in meters).

        Returns:
            np.ndarray: unpadded DEM with craters, and matching mask."""

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
