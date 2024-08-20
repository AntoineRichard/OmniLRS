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
from typing import List, Tuple
import dataclasses
import numpy as np
import colorsys
import pickle
import math

from src.terrain_management.large_scale_terrain.utils import BoundingBox, RockBlockData
from src.terrain_management.large_scale_terrain.rock_database import RockDB

# Poisson, thomas point process


def mock_call(x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    # quat convention is x,y,z,w
    z = np.zeros_like(x)
    quat = np.zeros((x.shape[0], 4))
    quat[:, -1] = 1
    return z, quat


@dataclasses.dataclass
class BaseDistribution:
    name: str = dataclasses.field(default_factory=str)

    def sample(self, **kwargs):
        raise NotImplementedError


@dataclasses.dataclass
class Poisson(BaseDistribution):
    name: str = "poisson"
    density: float = 0.0
    seed: int = 42

    def __post_init__(self):
        self.density = float(self.density)
        self.seed = int(self.seed)
        self.rng = np.random.default_rng(self.seed)

    def get_num_points(self, region: BoundingBox):
        area = (region.x_max - region.x_min) * (region.y_max - region.y_min)
        return self.rng.poisson(area * self.density)

    def sample(self, region: BoundingBox, **kwargs):
        num_points = self.get_num_points(region)
        x_coords = self.rng.uniform(region.x_min, region.x_max, num_points)
        y_coords = self.rng.uniform(region.y_min, region.y_max, num_points)
        return np.stack([x_coords, y_coords]).T, num_points


@dataclasses.dataclass
class ThomasPointProcess(BaseDistribution):
    name: str = "thomas_point_process"
    parent_density: float = 0.0
    child_density: float = 0.0
    sigma: float = 0.0
    seed: int = 42

    def __post_init__(self):
        self.density = float(self.density)
        self.seed = int(self.seed)
        self.parent = Poisson(self.parent_density, self.seed)
        self.rng = np.random.default_rng(self.seed)
        self.normal = Normal(0, self.sigma, self.seed)
        self.extension = 7 * self.sigma

    def sample_parents(self, region: BoundingBox):
        region_ext = BoundingBox(region.x_min, region.x_max, region.y_min, region.y_max)
        region_ext.x_min = region.x_min - self.extension
        region_ext.x_max = region.x_max + self.extension
        region_ext.y_min = region.y_min - self.extension
        region_ext.y_max = region.y_max + self.extension

        parent_coords, num_parents = self.parent.sample(region_ext)
        return parent_coords, num_parents

    def sample_children(self, parent_coords, num_parents):
        num_child = self.rng.poisson(self.child_density, num_parents)
        num_points = np.sum(num_child)
        children_coords = self.normal.sample(num_points, 2)
        children_coords = children_coords + np.repeat(parent_coords, num_child, axis=0)
        return children_coords, num_points

    def sample(self, region: BoundingBox, **kwargs):
        parent_coords, num_parents = self.sample_parents(region)
        children_coords, num_children = self.sample_children(parent_coords, num_parents)
        return children_coords, num_children


@dataclasses.dataclass
class Uniform(BaseDistribution):
    name: str = "uniform"
    min: float = 0.0
    max: float = 0.0
    seed: int = 42

    def __post_init__(self):
        assert self.min < self.max, "min must be smaller than max"
        self.min = float(self.min)
        self.max = float(self.max)
        self.seed = int(self.seed)
        self.rng = np.random.default_rng(self.seed)

    def sample(self, num_points, dim=1, **kwargs):
        return self.rng.uniform(self.min, self.max, (num_points, dim))


@dataclasses.dataclass
class Normal(BaseDistribution):
    name: str = "normal"
    mean: float = 0.0
    std: float = 0.0
    seed: int = 42

    def __post_init__(self):
        assert self.std > 0, "std must be larger than 0"
        self.mean = float(self.mean)
        self.std = float(self.std)
        self.seed = int(self.seed)
        self.rng = np.random.default_rng(self.seed)

    def sample(self, num_points, dim=1, **kwargs):
        return self.rng.normal(self.mean, self.std, (num_points, dim))


@dataclasses.dataclass
class Integer(BaseDistribution):
    name: str = "integer"
    min: int = 0
    max: int = 1
    seed: int = 42

    def __post_init__(self):
        assert self.min < self.max, "min must be smaller than max"
        self.min = int(self.min)
        self.max = int(self.max)
        self.seed = int(self.seed)
        self.rng = np.random.default_rng(self.seed)

    def sample(self, num_points, **kwargs):
        return self.rng.integers(self.min, self.max, num_points)


class Factory:
    """
    Factory class to create objects based on configs.
    """

    def __init__(self) -> None:
        """
        Args:
            cfg (dict): configuration for the distribution.
        """

        self.objs = {}

    def add(self, obj, name) -> None:
        """
        Creates the distribution object.
        """

        self.objs[name] = obj

    def create(self, cfg) -> object:
        """
        Creates the distribution object.
        """

        if cfg["name"] not in self.objs:
            raise ValueError(f"Unknown distribution: {cfg['name']}")

        return self.objs[cfg["name"]](**cfg)


distribution_factory = Factory()
distribution_factory.add(Poisson, "poisson")
distribution_factory.add(ThomasPointProcess, "thomas_point_process")
distribution_factory.add(Uniform, "uniform")
distribution_factory.add(Normal, "normal")
distribution_factory.add(Integer, "integer")


@dataclasses.dataclass
class RockDynamicDistributionCfg:
    """
    Args:
    """

    position_distribution: dict = dataclasses.field(default_factory=dict)
    scale_distribution: dict = dataclasses.field(default_factory=dict)
    seed: int = dataclasses.field(default_factory=int)
    num_rocks_id: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        self.position_distribution = distribution_factory.create(
            self.position_distribution
        )
        self.scale_distribution = distribution_factory.create(self.scale_distribution)

        assert self.num_rocks_id > 0, "num_rocks_id must be larger than 0"


class DynamicDistribute:
    """
    Distributes craters on a DEM using a Poisson process with hardcore rejection.
    """

    def __init__(
        self,
        settings: RockDynamicDistributionCfg,
        sampling_func: callable = mock_call,
    ) -> None:
        """
        Args:
            settings (CraterDynamicDistributionCfg): settings for the crater distribution.
        """

        self.settings = settings
        self._rng = np.random.default_rng(self.settings.seed)
        self.sampling_func = sampling_func

    def build_samplers(self):
        self.position_sampler = self.settings.position_distribution
        self.scale_sampler = self.settings.scale_distribution
        id_sampler_cfg = {
            "name": "integer",
            "min": 0,
            "max": self.settings.num_rocks_id,
            "seed": self.settings.seed,
        }
        self.id_sampler = distribution_factory.create(id_sampler_cfg)

    def run(self, region: BoundingBox) -> RockBlockData:
        xy_position, num_points = self.position_sampler(region)
        scale = self.scale_sampler(num_points)
        ids = self.id_sampler(num_points)
        z_position, quat = self.sampling_func(
            xy_position[:, 0], xy_position[:, 1], self.settings.seed
        )
        xyz_position = np.stack([xy_position[:, 0], xy_position[:, 1], z_position]).T

        block = RockBlockData(xyz_position, quat, scale, ids)
        return block

    def get_xy_coordinates_from_block(self, block: RockBlockData) -> np.ndarray:
        return block.coordinates[:, :2]


@dataclasses.dataclass
class RockSamplerCfg:
    """
    Args:
        block_size (int): size of the blocks.
        crater_gen_cfg (CraterGeneratorCfg): configuration for the rock generator.
        crater_dist_cfg (CraterDynamicDistributionCfg): configuration for the rock distribution
    """

    block_size: int = 50
    rock_dist_cfg: RockDynamicDistributionCfg = dataclasses.field(default_factory=dict)

    def __post_init__(self) -> None:
        assert (
            self.crater_dist_cfg is not None
        ), "Crater distribution configuration must be provided."

        self.crater_dist_cfg = RockDynamicDistributionCfg(**self.rock_dist_cfg)


class RockSampler:
    """
    Class to scatter rocks on a DEM. It aggregates the crater distribution, the crater
    generation and the crater database to generate craters' metadata on the fly.
    """

    def __init__(
        self,
        crater_sampler_cfg: RockSamplerCfg,
        db: RockDB,
        map_sampling_func: callable = mock_call,
    ) -> None:
        """
        Args:
            crater_sampler_cfg (CraterSamplerCfg): configuration for the crater sampler.
            db (CraterDB): database to store the craters.
        """

        self.settings = crater_sampler_cfg
        self.rock_dist_gen = DynamicDistribute(
            self.settings.crater_dist_cfg, sampling_func=map_sampling_func
        )
        self.rock_db = db

    @staticmethod
    def compute_largest_rectangle(matrix: np.ndarray) -> Tuple[int, int]:
        """
        Computes the largest rectangle in a binary matrix.
        This is used to find the largest region without craters in the matrix.

        Args:
            matrix (np.ndarray): binary matrix.

        Returns:
            Tuple[float, Tuple[int,int]]: the area and coordinates of the largest rectangle.
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

    def sample_rocks_by_block(self, block_coordinates: Tuple[int, int]) -> None:
        """
        Samples rocks by block. This method is used to sample rocks in a block
        that does not contain any rocks.

        Args:
            block_coordinates (Tuple[int,int]): coordinates of the block.
        """

        # Checks if the block is valid
        self.rock_db.is_valid(block_coordinates)

        # Checks if the block already contains rocks
        if not self.rock_db.check_block_exists(block_coordinates):
            bb = BoundingBox(
                block_coordinates[0],
                block_coordinates[0] + self.settings.block_size,
                block_coordinates[1],
                block_coordinates[1] + self.settings.block_size,
            )
            block = self.rock_dist_gen.run(bb)
            self.rock_db.add_block_data(block, block_coordinates)

    def dissect_region_blocks(
        self, block: RockBlockData, region: BoundingBox
    ) -> Tuple[List[RockBlockData], List[Tuple[float, float]]]:
        """
        Dissects the region into blocks and returns the rocks in each block.

        Args:
            blocks (Tuple[np.ndarray, np.ndarray]): coordinates and radii of the rocks.
            region (BoundingBox): region to dissect.

        Returns:
            Tuple[List[Tuple[np.ndarray, np.ndarray]], List[Tuple[float, float]]]: list of
                rocks in each block, coordinates of the blocks.
        """

        block_list = []
        coordinate_list = []

        xy = self.rock_dist_gen.get_xy_coordinates_from_block(block)

        for i, x in enumerate(
            range(region.x_min, region.x_max, self.settings.block_size)
        ):
            for j, y in enumerate(
                range(region.y_min, region.y_max, self.settings.block_size)
            ):
                # If rocks are generated outside the regin boundaries, we store them
                # in the boundary blocks. We do this to prevent the db from believing
                # data has been generated in the neighboring blocks. Ideally, we would
                # like to avoid this, but this would require extra work on the db side
                # to flag partially generated blocks.
                # The issue will be that we will need to be careful when querying the
                # DEM to avoid requesting data outside its boundaries.
                # TODO (antoine.richard / JAOPS): handle this case properly

                if i == 0:
                    c1 = np.ones_like(xy[:, 0], dtype=bool)
                else:
                    c1 = xy[:, 0] >= x
                if i == region.x_max // self.settings.block_size:
                    c2 = np.ones_like(xy[:, 0], dtype=bool)
                else:
                    c2 = xy[:, 0] < x + self.settings.block_size
                if j == 0:
                    c3 = np.ones_like(xy[:, 1], dtype=bool)
                else:
                    c3 = xy[:, 1] >= y
                if j == region.y_max // self.settings.block_size:
                    c4 = np.ones_like(xy[:, 1], dtype=bool)
                else:
                    c4 = xy[:, 1] < y + self.settings.block_size

                cond = c1 & c2 & c3 & c4

                idx = np.where(cond)
                if len(idx[0]) > 0:
                    block_list.append(
                        RockBlockData(
                            xy[idx],
                            block.quaternion[idx],
                            block.scale[idx],
                            block.ids[idx],
                        )
                    )
                    coordinate_list.append((x, y))

        return block_list, coordinate_list

    def sample_rocks_by_region(self, region: BoundingBox) -> None:
        """
        Samples craters by region. This method is used to sample craters in a region.
        It will sample craters only in the blocks that do not contain any craters.
        To optimize the process, it finds the largest rectangle in the region that does not
        contain any craters and samples craters in this region. This process is repeated
        until the largest rectangle is smaller than a certain threshold. After that,
        the method samples craters in the remaining blocks that do not contain any craters.

        Args:
            region (BoundingBox): region to sample craters from.
        """

        # Process by regions until the largest rectangle is smaller or equal to 1 block
        process_by_regions = True
        while process_by_regions:
            _, _, matrix = self.rock_db.get_blocks_within_region_with_neighbors(region)
            # Computes the largest rectangle in the region that does not contain any craters
            area, coords = self.compute_largest_rectangle(matrix)
            # If the area is smaller or equal to 1 block, we stop the process
            # and fallback to the block sampling method
            if area <= 1:
                process_by_regions = False
                break

            # Region to sample craters from
            new_region = BoundingBox(
                region.x_min + coords[0][0] * self.settings.block_size,
                region.x_min + coords[0][1] * self.settings.block_size,
                region.y_min + coords[1][0] * self.settings.block_size,
                region.y_min + coords[1][1] * self.settings.block_size,
            )

            # Samples craters in the region
            new_blocks = self.rock_dist_gen.run(new_region)

            # Dissects the region into blocks and adds the craters to the database
            new_blocks_list, block_coordinates_list = self.dissect_region_blocks(
                new_blocks, region
            )
            for block_data, block_coordinates in zip(
                new_blocks_list, block_coordinates_list
            ):
                self.rock_db.add_block_data(block_data, block_coordinates)

        # If the largest rectangle is smaller or equal to 1 block, we sample craters
        # on a per block basis.
        coordinates = self.rock_db.get_missing_blocks(region)
        for coord in coordinates:
            self.sample_rocks_by_block(coord)

    def display_block(self, coordinates: Tuple[float, float]):
        """
        Displays the craters in a block.

        Args:
            coordinates (Tuple[float,float]): coordinates of the block.
        """
        fig = plt.figure(figsize=(10, 10), dpi=300)
        if self.rock_db.check_block_exists(coordinates):
            block = self.rock_db.get_block_data(coordinates)
            coordinates = self.rock_dist_gen.get_xy_coordinates_from_block(block)
            plt.scatter(coordinates[:, 0], coordinates[:, 1])
            plt.axis("equal")

    def display_block_set(self, coords: List[Tuple[float, float]]):
        """
        Displays a set of blocks. Each block is represented by a different color.

        Args:
            coords (List[Tuple[float,float]]): list of coordinates of the blocks.
        """
        fig = plt.figure(figsize=(10, 10), dpi=300)
        blocks = [self.rock_db.get_block_data(coord) for coord in coords]

        color_interp = np.linspace(0, 1, len(blocks), endpoint=False)
        colors = [colorsys.hsv_to_rgb(i, 1, 1) for i in color_interp]
        for i, block in enumerate(blocks):
            coordinates = self.rock_dist_gen.get_xy_coordinates_from_block(block)
            plt.scatter(coordinates[:, 0], coordinates[:, 1], c=[colors[i]])
        plt.axis("equal")

    def display_region(self, region: BoundingBox):
        """
        Displays the craters in a region. Each block is represented by a different color.

        Args:
            region (BoundingBox): region to display.
        """

        fig = plt.figure(figsize=(10, 10), dpi=300)
        blocks, _, _ = self.rock_db.get_blocks_within_region(region)
        color_interp = np.linspace(0, 1, len(blocks), endpoint=False)
        colors = [colorsys.hsv_to_rgb(i, 1, 1) for i in color_interp]
        for i in range(len(blocks)):
            coordinates = self.rock_dist_gen.get_xy_coordinates_from_block(blocks[i])
            plt.scatter(coordinates[:, 0], coordinates[:, 1], c=[colors[i]])
        plt.axis("equal")
