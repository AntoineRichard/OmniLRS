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

from src.terrain_management.large_scale_terrain.utils import BoundingBox, RockBlockData
from src.terrain_management.large_scale_terrain.rock_database import RockDB

# Poisson, thomas point process

@dataclasses.dataclass
class Poisson:
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

    def sample(self, region: BoundingBox):
        num_points = self.get_num_points(region)
        x_coords = self.rng.uniform(region.x_min, region.x_max, num_points)
        y_coords = self.rng.uniform(region.y_min, region.y_max, num_points)
        return np.stack([x_coords, y_coords]).T, num_points


@dataclasses.dataclass
class ThomasPointProcess:
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
        self.extension = 7*self.sigma

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
    
    def sample(self, region: BoundingBox):
        parent_coords, num_parents = self.sample_parents(region)
        children_coords, num_children = self.sample_children(parent_coords, num_parents)
        return children_coords, num_children

@dataclasses.dataclass
class Uniform:
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

    def sample(self, num_points, dim=1):
        return self.rng.uniform(self.min, self.max, (num_points, dim))

@dataclasses.dataclass
class Normal:
    name: str = "normal"
    mean: float = 0.0
    std: float = 0.0
    seed: int = 42

    def __post_init__(self):
        assert self.std > 0, "std must be larger than 0"
        self.mean = float(self.mean)
        self.std = float(self.std)
        self.seed = int(self.seed)
    
    def sample(self, num_points, dim=1):
        rng = np.random.default_rng(self.seed)
        return rng.normal(self.mean, self.std, (num_points, dim))
    

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


@dataclasses.dataclass
class RockDynamicDistributionCfg:
    """
    Args:
    """

    position_distribution: dict = dataclasses.field(default_factory=dict)
    scale_distribution: dict = dataclasses.field(default_factory=dict)

    def __post_init__(self):
        self.position_distribution = distribution_factory.create(self.position_distribution)
        self.scale_distribution = distribution_factory.create(self.scale_distribution)


class DynamicDistribute:
    """
    Distributes craters on a DEM using a Poisson process with hardcore rejection.
    """

    def __init__(
        self,
        settings: RockDynamicDistributionCfg,
    ) -> None:
        """
        Args:
            settings (CraterDynamicDistributionCfg): settings for the crater distribution.
        """

        self.settings = settings
        self._rng = np.random.default_rng(self.settings.seed)

    def build_samplers(self):
        self.position_sampler = self.settings.position_distribution
        self.scale_sampler = self.settings.scale_distribution
        self.rotation_sampler = distribution_factory.create(random_rotation)


@dataclasses.dataclass
class CraterSamplerCfg:
    """
    Args:
        block_size (int): size of the blocks.
        crater_gen_cfg (CraterGeneratorCfg): configuration for the crater generator.
        crater_dist_cfg (CraterDynamicDistributionCfg): configuration for the crater distribution
    """

    block_size: int = 50
    crater_dist_cfg: CraterDynamicDistributionCfg = dataclasses.field(
        default_factory=dict
    )

    def __post_init__(self) -> None:
        assert (
            self.crater_gen_cfg is not None
        ), "Crater generator configuration must be provided."
        assert (
            self.crater_dist_cfg is not None
        ), "Crater distribution configuration must be provided."

        self.crater_dist_cfg = CraterDynamicDistributionCfg(**self.crater_dist_cfg)


class RockSampler:
    """
    Class to scatter rocks on a DEM. It aggregates the crater distribution, the crater
    generation and the crater database to generate craters' metadata on the fly.
    """

    def __init__(self, crater_sampler_cfg: CraterSamplerCfg, db: CraterDB) -> None:
        """
        Args:
            crater_sampler_cfg (CraterSamplerCfg): configuration for the crater sampler.
            db (CraterDB): database to store the craters.
        """

        self.settings = crater_sampler_cfg
        self.crater_dist_gen = DynamicDistribute(self.settings.crater_dist_cfg)
        self.crater_metadata_gen = CraterMetadataGenerator(self.settings.crater_gen_cfg)
        self.rock_db = db
        self.add_profiles_to_db()

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
        Samples craters by block. This method is used to sample craters in a block
        that does not contain any craters.

        Args:
            block_coordinates (Tuple[int,int]): coordinates of the block.
        """

        # Checks if the block is valid
        self.crater_db.is_valid(block_coordinates)

        # Checks if the block already contains craters
        if self.crater_db.check_block_exists(block_coordinates):
            block = self.crater_db.get_block_data(block_coordinates)
        else:
            # Converts the metadatas into numpy arrays
            bb = BoundingBox(
                block_coordinates[0],
                block_coordinates[0] + self.settings.block_size,
                block_coordinates[1],
                block_coordinates[1] + self.settings.block_size,
            )
            block = self.rock_dist_gen.run(bb)
            self.rock_db.add_block_data(block, block_coordinates)

    def dissect_region_blocks(
        self, blocks: Tuple[np.ndarray, np.ndarray], region: BoundingBox
    ) -> Tuple[List[Tuple[np.ndarray, np.ndarray]], List[Tuple[float, float]]]:
        """
        Dissects the region into blocks and returns the craters in each block.

        Args:
            blocks (Tuple[np.ndarray, np.ndarray]): coordinates and radii of the craters.
            region (BoundingBox): region to dissect.

        Returns:
            Tuple[List[Tuple[np.ndarray, np.ndarray]], List[Tuple[float, float]]]: list of
                craters in each block, coordinates of the blocks.
        """

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
            _, _, matrix = self.crater_db.get_blocks_within_region_with_neighbors(
                region
            )
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

            # Get all the existing craters in the region (should be empty except
            # for the neighbors). This is done for the hardcore rejection.
            blocks, _, _ = self.crater_db.get_blocks_within_region_with_neighbors(
                new_region
            )

            # Converts the metadatas into numpy arrays
            prev_coords = self.crater_metadata_gen.castMetadata(blocks)

            # Samples craters in the region
            new_blocks = self.crater_dist_gen.run(new_region, prev_coords=prev_coords)

            # Dissects the region into blocks and adds the craters to the database
            new_blocks_list, block_coordinates_list = self.dissect_region_blocks(
                new_blocks, region
            )
            for (coordinates, radius), block_coordinates in zip(
                new_blocks_list, block_coordinates_list
            ):
                metadata = self.crater_metadata_gen.run(coordinates, radius)
                self.crater_db.add_block_data(metadata, block_coordinates)

        # If the largest rectangle is smaller or equal to 1 block, we sample craters
        # on a per block basis.
        coordinates = self.crater_db.get_missing_blocks(region)
        for coord in coordinates:
            self.sample_craters_by_block(coord)

    def display_block(self, coordinates: Tuple[float, float]):
        """
        Displays the craters in a block.

        Args:
            coordinates (Tuple[float,float]): coordinates of the block.
        """
        fig = plt.figure(figsize=(10, 10), dpi=300)
        if self.crater_db.check_block_exists(coordinates):
            block = self.crater_db.get_block_data(coordinates)
            coordinates, radius = self.crater_metadata_gen.castMetadata(block)
            ppm = 3000 / self.settings.block_size / 5
            plt.scatter(coordinates[:, 0], coordinates[:, 1], s=radius * ppm)
            plt.axis("equal")

    def display_block_set(self, coords: List[Tuple[float, float]]):
        """
        Displays a set of blocks. Each block is represented by a different color.

        Args:
            coords (List[Tuple[float,float]]): list of coordinates of the blocks.
        """
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

    def display_region(self, region: BoundingBox):
        """
        Displays the craters in a region. Each block is represented by a different color.

        Args:
            region (BoundingBox): region to display.
        """

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
