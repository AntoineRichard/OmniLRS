__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import Tuple, List
import numpy as np
import dataclasses
import sys

from src.terrain_management.large_scale_terrain.utils import BoundingBox, RockBlockData


@dataclasses.dataclass
class RockDBCfg:
    """
    Args:
        block_size (int): size of the block in meters
        max_blocks (int): maximum number of blocks to store
        save_to_disk (bool): flag to save the database to disk
        write_to_disk_interval (int): interval to write to disk
    """

    block_size: int = 50
    max_blocks: int = int(1e7)
    save_to_disk: bool = False
    write_to_disk_interval: int = 1000


class RockDB:
    """
    Class to manage the crater database.
    """

    def __init__(self, cfg: RockDBCfg) -> None:
        """
        Args:
            cfg (CraterDBCfg): configuration for the database.
        """

        self.crater_db = {}
        self.profile_db = {}
        self.rock_db_config = cfg

    def add_block_data(
        self, block_data: RockBlockData, block_coordinates: Tuple[float, float]
    ) -> None:
        """
        Adds a block of data to the database. That is, a list of crater metadata
        indexed by the block coordinates.

        Args:
            block_data (List[CraterMetadata]): list of crater metadata.
            block_coordinates (Tuple[float, float]): coordinates of the block.
        """

        assert (
            block_coordinates[0] % self.rock_db_config.block_size == 0
        ), "Block x-coordinate must be a multiple of the block size."
        assert (
            block_coordinates[1] % self.rock_db_config.block_size == 0
        ), "Block y-coordinate must be a multiple of the block size."

        self.crater_db[block_coordinates] = block_data

    def is_valid(self, block_coordinates) -> bool:
        """
        Checks if the block coordinates are valid. Blocks must be a multiple of the block size.
        This function asserts that the x and y coordinates are multiples of the block size.

        Args:
            block_coordinates (Tuple[float, float]): coordinates of the block.

        Returns:
            bool: True if the block coordinates are valid.
        """

        assert (
            block_coordinates[0] % self.rock_db_config.block_size == 0
        ), "Block x-coordinate must be a multiple of the block size."
        assert (
            block_coordinates[1] % self.rock_db_config.block_size == 0
        ), "Block y-coordinate must be a multiple of the block size."
        return True

    def get_block_data(
        self, block_coordinates: Tuple[float, float]
    ) -> RockBlockData:
        """
        Gets the block data with the given coordinates.

        Args:
            block_coordinates (Tuple[float, float]): coordinates of the block.

        Returns:
            List[CraterMetadata]: list of crater metadata.
        """

        return self.crater_db[block_coordinates]

    def get_block_data_with_neighbors(
        self, block_coordinates: Tuple[float, float]
    ) -> RockBlockData:
        """
        Gets the block data with the given coordinates and its neighbors.
        That is 1 block in each direction, so the resulting block is
        a list composed 3x3 blocks.

        Args:
            block_coordinates (Tuple[float, float]): coordinates of the block.

        Returns:
            List[CraterMetadata]: list of crater metadata.
        """

        blocks = []
        for x in range(-1, 2, 1):
            xc = block_coordinates[0] + x * self.rock_db_config.block_size
            for y in range(-1, 2, 1):
                yc = block_coordinates[1] + y * self.rock_db_config.block_size
                if self.check_block_exists((xc, yc)):
                    blocks += self.get_block_data((xc, yc))
        return blocks

    def check_block_exists(self, block_coordinates: Tuple[float, float]) -> bool:
        """
        Checks if the block with the given coordinates exists in the database.

        Args:
            block_coordinates (Tuple[float, float]): coordinates of the block.

        Returns:
            bool: True if the block exists in the database.
        """

        return block_coordinates in self.crater_db

    def get_missing_blocks(self, region: BoundingBox) -> List[Tuple[float, float]]:
        """
        Gets the missing blocks in the given region. That is, the blocks that are not
        in the database for the given region.

        Args:
            region (BoundingBox): region to check for missing blocks.

        Returns:
            List[Tuple[float, float]]: list of missing blocks' coordinates.
        """

        missing_blocks = []
        x_min_rem = region.x_min % self.rock_db_config.block_size
        y_min_rem = region.y_min % self.rock_db_config.block_size
        x_max_rem = region.x_max % self.rock_db_config.block_size
        y_max_rem = region.y_max % self.rock_db_config.block_size

        x_min = region.x_min - x_min_rem
        y_min = region.y_min - y_min_rem
        x_max = region.x_max - x_max_rem
        y_max = region.y_max - y_max_rem

        for x in range(x_min, x_max, self.rock_db_config.block_size):
            for y in range(y_min, y_max, self.rock_db_config.block_size):
                if not self.check_block_exists((x, y)):
                    missing_blocks.append((x, y))
        return missing_blocks

    def get_blocks_within_region(self, region: BoundingBox) -> RockBlockData:
        """
        Gets the blocks within the given region. That is, the blocks that are in the
        database for the given region.

        Args:
            region (BoundingBox): region to check for blocks.

        Returns:
            List[CraterMetadata]: list of crater metadata.
        """

        blocks = []

        x_min_rem = region.x_min % self.rock_db_config.block_size
        y_min_rem = region.y_min % self.rock_db_config.block_size
        x_max_rem = region.x_max % self.rock_db_config.block_size
        y_max_rem = region.y_max % self.rock_db_config.block_size

        x_min = region.x_min - x_min_rem
        y_min = region.y_min - y_min_rem
        x_max = region.x_max - x_max_rem
        y_max = region.y_max - y_max_rem

        new_region = BoundingBox(x_min, x_max, y_min, y_max)

        occupied_block_matrix = np.zeros(
            (
                (int((x_max - x_min) / self.rock_db_config.block_size)),
                (int((y_max - y_min) / self.rock_db_config.block_size)),
            ),
            dtype=int,
        )

        for x in range(x_min, x_max, self.rock_db_config.block_size):
            for y in range(y_min, y_max, self.rock_db_config.block_size):
                if self.check_block_exists((x, y)):
                    blocks += self.get_block_data((x, y))

                    occupied_block_matrix[
                        int((x - x_min) / self.rock_db_config.block_size),
                        int((y - y_min) / self.rock_db_config.block_size),
                    ] = 1
        return (
            blocks,
            new_region,
            occupied_block_matrix,
        )

    def get_blocks_within_region_with_neighbors(
        self, region: BoundingBox
    ) -> RockBlockData:
        """
        Gets the blocks within the given region and its neighbors. It collects all
        the block available in the given region, plus 1 block in each direction.

        Args:
            region (BoundingBox): region to check for blocks.

        Returns:
            List[CraterMetadata]: list of crater metadata.
        """

        blocks = []

        x_min_rem = region.x_min % self.rock_db_config.block_size
        y_min_rem = region.y_min % self.rock_db_config.block_size
        x_max_rem = region.x_max % self.rock_db_config.block_size
        y_max_rem = region.y_max % self.rock_db_config.block_size

        x_min = region.x_min - x_min_rem - self.rock_db_config.block_size
        y_min = region.y_min - y_min_rem - self.rock_db_config.block_size
        x_max = region.x_max - x_max_rem + self.rock_db_config.block_size
        y_max = region.y_max - y_max_rem + self.rock_db_config.block_size

        new_region = BoundingBox(x_min, x_max, y_min, y_max)

        occupied_block_matrix = np.zeros(
            (
                (int((x_max - x_min) / self.rock_db_config.block_size)),
                (int((y_max - y_min) / self.rock_db_config.block_size)),
            ),
            dtype=int,
        )

        for x in range(x_min, x_max, self.rock_db_config.block_size):
            for y in range(y_min, y_max, self.rock_db_config.block_size):
                if self.check_block_exists((x, y)):
                    blocks += self.get_block_data((x, y))
                    occupied_block_matrix[
                        int((x - x_min) / self.rock_db_config.block_size),
                        int((y - y_min) / self.rock_db_config.block_size),
                    ] = 1

        return (
            blocks,
            new_region,
            occupied_block_matrix[1:-1, 1:-1],
        )

    def get_all_blocks(self) -> RockBlockData:
        """
        Gets all the blocks in the database.

        Returns:
            List[CraterMetadata]: list of crater metadata.
        """

        blocks = []
        for block in self.crater_db.values():
            blocks += block
        return blocks

    def get_memory_footprint(self) -> int:
        """
        Gets the memory footprint of the database.

        Returns:
            int: memory footprint of the database in bytes.
        """

        return sys.getsizeof(self.crater_db), sys.getsizeof(self.profile_db)
