from typing import List, Tuple, Dict
import numpy as np
import dataclasses
import threading
import time
import copy
import os

import omni

from src.terrain_management.large_scale_terrain.rock_distribution import (
    RockSamplerCfg,
    RockSampler,
)
from src.terrain_management.large_scale_terrain.rock_database import RockDB, RockDBCfg
from WorldBuilders.pxr_utils import createInstancerAndCache, setInstancerParameters
from src.terrain_management.large_scale_terrain.utils import BoundingBox, RockBlockData

from pxr import UsdGeom, Gf, Usd
from omni.physx.scripts import utils as physx_utils


class OGInstancer:
    """
    The Original Gangster: the point instancer.
    """

    def __init__(self, instancer_path: str, assets_path: str, seed: int):
        """
        Args:
            instancer_path (str): The path to the instancer.
            assets_path (str): The path to the assets folder.
            seed (int): The seed for the random number generator.
        """

        self.instancer_path = instancer_path
        self.stage = omni.usd.get_context().get_stage()
        self.assets_path = assets_path
        self.prototypes = []
        self.get_asset_list()
        self.createInstancer()
        self.rng = np.random.default_rng(seed=seed)

    def get_asset_list(self):
        """
        Get the list of assets in the assets folder.
        """

        self.file_paths = [
            os.path.join(self.assets_path, file)
            for file in os.listdir(self.assets_path)
            if file.endswith(".usd")
        ]
        print(self.file_paths)
        print(len(self.file_paths))

    def createInstancer(self):
        """
        Create the instancer.
        """

        self.instancer_path = omni.usd.get_stage_next_free_path(
            self.stage, self.instancer_path, False
        )
        self.instancer = UsdGeom.PointInstancer.Define(self.stage, self.instancer_path)
        self.instancer_prim = self.instancer.GetPrim()

        prim_paths = []
        for i, file_path in enumerate(self.file_paths):
            prim = self.stage.DefinePrim(
                os.path.join(self.instancer_path, "asset_" + str(i)), "Xform"
            )
            prim.GetReferences().AddReference(file_path)
            iterator = iter(Usd.PrimRange(prim))
            for prim in iterator:
                physx_utils.removeCollider(prim)
            prim_paths.append(prim.GetPath())

        self.instancer.GetPrototypesRel().SetTargets(prim_paths)

        self.setInstancerParameters(
            np.array([[0, 0, 0]]),
            np.array([[0, 0, 0, 1]]),
            np.array([[1, 1, 1]]),
            np.array([0]),
        )

    def setInstancerParameters(self, positions, orientations, scales, ids):
        """
        Set the instancer parameters.
        """

        print("positions shape", positions.shape)
        print("orientations shape", orientations.shape)
        print("scales shape", scales.shape)
        print("ids shape", ids.shape)
        print("num prototypes rel", len(self.instancer.GetPrototypesRel().GetTargets()))
        start = time.time()
        self.instancer.GetPositionsAttr().Set(positions)
        self.instancer.GetOrientationsAttr().Set(orientations)
        self.instancer.GetScalesAttr().Set(scales)
        self.instancer.GetProtoIndicesAttr().Set(ids)
        end = time.time()
        print("Set instancer parameters took", end - start, "seconds")


@dataclasses.dataclass
class RockGeneratorCfg:
    """
    Args:
        rock_db_cfg (RockDBCfg): The configuration for the rock database.
        rock_sampler_cfg (RockSamplerCfg): The configuration for the rock sampler.
        rock_assets_folder (str): The path to the rock assets folder.
        instancer_name (str): The name of the instancer.
        seed (int): The seed for the random number generator.
        block_span (int): The number of blocks to sample around the position.
        block_size (int): The size of the blocks in meters.
    """

    rock_db_cfg: RockDBCfg = dataclasses.field(default_factory=dict)
    rock_sampler_cfg: RockSamplerCfg = dataclasses.field(default_factory=dict)
    rock_assets_folder: str = dataclasses.field(default_factory=list)
    instancer_name: str = dataclasses.field(default_factory=str)
    seed: int = dataclasses.field(default_factory=int)
    block_span: int = dataclasses.field(default_factory=int)
    block_size: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        assert self.instancer_name != "", "Instancer name cannot be empty"
        assert self.block_span >= 0, "Block span must be greater or equal to 0"
        assert os.path.exists(self.rock_assets_folder), "Rock assets folder not found"

        self.rock_db_cfg = RockDBCfg(**self.rock_db_cfg)
        self.rock_sampler_cfg = RockSamplerCfg(**self.rock_sampler_cfg)


class RockGenerator:
    """
    The rock generator is responsible for generating rocks on the terrain. It samples the rocks
    around a given position and updates the instancer with the new data.
    """

    def __init__(
        self,
        settings: RockGeneratorCfg,
        sampling_func: callable,
        is_map_done: callable,
        instancer_path: str,
    ):
        """
        Args:
            settings (RockGeneratorCfg): The settings for the rock generator.
            sampling_func (callable): The function to sample the rocks.
            is_map_done (callable): The function to check if the map is done.
            instancer_path (str): The path to the instancer.
        """

        self.settings = settings
        self.sampling_func = sampling_func
        self.is_map_done = is_map_done
        self.instancer_path = instancer_path
        self.is_sampling = False

    def build(self) -> None:
        """
        Builds the rock generator. It initializes the rock database, the rock sampler, and the
        rock instancer.
        """

        self.rock_db = RockDB(self.settings.rock_db_cfg)
        self.rock_sampler = RockSampler(
            self.settings.rock_sampler_cfg,
            self.rock_db,
            map_sampling_func=self.sampling_func,
        )
        self.rock_instancer = OGInstancer(
            os.path.join(self.instancer_path, self.settings.instancer_name),
            self.settings.rock_assets_folder,
            self.settings.seed,
        )

    def cast_coordinates_to_block_space(
        self, coordinates: Tuple[float, float]
    ) -> Tuple[int, int]:
        """
        Casts the given coordinates to the block space. The block space is the space
        where the blocks are defined. The block space is defined by the block size and
        the resolution of the high resolution DEM.

        The coordinates are still expressed in meters, but they can only be an increment of
        the block size (in meters).

        Args:
            coordinates (Tuple[float, float]): The coordinates to cast to the block space.

        Returns:
            Tuple[int, int]: The coordinates in the block space.
        """

        x, y = coordinates
        x_block = int(x // self.settings.block_size) * self.settings.block_size
        y_block = int(y // self.settings.block_size) * self.settings.block_size
        return (x_block, y_block)

    def define_region(self, coordinates: Tuple[float, float]) -> BoundingBox:
        """
        Defines the region to sample the rocks. The region is defined by the coordinates
        and the block span. The block span is the number of blocks to sample around the
        coordinates.

        The region is defined by the following formula:
            x_low = coordinates[0] - (block_span + 1) * block_size
            x_high = coordinates[0] + (block_span + 2) * block_size
            y_low = coordinates[1] - (block_span + 1) * block_size
            y_high = coordinates[1] + (block_span + 2) * block_size

        Note the +1 on the minimum and +2 on the maximum. This is done to have a buffer of
        blocks around the sampling region.

        Args:
            coordinates (Tuple[float, float]): The coordinates to define the region around.

        Returns:
            BoundingBox: The bounding box of the region.
        """

        x_low = (
            coordinates[0] - (self.settings.block_span + 1) * self.settings.block_size
        )
        x_high = (
            coordinates[0] + (self.settings.block_span + 2) * self.settings.block_size
        )
        y_low = (
            coordinates[1] - (self.settings.block_span + 1) * self.settings.block_size
        )
        y_high = (
            coordinates[1] + (self.settings.block_span + 2) * self.settings.block_size
        )

        return BoundingBox(x_min=x_low, x_max=x_high, y_min=y_low, y_max=y_high)

    def aggregate_block_data(
        self, blocks: List[RockBlockData]
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Aggregates the block data into numpy arrays. The block data is a list of dictionaries
        where each dictionary contains the following keys:
            - position: The position of the block.
            - orientation: The orientation of the block.
            - scale: The scale of the block.
            - id: The id of the block.

        Args:
            blocks (List[RockBlockData]): The list of block data to aggregate.

        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: The position, orientation,
            scale, and ids as numpy arrays
        """

        position = np.vstack([block.coordinates for block in blocks])
        orientation = np.vstack([block.quaternion for block in blocks])
        scale = np.vstack([block.scale for block in blocks])
        ids = np.hstack([block.ids for block in blocks])

        return position, orientation, scale, ids

    def update_instancer(self, region: BoundingBox) -> None:
        """
        Updates the instancer with the new block data. The function gets the blocks within
        the region and aggregates the block data. It then sets the instancer's parameters
        with the new data.

        Args:
            region (BoundingBox): The region to sample the rocks.
        """

        blocks, _, _ = self.rock_db.get_blocks_within_region(region)
        print("blocks retrieved")
        position, orientation, scale, ids = self.aggregate_block_data(blocks)
        print("num_objects", len(position))
        self.rock_instancer.setInstancerParameters(position, orientation, scale, ids)
        print("instancer set")

    def sample(self, global_position: Tuple[float, float]) -> None:
        """
        Samples the rocks at the given position. The function casts the position to the block
        space and defines the region around the position. It then samples the rocks within the
        region and updates the instancer with the new data.

        Args:
            position (Tuple[float, float]): The position to sample the rocks.
            map_coordinates (Tuple[float, float]): The coordinates in the map space.
        """

        while not self.is_map_done():
            time.sleep(0.1)

        self.is_sampling = True
        coordinates = self.cast_coordinates_to_block_space(global_position)
        region = self.define_region(coordinates)
        print("region", region)
        self.rock_sampler.sample_rocks_by_region(region, global_position)
        print("sampling done")
        self.update_instancer(region)
        print("instancer updated")
        self.is_sampling = False

    def _sample_threaded(self, position: Tuple[float, float]) -> None:
        """
        Internal function DO NOT CALL.

        This function is used to sample the rocks in a separate thread. It waits for potential previous
        calls to finish before starting a new one.

        Args:
            position (Tuple[float, float]): The position to sample the rocks.
        """

        while self.is_sampling:
            time.sleep(0.1)

        self.sample(position)

    def sample_threaded(self, position: Tuple[float, float]) -> None:
        """
        Samples the rocks at the given position in a separate thread. The function waits for potential
        previous calls to finish before starting a new one.

        Args:
            position (Tuple[float, float]): The position to sample the rocks.
        """

        thread = threading.Thread(target=self._sample_threaded, args=(position,))
        thread.start()


@dataclasses.dataclass
class RockManagerCfg:
    """
    Args:
        rock_gen_cfgs (List[RockGeneratorCfg]): The configuration for the rock generators.
        instancers_path (str): The path to the instancers.
        seed (int): The seed for the random number generator.
    """

    rock_gen_cfgs: List[RockGeneratorCfg] = dataclasses.field(default_factory=list)
    instancers_path: str = dataclasses.field(default_factory=str)
    seed: int = dataclasses.field(default_factory=int)
    block_size: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        self.rock_gen_cfgs = [RockGeneratorCfg(**cfg) for cfg in self.rock_gen_cfgs]


class RockManager:
    """
    The rock manager is responsible for managing the rocks on the terrain. It manages multiple
    rock generators and samples the rocks around a given position.
    """

    def __init__(
        self, settings: RockManagerCfg, sampling_func: callable, is_map_done: callable
    ) -> None:
        """
        Args:
            settings (RockManagerCfg): The settings for the rock manager.
            sampling_func (callable): The function to sample the rocks height and normals.
            is_map_done (callable): The function to check if the map is done.
        """

        self.settings = settings
        self.sampling_func = sampling_func
        self.is_map_done = is_map_done
        self.last_update = None

    def build(self) -> None:
        """
        Builds the rock manager. It initializes the rock generators.
        """

        self.rock_generators: List[RockGenerator] = []

        for rock_gen_cfg in self.settings.rock_gen_cfgs:
            rock_generator = RockGenerator(
                rock_gen_cfg,
                self.sampling_func,
                self.is_map_done,
                self.settings.instancers_path,
            )
            rock_generator.build()
            self.rock_generators.append(rock_generator)

    def cast_coordinates_to_block_space(
        self, coordinates: Tuple[float, float]
    ) -> Tuple[int, int]:
        """
        Casts the given coordinates to the block space. The block space is the space
        where the blocks are defined. The block space is defined by the block size and
        the resolution of the high resolution DEM.

        The coordinates are still expressed in meters, but they can only be an increment of
        the block size (in meters).

        Args:
            coordinates (Tuple[float, float]): The coordinates to cast to the block space.

        Returns:
            Tuple[int, int]: The coordinates in the block space.
        """

        x, y = coordinates
        x_block = int(x // self.settings.block_size) * self.settings.block_size
        y_block = int(y // self.settings.block_size) * self.settings.block_size
        return (x_block, y_block)

    def check_if_update_needed(self, position: Tuple[float, float]) -> bool:
        """
        Checks if an update is needed. The function checks if the position is different from the
        last update in the block space coordinates.

        Args:
            position (Tuple[float, float]): The position to check. (block space coordinates)
        """
        if self.last_update is None:
            return True
        else:
            x, y = position
            x_last, y_last = self.last_update
            return (x != x_last) or (y != y_last)

    def sample(self, global_position: Tuple[float, float]) -> None:
        """
        Samples the rocks at the given position. The function samples the rocks for each rock
        generator.

        Args:
            global_position (Tuple[float, float]): The position to sample the rocks.
            map_coordinates (Tuple[float, float]): The coordinates in the map space.
        """
        position = self.cast_coordinates_to_block_space(global_position)

        if self.check_if_update_needed(position):
            self.last_update = position
            print("updating rocks")
            for rock_generator in self.rock_generators:
                rock_generator.sample(position)

    def sample_threaded(self, global_position: Tuple[float, float]) -> None:
        """
        Samples the rocks at the given position in a separate thread. The function samples the rocks
        for each rock generator in a separate thread.

        Args:
            global_position (Tuple[float, float]): The position to sample the rocks.
        """

        position = self.cast_coordinates_to_block_space(global_position)
        if self.check_if_update_needed(position):
            for rock_generator in self.rock_generators:
                rock_generator.sample_threaded(position)
