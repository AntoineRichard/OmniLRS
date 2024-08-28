from typing import Tuple
import dataclasses
import numpy as np
import cv2

from omni.isaac.core.physics_context.physics_context import PhysicsContext
import omni


from src.terrain_management.large_scale_terrain.collider_builder import ColliderBuilder, ColliderBuilderCfg
from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops


@dataclasses.dataclass
class PhysicsSceneManagerCfg:
    dt: float = dataclasses.field(default_factory=float)
    gravity: Tuple[float, float, float] = None
    substeps: int = None
    use_gpu_pipeline: bool = None
    worker_thread_count: int = None
    use_fabric: bool = None
    enable_scene_query_support: bool = None
    gpu_max_rigid_contact_count: int = None
    gpu_max_rigid_patch_contact_count: int = None
    gpu_found_lost_pairs_capacity: int = None
    gpu_total_aggregate_pairs_capacity: int = None
    gpu_max_soft_body_contacts: int = None
    gpu_max_particle_contacts: int = None
    gpu_heap_capacity: int = None
    gpu_temp_buffer_capacity: int = None
    gpu_max_num_partions: int = None
    gpu_collision_stack_size: int = None
    solver_type: str = None
    enable_stabilization: bool = None
    bounce_threshold_velocity: float = None
    friction_offset_threshold: float = None
    friction_correction_distance: float = None
    enable_ccd: bool = None

    def __post_init__(self):
        self.physics_scene_args = {}
        for attribute in dataclasses.fields(self):
            if getattr(self, attribute.name) is not None:
                self.physics_scene_args[attribute.name] = getattr(self, attribute.name)


class PhysicsSceneManager:
    def __init__(self, settings: PhysicsSceneManagerCfg) -> None:
        self.settings = settings
        self.physics_context = PhysicsContext(sim_params=self.settings.physics_scene_args)
        if self.settings.enable_ccd:
            self.physics_context.enable_ccd(True)


@dataclasses.dataclass
class ColliderManagerCfg:
    """
    Args:
        collider_resolution (float): resolution of the collider (meter per pixel).
        source_resolution (float): resolution of the source (meter per pixel).
        block_size (int): size of the block (meters).
        collider_path (str): path to the collider.
        collider_builder_cfg (dict): configuration for the collider builder.
    """

    collider_resolution: float = dataclasses.field(default_factory=float)
    source_resolution: float = dataclasses.field(default_factory=float)
    block_size: int = dataclasses.field(default_factory=int)
    collider_path: str = dataclasses.field(default_factory=str)
    collider_builder_cfg: ColliderBuilderCfg = dataclasses.field(default_factory=dict)

    def __post_init__(self):
        self.collider_builder_cfg = ColliderBuilderCfg(**self.collider_builder_cfg)


class ColliderManager:
    """
    Class to manage the colliders for the terrain.
    We build 9 colliders that we update as the robot moves around.
    Both the colliders geometries and their positions are updated.
    The object of interest is always at the center of the 3x3 grid.

    x --- x --- x --- x
    |     |     |     |
    x --- x --- x --- x
    |     |robot|     |
    x --- x --- x --- x
    |     |     |     |
    x --- x --- x --- x
    """

    def __init__(
        self,
        settings: ColliderManagerCfg,
        hr_dem: np.ndarray,
        hr_dem_shape: Tuple[int, int],
        dem_center: Tuple[float, float],
    ) -> None:
        """
        Args:
            settings (ColliderManagerCfg): configuration for the collider manager.
            hr_dem (np.ndarray): high resolution DEM.
            hr_dem_shape (Tuple[int, int]): shape of the high resolution DEM.
            dem_center (Tuple[float, float]): center of the DEM.
        """

        self.settings = settings
        self.hr_dem = hr_dem
        self.hr_dem_shape = hr_dem_shape
        self.dem_center = dem_center
        self.stage = omni.usd.get_context().get_stage()
        self.block_hw = int(self.settings.block_size / self.settings.collider_resolution) + 1

    def build(self) -> None:
        """
        Builds the collider builder and applies it to generate a collider grid.
        """

        self.collider_builder = ColliderBuilder(self.settings.collider_builder_cfg, self.stage)
        self.collider_builder.build_base_grid()
        self.build_collider_grid()

    def build_collider_grid(self) -> None:
        """
        Builds the collider grid.
        """

        self.prim = self.stage.DefinePrim(self.settings.collider_path, "Xform")
        set_xform_ops(self.prim)

        # Build the collider grid
        uid = 0
        for y in range(-self.settings.block_size, self.settings.block_size * 2, self.settings.block_size):
            for x in range(-self.settings.block_size, self.settings.block_size * 2, self.settings.block_size):
                data = self.get_terrain_block((x, y))
                self.collider_builder.create_collider((x, y, 0), data, uid)
                uid += 1

    def update_collider(self, position: Tuple[float, float]) -> None:
        """
        Updates the collider grid with the given position and uses the dem to change their geometry.

        Args:
            position (Tuple[float, float]): position of the object of interest.
        """

        uid = 0
        position_update = (position[0], position[1], 0)
        set_xform_ops(self.prim, position_update)
        for y in range(-self.settings.block_size, self.settings.block_size * 2, self.settings.block_size):
            for x in range(-self.settings.block_size, self.settings.block_size * 2, self.settings.block_size):
                data = self.get_terrain_block((x + position[0], y + position[1]))
                self.collider_builder.update_collider(data, uid)
                uid += 1

    def get_terrain_block(self, coords: Tuple[float, float]) -> None:
        """
        Returns a terrain block given the coordinates.

        Args:
            coords (Tuple[float, float]): coordinates of the block.
        """

        x_min = (self.dem_center[0] + coords[0]) / self.settings.source_resolution
        x_max = (self.dem_center[0] + coords[0] + self.settings.block_size) / self.settings.source_resolution
        y_min = (self.dem_center[1] + coords[1]) / self.settings.source_resolution
        y_max = (self.dem_center[1] + coords[1] + self.settings.block_size) / self.settings.source_resolution
        source_data_block = self.hr_dem[int(y_min) : int(y_max) + 1, int(x_min) : int(x_max) + 1]
        return cv2.resize(source_data_block, (self.block_hw, self.block_hw), interpolation=cv2.INTER_LINEAR)
