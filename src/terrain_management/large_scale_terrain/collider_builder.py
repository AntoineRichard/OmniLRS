__author__ = "Antoine Richard"
__copyright__ = "Copyright 2024, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import Tuple
import numpy as np
import dataclasses
import os

from pxr import UsdGeom, Gf, Usd

from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops, add_collider, remove_collider


@dataclasses.dataclass
class ColliderBuilderConf:
    """
    Args:
        resolution (float): resolution of the collider (meter per pixel).
        block_size (int): size of the block (meters).
        collider_path (str): path to the collider.
        base_name (str): base name of the collider.
        collider_mode (str): mode of the collider.
        visible (bool): visibility of the collider. True means visible.
    """

    resolution: float = dataclasses.field(default_factory=float)
    block_size: int = dataclasses.field(default_factory=int)
    collider_path: str = dataclasses.field(default_factory=str)
    base_name: str = dataclasses.field(default_factory=str)
    collider_mode: str = dataclasses.field(default_factory=str)
    visible: bool = dataclasses.field(default_factory=bool)


class ColliderBuilder:
    """
    Class to build a collider mesh for the terrain.
    """

    def __init__(self, settings: ColliderBuilderConf, stage: Usd.Stage) -> None:
        """
        Args:
            settings (ColliderBuilderConf): settings of the collider.
            stage (Usd.Stage): stage to create the collider
        """

        self.settings = settings
        self.stage = stage
        self.indices = []

    @staticmethod
    def grid_index(x: int, y: int, stride: int) -> int:
        """
        Returns the index of the grid point at (x, y) in a grid of width stride.

        Args:
            x (int): x coordinate of the grid point.
            y (int): y coordinate of the grid point.
            stride (int): width of the grid.

        Returns:
            int: index of the grid point at (x, y)."""

        return y * stride + x

    def build_base_grid(self) -> None:
        """
        Builds the grid of vertices and indices for the terrain mesh.
        Also generates the UVs coordinates for the terrain mesh."""

        block_size_pixels = int(self.settings.block_size / self.settings.resolution) + 1

        vertices = []
        for y in range(
            0,
            block_size_pixels,
        ):
            for x in range(
                0,
                block_size_pixels,
            ):
                pos = (
                    float(x) * self.settings.resolution,
                    float(y) * self.settings.resolution,
                    float(0),
                )

                # Generates vertices indices for the terrain mesh.
                vertices.append(pos)
                if x > 0 and y > 0:  # Two triangles per pixel.
                    # First triangle.
                    self.indices.append(self.grid_index(x, y - 1, block_size_pixels))
                    self.indices.append(self.grid_index(x, y, block_size_pixels))
                    self.indices.append(self.grid_index(x - 1, y - 1, block_size_pixels))
                    # Second triangle.
                    self.indices.append(self.grid_index(x, y, block_size_pixels))
                    self.indices.append(self.grid_index(x - 1, y, block_size_pixels))
                    self.indices.append(self.grid_index(x - 1, y - 1, block_size_pixels))

        self.sim_verts = np.array(vertices, dtype=np.float32)
        self.indices = np.array(self.indices).reshape(-1, 3)
        self.num_indices = [3] * len(self.indices)

    def create_collider(self, position: Tuple[float, float], map: np.ndarray, uid: int) -> None:
        """
        Creates a mesh with a collider at the given position.
        It uses the data inside map to assign the mesh vertices heights.

        Args:
            position (Tuple[float, float]): position of the mesh.
            map (np.ndarray): map to use for the mesh vertices heights.
            uid (int): unique identifier of the mesh.
        """

        self.sim_verts[:, -1] = map.flatten()
        mesh_path = os.path.join(self.settings.collider_path, self.settings.base_name + str(uid))

        mesh = UsdGeom.Mesh.Define(self.stage, mesh_path)
        mesh.GetPointsAttr().Set(self.sim_verts)
        mesh.GetFaceVertexIndicesAttr().Set(self.indices)
        mesh.GetFaceVertexCountsAttr().Set(self.num_indices)
        UsdGeom.Primvar(mesh.GetDisplayColorAttr()).SetInterpolation("vertex")
        mesh.GetDisplayColorAttr().Set(np.ones_like(self.indices))

        set_xform_ops(mesh, translate=Gf.Vec3d(position[0], position[1], position[2]))
        add_collider(self.stage, mesh.GetPath(), mode=self.settings.collider_mode)

    def update_collider(self, map: np.ndarray, uid: int) -> None:
        """
        Updates the mesh vertices heights given the data inside map.
        Also updates the collider of the mesh.

        Args:
            map (np.ndarray): map to use for the mesh vertices heights.
            uid (int): unique identifier of the mesh.
        """
        self.sim_verts[:, -1] = map.flatten()
        mesh_path = os.path.join(self.settings.collider_path, self.settings.base_name + str(uid))
        mesh = UsdGeom.Mesh.Get(self.stage, mesh_path)

        mesh.GetPointsAttr().Set(self.sim_verts)
        remove_collider(self.stage, mesh.GetPath())
        add_collider(self.stage, mesh.GetPath(), mode=self.settings.collider_mode)
