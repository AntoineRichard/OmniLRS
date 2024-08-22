__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from src.terrain_management.large_scale_terrain.geometric_clipmaps import (
    GeoClipmapSpecs,
    GeoClipmap,
)
from dataclasses import dataclass, field
from WorldBuilders import pxr_utils
from pxr import UsdGeom, Sdf
from typing import Tuple
import numpy as np
import warp as wp
import omni


@dataclass
class GeoClipmapManagerConf:
    """
    Args:
        root_path (str): path to the root of the clipmap.
        geo_clipmap_specs (GeoClipmapSpecs): specifications for the clipmap.
        mesh_position (tuple): position of the mesh.
        mesh_orientation (tuple): orientation of the mesh.
        mesh_scale (tuple): scale of the mesh.
    """

    root_path: str = "/World"
    geo_clipmap_specs: GeoClipmapSpecs = field(default_factory=dict)
    mesh_position: tuple = field(default_factory=np.array)
    mesh_orientation: tuple = field(default_factory=np.array)
    mesh_scale: tuple = field(default_factory=np.array)


class GeoClipmapManager:
    """
    Class to manage the geometric clipmap.
    """

    def __init__(
        self,
        cfg: GeoClipmapManagerConf,
        interpolation_method: str = "bilinear",
        acceleration_mode: str = "hybrid",
        name_prefix: str = "",
        profiling: bool = False,
    ) -> None:
        """
        Args:
            cfg (GeoClipmapManagerConf): configuration for the clipmap.
            interpolation_method (str): method to use for interpolation.
            acceleration_mode (str): mode to use for acceleration.
            name_prefix (str): prefix to add to the mesh name.
        """

        self._stage = omni.usd.get_context().get_stage()
        self._geo_clipmap = GeoClipmap(
            cfg.geo_clipmap_specs,
            interpolation_method=interpolation_method,
            acceleration_mode=acceleration_mode,
        )
        self._root_path = cfg.root_path
        self.profiling = profiling

        self._mesh_pos = cfg.mesh_position
        self._mesh_rot = cfg.mesh_orientation
        self._mesh_scale = cfg.mesh_scale

        self._og_mesh_path = self._root_path + "/Terrain/terrain_mesh" + name_prefix
        self._mesh_path = self._root_path + "/Terrain/terrain_mesh" + name_prefix

        self.createXforms()
        self.update_topology = True

    def build(self, dem: np.ndarray, dem_shape: Tuple[int, int]) -> None:
        """
        Builds the clipmap with the given dem.

        Args:
            dem (np.ndarray): dem to use for the clipmap.
            dem_shape (Tuple[int, int]): shape of the dem.
        """

        self._geo_clipmap.build(dem, dem_shape)

    def updateDEMBuffer(self) -> None:
        """
        Updates the DEM buffer of the clipmap.
        """

        self._geo_clipmap.updateDEMBuffer()

    def updateGeoClipmap(self, position: np.ndarray, mesh_position: np.ndarray) -> None:
        """
        Updates the clipmap with the given position and mesh position.

        Args:
            position (np.ndarray): position to use for the update (in meters).
            mesh_position (np.ndarray): position of the mesh (in meters).
        """
        with wp.ScopedTimer("complete update loop", active=self.profiling):
            self._geo_clipmap.updateElevation(position)
            with wp.ScopedTimer("mesh update", active=self.profiling):
                self.renderMesh(
                    self._geo_clipmap.points,
                    self._geo_clipmap.indices,
                    self._geo_clipmap.uvs,
                    update_topology=self.update_topology,
                )
        self.moveMesh(mesh_position)
        self.update_topology = False

    def moveMesh(self, position: np.ndarray) -> None:
        """
        Moves the mesh to the given position.

        Args:
            position (np.ndarray): position to move the mesh to (in meters).
        """

        self._mesh_pos = position
        mesh = UsdGeom.Mesh.Get(self._stage, self._mesh_path)
        if mesh:
            pxr_utils.setDefaultOps(
                mesh, self._mesh_pos, self._mesh_rot, self._mesh_scale
            )

    def createXforms(self) -> None:
        """
        Creates the xforms for the clipmap.
        """

        if not self._stage.GetPrimAtPath(self._root_path):
            pxr_utils.createXform(self._stage, self._root_path, add_default_op=True)
        if not self._stage.GetPrimAtPath(self._root_path + "/Terrain"):
            pxr_utils.createXform(
                self._stage, self._root_path + "/Terrain", add_default_op=True
            )
        pxr_utils.createXform(self._stage, self._mesh_path, add_default_op=True)

    def renderMesh(
        self,
        points: np.ndarray,
        indices: np.ndarray,
        uvs: np.ndarray,
        colors=None,
        update_topology: bool = False,
    ) -> None:
        """
        Creates or updates a mesh prim with the given points and indices.

        Args:
            points (np.ndarray): array of points to set as the mesh vertices.
            indices (np.ndarray): array of indices to set as the mesh indices.
            uvs (np.ndarray): array of uvs to set as the mesh uvs.
            colors (np.ndarray): array of colors to set as the mesh colors.
            update_topology (bool): whether to update the mesh topology.
        """

        mesh = UsdGeom.Mesh.Get(self._stage, self._mesh_path)
        self._mesh_path = self._og_mesh_path
        if not mesh:
            mesh = UsdGeom.Mesh.Define(self._stage, self._mesh_path)
            UsdGeom.Primvar(mesh.GetDisplayColorAttr()).SetInterpolation("vertex")
            pxr_utils.addDefaultOps(mesh)

            # force topology update on first update
            update_topology = True

        mesh.GetPointsAttr().Set(points)

        if update_topology:
            idxs = np.array(indices).reshape(-1, 3)
            mesh.GetFaceVertexIndicesAttr().Set(idxs)
            mesh.GetFaceVertexCountsAttr().Set([3] * len(idxs))
            UsdGeom.Primvar(mesh.GetDisplayColorAttr()).SetInterpolation("vertex")
            pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
                "st", Sdf.ValueTypeNames.Float2Array
            )
            pv.Set(uvs)
            pv.SetInterpolation("faceVarying")

        if colors:
            mesh.GetDisplayColorAttr().Set(colors)

        pxr_utils.setDefaultOps(mesh, self._mesh_pos, self._mesh_rot, self._mesh_scale)

    def get_height_and_random_orientation(
        self,
        x: np.ndarray,
        y: np.ndarray,
        seed: int = 0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Gets the height and random orientation at the given x and y.

        Args:
            x (np.ndarray): x value.
            y (np.ndarray): y value.

        Returns:
            Tuple[np.ndarray, np.ndarray]: height and random orientation.
        """

        return self._geo_clipmap.get_height_and_random_orientation(x, y, seed=seed)
