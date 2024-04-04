__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from src.terrain_management.geo_clipmaps_static import GeoClipmapSpecs, GeoClipmap
from WorldBuilders import pxr_utils
from pxr import UsdGeom, Sdf
import numpy as np
import warp as wp
import omni


class GeoClipmapManagerConf:
    root_path: str = "/World"
    geo_clipmap_specs: GeoClipmapSpecs = GeoClipmapSpecs()
    mesh_position: np.ndarray = np.array([0, 0, 0])
    mesh_orientation: np.ndarray = np.array([0, 0, 0, 1])
    mesh_scale: np.ndarray = np.array([1, 1, 1])


class GeoClipmapManager:
    def __init__(self, cfg: GeoClipmapManagerConf):
        self._stage = omni.usd.get_context().get_stage()
        self._geo_clipmap = GeoClipmap(cfg.geo_clipmap_specs)
        self._root_path = cfg.root_path

        self._mesh_pos = cfg.mesh_position
        self._mesh_rot = cfg.mesh_orientation
        self._mesh_scale = cfg.mesh_scale

        self._og_mesh_path = self._root_path + "/Terrain/terrain_mesh"
        self._mesh_path = self._root_path + "/Terrain/terrain_mesh"

        self.createXforms()
        self.update_topology = True

    def updateGeoClipmap(self, position: np.ndarray) -> None:
        self._geo_clipmap.getElevation(position)
        with wp.ScopedTimer("mesh update"):
            self.renderMesh(
                self._geo_clipmap.points,
                self._geo_clipmap.indices,
                self._geo_clipmap.uvs,
                update_topology=self.update_topology,
            )
        self.update_topology = False

    def createXforms(self):
        pxr_utils.createXform(self._stage, self._root_path, add_default_op=True)
        pxr_utils.createXform(
            self._stage, self._root_path + "/Terrain", add_default_op=True
        )
        pxr_utils.createXform(
            self._stage, self._root_path + "/Terrain/terrain_mesh", add_default_op=True
        )

    @staticmethod
    def gridIndex(x: int, y: int, stride: int) -> int:
        """
        Returns the index of the grid point at (x, y) in a grid of width stride.

        Args:
            x (int): x coordinate of the grid point.
            y (int): y coordinate of the grid point.
            stride (int): width of the grid.

        Returns:
            int: index of the grid point at (x, y)."""

        return y * stride + x

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
            update_topology (bool): whether to update the mesh topology."""

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
