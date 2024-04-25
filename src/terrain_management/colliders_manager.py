__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple
import numpy as np
import warnings
import asyncio
import os

from semantics.schema.editor import PrimSemanticData
from pxr import UsdGeom, Sdf
import omni

from src.terrain_management.terrain_generation import GenerateProceduralMoonYard
from src.configurations.procedural_terrain_confs import TerrainManagerConf
from WorldBuilders import pxr_utils
from assets import get_assets_path


class TerrainManager:
    """
    TerrainManager is a class that manages the terrain in the simulation.
    It is responsible for generating the terrain, and updating the terrain.
    It can also use pre-existing terrain data to update the terrain mesh."""

    def __init__(self, cfg: TerrainManagerConf, **kwargs) -> None:
        """
        Args:
            cfg (TerrainManagerConf): the configuration of the terrain manager.
            **kwargs: additional arguments."""

        self._dems_path = os.path.join(get_assets_path(), cfg.dems_path)
        # self._G = GenerateProceduralMoonYard(cfg.moon_yard)

        self._stage = omni.usd.get_context().get_stage()
        self._root_path = cfg.root_path

        self._dems = {}
        self._DEM = None
        self._mask = None

        self._sim_width = int(cfg.sim_width / cfg.resolution)
        self._sim_length = int(cfg.sim_length / cfg.resolution)
        self._grid_size = cfg.resolution
        self._mesh_scale = cfg.mesh_scale

        self._indices = []
        self._sim_uvs = []
        self._sim_verts = []
        self._sim_grid = []

        self._id = 0
        self._meshes_path = self._root_path + "/Terrain/terrain_meshes"

        pxr_utils.createXform(self._stage, self._root_path, add_default_op=True)
        pxr_utils.createXform(
            self._stage, self._root_path + "/Terrain", add_default_op=True
        )
        pxr_utils.createXform(
            self._stage,
            self._root_path + "/Terrain/terrain_meshes",
            add_default_op=True,
        )

        self.buildGrid()
        self.active_ids = []

        # self._DEM = np.zeros((self._sim_length, self._sim_width), dtype=np.float32)
        # self._mask = np.zeros((self._sim_length, self._sim_width), dtype=np.float32)

    async def addBlock(self, position: Tuple[float, float, float], id: str):
        sim_verts = self._sim_verts.copy()
        sim_verts[:, -1] = np.zeros(
            (self._sim_length, self._sim_width), dtype=np.float32
        ).flatten()
        self.renderMesh(id, sim_verts, self._indices, self._sim_uvs, mesh_pos=position)
        self.updateTerrainCollider()

    def build_initial_block(self):
        for i in range(-1, 2, 1):
            for j in range(-1, 2, 1):
                asyncio.run(
                    self.addBlock(
                        (i * 50, j * 50), str(i) + "_" + str(j) + "_" + self._id
                    )
                )
                self.active_ids.append(str(i) + "_" + str(j) + "_" + self._id)
                self._id += 1

    def build_block(self, position):
        x = position[0]
        y = position[1]
        asyncio.run(self.addBlock(position, str(x) + "_" + str(y) + "_" + self._id))
        self.active_ids.append(str(x) + "_" + str(y) + "_" + self._id)
        self._id += 1

    def update_blocks(self, position):
        x = position[0]
        y = position[1]

        xi = int(x / 50)
        yi = int(y / 50)

        # Add new blocks
        for i in range(-1, 2, 1):
            for j in range(-1, 2, 1):
                if (
                    str(xi + i) + "_" + str(yi + j) + "_" + self._id
                    not in self.active_ids
                ):
                    self.build_block(((xi + i) * 50, (yi + j) * 50))

        # Remove old blocks
        for i in range(-2, 3, 1):
            for j in range(-2, 3, 1):
                if str(xi + i) + "_" + str(yi + j) + "_" + self._id in self.active_ids:
                    self.remove_block(((xi + i) * 50, (yi + j) * 50))

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

    def buildGrid(self):
        """
        Builds the grid of vertices and indices for the terrain mesh.
        Also generates the UVs coordinates for the terrain mesh."""

        vertices = []
        uvs = []
        for y in range(self._sim_length):
            for x in range(self._sim_width):
                pos = (
                    float(x) * self._grid_size,
                    float(y) * self._grid_size,
                    float(0),
                )

                # Generates vertices indices for the terrain mesh.
                vertices.append(pos)
                if x > 0 and y > 0:  # Two triangles per pixel.
                    # First triangle.
                    self._indices.append(self.gridIndex(x, y - 1, self._sim_width))
                    self._indices.append(self.gridIndex(x, y, self._sim_width))
                    self._indices.append(self.gridIndex(x - 1, y - 1, self._sim_width))
                    # Second triangle.
                    self._indices.append(self.gridIndex(x, y, self._sim_width))
                    self._indices.append(self.gridIndex(x - 1, y, self._sim_width))
                    self._indices.append(self.gridIndex(x - 1, y - 1, self._sim_width))

                # Generates UVs coordinates for the terrain mesh.
                if x > 0 and y > 0:  # Two triangles per pixel.
                    # First triangle.
                    uvs.append((x, y - 1))
                    uvs.append((x, y))
                    uvs.append((x - 1, y - 1))
                    # Second triangle.
                    uvs.append((x, y))
                    uvs.append((x - 1, y))
                    uvs.append((x - 1, y - 1))

        self._sim_grid = np.zeros(self._sim_width * self._sim_length, dtype=np.float32)
        self._sim_verts = np.array(vertices, dtype=np.float32)
        self._sim_uvs = np.array(uvs, dtype=np.float32)
        self._sim_uvs = self._sim_uvs * self._grid_size

    def renderMesh(
        self,
        id: str,
        points: np.ndarray,
        indices: np.ndarray,
        uvs: np.ndarray,
        colors=None,
        update_topology: bool = False,
        mesh_pos: Tuple[float, float, float] = (0, 0, 0),
        mesh_rot: Tuple[float, float, float, float] = (0, 0, 0, 1),
    ) -> str:
        """
        Creates or updates a mesh prim with the given points and indices.

        Args:
            points (np.ndarray): array of points to set as the mesh vertices.
            indices (np.ndarray): array of indices to set as the mesh indices.
            uvs (np.ndarray): array of uvs to set as the mesh uvs.
            colors (np.ndarray): array of colors to set as the mesh colors.
            update_topology (bool): whether to update the mesh topology."""

        mesh_path = self._meshes_path + "/block_" + id
        mesh = UsdGeom.Mesh.Get(self._stage, mesh_path)
        if not mesh:
            mesh = UsdGeom.Mesh.Define(self._stage, mesh_path)
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

        self._id += 1
        pxr_utils.setDefaultOps(mesh, mesh_pos, mesh_rot, self._mesh_scale)
        return mesh_path

    def updateTerrainCollider(self):
        """
        Updates the terrain collider. This function should be called after the terrain mesh is updated.
        """

        # pxr_utils.removeCollision(self._stage, self._mesh_path)
        pxr_utils.addCollision(self._stage, self._mesh_path)

    def update(self) -> None:
        """
        Updates the terrain mesh. This function should be called after the DEM and mask are updated.
        """

        pxr_utils.deletePrim(self._stage, self._mesh_path)
        self._sim_verts[:, -1] = np.flip(self._DEM, 0).flatten()
        self.renderMesh(self._sim_verts, self._indices, self._sim_uvs)
        self.updateTerrainCollider()

    def randomizeTerrain(self) -> None:
        """
        Randomizes the terrain."""

        self._DEM, self._mask, self._craters_data = self._G.randomize()
        self.update()

    def autoLabel(self):
        prim_sd = PrimSemanticData(self._stage.GetPrimAtPath(self._mesh_path))
        prim_sd.add_entry("class", "ground")

    def getDEM(self):
        """
        Returns the DEM of the terrain.

        Returns:
            np.ndarray: the DEM of the terrain."""

        return self._DEM

    def getMask(self):
        """
        Returns the mask of the terrain.

        Returns:
            np.ndarray: the mask of the terrain."""

        return self._mask


if __name__ == "__main__":
    import omni
    from omni.isaac.kit import SimulationApp

    simulation_app = SimulationApp({"headless": False})
    from omni.isaac.core import World

    world = World(stage_units_in_meters=1.0)
    T = TerrainManager()

    j = 0
    while True:
        T.randomizeTerrain()
        T.loadTerrainId(j)
        j += 1
        for i in range(20):
            world.step(render=True)
