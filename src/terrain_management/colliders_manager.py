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
import ctypes
import asyncio
import math
import os

from semantics.schema.editor import PrimSemanticData
from pxr import UsdGeom, Sdf
import omni

from src.configurations.procedural_terrain_confs import CollidersManagerConf
from WorldBuilders import pxr_utils
from assets import get_assets_path

hashu=lambda word: ctypes.c_uint64(hash(word)).value.to_bytes(8,"big").hex()

class CollidersManager:
    """
    TerrainManager is a class that manages the terrain in the simulation.
    It is responsible for generating the terrain, and updating the terrain.
    It can also use pre-existing terrain data to update the terrain mesh.
    """

    def __init__(self, cfg: CollidersManagerConf, **kwargs) -> None:
        """
        Args:
            cfg (TerrainManagerConf): the configuration of the terrain manager.
            **kwargs: additional arguments.
        """

        self._stage = omni.usd.get_context().get_stage()
        self._root_path = cfg.root_path

        # dynamic DEM data
        self.DEM = None
        self.DEM_offset = (0, 0)

        # Block mesh parameters
        self._sim_width = int(cfg.sim_width / cfg.resolution) + 1
        self._sim_length = int(cfg.sim_length / cfg.resolution) + 1
        self._grid_size = cfg.resolution

        # Block management parameters
        self.build_radius = cfg.build_radius
        self.remove_radius = cfg.remove_radius
        self.max_cache_size = cfg.max_cache_size
        self._meshes_path = self._root_path + "/Terrain/colliders_meshes"

        self.on_setup()
        self.on_reset()


    def on_setup(self) -> None:
        """
        Sets up the collider manager.
        Run once at the beginning of the simulation.
        """

        pxr_utils.createXform(self._stage, self._root_path, add_default_op=True)
        pxr_utils.createXform(
            self._stage, self._root_path + "/Terrain", add_default_op=True
        )
        pxr_utils.createXform(
            self._stage,
            self._root_path + "/Terrain/colliders_meshes",
            add_default_op=True,
        )

        # Terrain mesh variables
        self._indices = []
        self._sim_uvs = []
        self._sim_verts = []
        self._sim_grid = []

        self.buildGrid()

    def on_reset(self) -> None:
        """
        Resets the collider manager.
        Run every time the simulation is reset.
        """

        # Block management variables
        self.active_ids = []
        self.active_terrains = []
        self.last_build = ()
        self.last_remove = ()

    def get_DEM_data(self, position: Tuple[float, float, float]) -> np.ndarray:
        """
        Returns the DEM data for a whole block given the position of the block.

        #TODO:
        As of now the X and Y axes are swapped. This could mean that we do not read the data properly.
        Inverting Y is typical, but the XY swap should be looked into.

        Args:
            position (Tuple[float, float, float]): the position of the block in the simulation.

        Returns:
            np.ndarray: the DEM data for the block.
        """

        if self.DEM is None:
            return np.zeros((self._sim_length, self._sim_width), dtype=np.float32)

        x = int(position[0] * 1 / self._grid_size)
        y = int(position[1] * 1 / self._grid_size)
        x = x + self.DEM_offset[0]
        y = y + self.DEM_offset[1]

        return self.DEM[y:y+self._sim_length, x:x+self._sim_width] # XY FLIP HERE


    def build_block(self, position: Tuple[float, float, float], id: str) -> None:
        """
        Builds a block at the given position.
        
        Args:
            position (Tuple[float, float, float]): the position of the block in the simulation.
            id (str): the id of the block.
        """

        x = position[0]
        y = position[1]
        sim_verts = self._sim_verts.copy()
        sim_verts[:, -1] = self.get_DEM_data(position).flatten()
        mesh_path = self.renderMesh(id, sim_verts, self._indices, self._sim_uvs, mesh_pos=position)
        self.updateTerrainCollider(mesh_path)
        self.active_ids.append(id)

    def remove_block(self, id: str) -> None:
        """
        Removes a block from the terrain.

        Args:
            id (str): the id of the block to remove.
        """

        to_remove = None
        for terrain in self.active_terrains:
            if terrain.endswith(id):
                to_remove = terrain
                break

        if to_remove is not None:
            pxr_utils.deletePrim(self._stage, terrain)
            self.active_terrains.remove(terrain)

    def update_blocks(self, position: Tuple[float, float, float]) -> None:
        """
        Updates the blocks in the terrain.
        
        Args:
            position (Tuple[float, float, float]): the position of the agent in the simulation.
        """

        # Get the position in the plane
        x = position[0]
        y = position[1]

        # Get the block indices
        # We use two thrsesholds to create a dead zone between the construction and the destruction
        # of the blocks. This is to avoid flickering between the two states.
        xmax_build = x + self.build_radius
        ymax_build = y + self.build_radius
        xmin_build = x - self.build_radius
        ymin_build = y - self.build_radius

        xmax_remove = x + self.remove_radius
        ymax_remove = y + self.remove_radius
        xmin_remove = x - self.remove_radius
        ymin_remove = y - self.remove_radius

        xi_max_build = math.floor(xmax_build / 50.)
        yi_max_build = math.floor(ymax_build / 50.)
        xi_min_build = math.floor(xmin_build / 50.)
        yi_min_build = math.floor(ymin_build / 50.)

        xi_max_remove = math.floor(xmax_remove / 50.)
        yi_max_remove = math.floor(ymax_remove / 50.)
        xi_min_remove = math.floor(xmin_remove / 50.)
        yi_min_remove = math.floor(ymin_remove / 50.)

        # Check if there are duplicates
        if xi_max_build != xi_min_build:
            x_build = [xi_min_build, xi_max_build]
        else:
            x_build = [xi_min_build]
        
        if yi_max_build != yi_min_build:
            y_build = [yi_min_build, yi_max_build]
        else:
            y_build = [yi_min_build]

        if xi_max_remove != xi_min_remove:
            x_remove = [xi_min_remove, xi_max_remove]
        else:
            x_remove = [xi_min_remove]

        if yi_max_remove != yi_min_remove:
            y_remove = [yi_min_remove, yi_max_remove]
        else:
            y_remove = [yi_min_remove]

        # If the build blocks are different from the last build blocks then build the new blocks
        if self.last_build != (x_build, y_build):
            self.last_build = (x_build, y_build)
            for i in x_build:
                for j in y_build:
                    id = hashu(str(i) + "_" + str(j))
                    if (id not in self.active_ids):
                        self.build_block(((i) * 50, (j) * 50, 0), id)

        # If the remove blocks are different from the last remove blocks then remove the old blocks
        if self.last_remove != (x_remove, y_remove):
            id_to_keep = []
            for i in x_remove:
                for j in y_remove:
                    id = hashu(str(i) + "_" + str(j))
                    id_to_keep.append(id)

            for id in self.active_ids:
                if id not in id_to_keep:
                    self.remove_block(id)
                    self.active_ids.remove(id)
    
    @staticmethod
    def gridIndex(x: int, y: int, stride: int) -> int:
        """
        Returns the index of the grid point at (x, y) in a grid of width stride.

        Args:
            x (int): x coordinate of the grid point.
            y (int): y coordinate of the grid point.
            stride (int): width of the grid.

        Returns:
            int: index of the grid point at (x, y).
        """

        return y * stride + x

    def buildGrid(self):
        """
        Builds the grid of vertices and indices for the terrain mesh.
        Also generates the UVs coordinates for the terrain mesh.
        """

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

        self._sim_grid = np.zeros((self._sim_width) * (self._sim_length), dtype=np.float32)
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
        mesh_scale: Tuple[float, float, float] = (1, 1, 1),
    ) -> str:
        """
        Creates or updates a mesh prim with the given points and indices.

        Args:
            points (np.ndarray): array of points to set as the mesh vertices.
            indices (np.ndarray): array of indices to set as the mesh indices.
            uvs (np.ndarray): array of uvs to set as the mesh uvs.
            colors (np.ndarray): array of colors to set as the mesh colors.
            update_topology (bool): whether to update the mesh topology.
        """

        mesh_path = self._meshes_path + "/block_" + id
        self.active_terrains.append(mesh_path)
        mesh = UsdGeom.Mesh.Get(self._stage, mesh_path)
        if not mesh:
            mesh = UsdGeom.Mesh.Define(self._stage, mesh_path)
            UsdGeom.Primvar(mesh.GetDisplayColorAttr()).SetInterpolation("vertex")
            pxr_utils.addDefaultOps(mesh)

            # force topology update on first update
            update_topology = True

        mesh.GetPointsAttr().Set(points)

        if update_topology:
            idxs = np.array(indices).reshape(-1, 3) # This could be cached
            mesh.GetFaceVertexIndicesAttr().Set(idxs)
            mesh.GetFaceVertexCountsAttr().Set([3] * len(idxs))
            UsdGeom.Primvar(mesh.GetDisplayColorAttr()).SetInterpolation("vertex")
            pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
                "st", Sdf.ValueTypeNames.Float2Array
            )
            pv.Set(uvs)
            pv.SetInterpolation("faceVarying")

        # Make the colliders red if they are visible
        colors = np.zeros_like(points) # This could be cached
        colors[:, 0] = 1.0 # This could be cached
        mesh.GetDisplayColorAttr().Set(colors)
        pxr_utils.setDefaultOps(mesh, mesh_pos, mesh_rot, mesh_scale)

        # Make the colliders invisible
        #self._stage.GetPrimAtPath(mesh_path).GetAttribute("visibility").Set("invisible")
        return mesh_path

    def updateTerrainCollider(self, mesh_path: str) -> None:
        """
        Updates the terrain collider. This function should be called after the terrain mesh is updated.
        """

        pxr_utils.addCollision(self._stage, mesh_path)

    def update_DEM_data(self, DEM: np.ndarray, offset: Tuple[float, float]) -> None:
        """
        Updates the DEM data of the terrain. 
        The DEM data must be provided at the PPM used to genenerate the colliders.
        This is very important or the code will fail to rebuild the colliders properly.
        This class does not handle the conversion of the DEM data to the correct resolution.
        This should be done at a higher level using the warp kernels to ensure fast and memory efficient conversion.
        Duplicating the warp kernels here would be a waste of GPU memory.

        Ideally, we'd like to pass a reference to the DEM data to avoid copying it.
        The DEM is read only and should not be modified by the colliders manager.
        This function should only be called once at the beginning of the simulation.
        (This should really be done in C++ to increase performance.)

        Args:
            DEM (np.ndarray): the new DEM data.
            offset (Tuple[float, float]): the offset of the DEM data.
        """

        self.DEM = DEM
        self.DEM_offset = offset

    def update_DEM_offset(self, offset: Tuple[float, float]) -> None:
        """
        Updates the offset of the DEM data.

        Args:
            offset (Tuple[float, float]): the new offset of the DEM data.
        """

        self.DEM_offset = offset


if __name__ == "__main__":
    from src.configurations.procedural_terrain_confs import TerrainManagerConf
    from src.terrain_management.colliders_manager import CollidersManager
    from omni.isaac.core import World
    from pxr import UsdLux, UsdGeom

    CMC = CollidersManagerConf(
        root_path="/Main",
        sim_length=50.0,
        sim_width=50.0,
        resolution=0.1,
        build_radius=1.5,
        remove_radius=2.5,
        max_cache_size=6,
    )

    CM = CollidersManager(cfg=CMC)

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()    

    #TM.build_initial_block()

    UsdLux.DistantLight.Define(stage, "/sun")
    UsdGeom.Sphere.Define(stage, "/sphere")
    sphere = stage.GetPrimAtPath("/sphere")
    pxr_utils.addDefaultOps(sphere)
    i = 0
    r = 60
    w = 0.0001
    while True:
        world.step(render=True)
        pxr_utils.setDefaultOps(sphere, (math.cos(i*w)*r, math.sin(i*w)*r, 1.), (0.,0.,0.,1.), (1.,1.,1.))
        i += 1
        CM.update_blocks((math.cos(i*w)*r, math.sin(i*w)*r, 1.))
