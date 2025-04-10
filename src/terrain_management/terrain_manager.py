__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple
import numpy as np
import warnings
import os

from semantics.schema.editor import PrimSemanticData
from pxr import UsdGeom, Sdf
import omni
import warp as wp

from src.terrain_management.terrain_generation import GenerateProceduralMoonYard
from src.configurations.procedural_terrain_confs import TerrainManagerConf
from WorldBuilders import pxr_utils
from assets import get_assets_path


class TerrainManager:
    """
    TerrainManager is a class that manages the terrain in the simulation.
    It is responsible for generating the terrain, and updating the terrain.
    It can also use pre-existing terrain data to update the terrain mesh.
    """

    def __init__(self, cfg: TerrainManagerConf, **kwargs) -> None:
        """
        Args:
            cfg (TerrainManagerConf): the configuration of the terrain manager.
            **kwargs: additional arguments.
        """

        self._dems_path = os.path.join(get_assets_path(), cfg.dems_path)
        self._G = GenerateProceduralMoonYard(cfg.moon_yard)

        self._stage = omni.usd.get_context().get_stage()
        self._texture_path = cfg.texture_path
        self._root_path = cfg.root_path
        self._augmentation = cfg.augmentation

        self._dems = {}
        self._DEM = None
        self._mask = None
        self._background_mask = None
        self._crater_mask = None
        self._crater_ejecta_mask = None
        self._material = None

        self._sim_width = int(cfg.sim_width / cfg.resolution)
        self._sim_length = int(cfg.sim_length / cfg.resolution)
        self._grid_size = cfg.resolution
        self._mesh_pos = cfg.mesh_position
        self._mesh_rot = cfg.mesh_orientation
        self._mesh_scale = cfg.mesh_scale

        self._indices = []
        self._sim_uvs = []
        self._sim_verts = []
        self._sim_grid = []

        self._id = 0
        self._og_mesh_path = self._root_path + "/Terrain/terrain_mesh"
        self._mesh_path = self._root_path + "/Terrain/terrain_mesh"

        pxr_utils.createXform(self._stage, self._root_path, add_default_op=True)
        pxr_utils.createXform(self._stage, self._root_path + "/Terrain", add_default_op=True)
        pxr_utils.createXform(self._stage, self._root_path + "/Terrain/terrain_mesh", add_default_op=True)

        self.buildGrid()
        self.fetchPreGeneratedDEMs()

    def fetchPreGeneratedDEMs(self) -> None:
        """
        fetches the path to the pre-generated DEMs and masks from the given path.
        To load the DEMs and masks, the paths must contain the following files:
            - Folder1:
                - DEM.npy
                - mask.npy
            - Folder2:
                - DEM.npy
                - mask.npy
            - ...

        Args:
            dems_path (str): path to the folders containing the DEMs and masks.

        Raises:
            AssertionError: if dems_path is not a directory.
            AssertionError: if dems_path contains files other than folders.
            AssertionError: if a folder does not contain DEM.npy.
        """

        assert os.path.isdir(self._dems_path), "dems_path must be a directory."

        for folder in os.listdir(self._dems_path):
            assert os.path.isdir(os.path.join(self._dems_path, folder)), "dems_path must contain only folders."
            dem_path = os.path.join(self._dems_path, folder, "dem.npy")
            mask_path = os.path.join(self._dems_path, folder, "mask.npy")
            assert os.path.isfile(dem_path), "dem.npy not found in folder."
            if not os.path.isfile(mask_path):
                warnings.warn("Mask not found.")
                mask_path = None
            self._dems[folder] = [dem_path, mask_path]

    def loadDEMAndMask(self, name: str) -> None:
        """
        Loads the DEM and mask from the given paths.

        Args:
            name (str): the name matching the dictionaty entry.

        Raises:
            AssertionError: if the name is not found in the dictionary.
            AssertionError: if the DEM.npy is not found in the folder.
        """

        assert name in self._dems, "name not found in dictionary."
        dem_path = self._dems[name][0]
        mask_path = self._dems[name][1]
        assert os.path.isfile(dem_path), "DEM.npy not found in folder."
        DEM = np.load(dem_path)
        if mask_path is None:
            mask = np.ones_like(DEM, dtype=bool)
        else:
            mask = np.load(mask_path)

        self._DEM = np.zeros((self._sim_length, self._sim_width), dtype=np.float32)
        self._crater_mask = np.zeros((self._sim_length, self._sim_width), dtype=np.float32)
        self._DEM[: DEM.shape[0], : DEM.shape[1]] = DEM[: self._sim_length, : self._sim_width]
        self._crater_mask[: mask.shape[0], : mask.shape[1]] = mask[: self._sim_length, : self._sim_width]

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

        self._sim_grid = np.zeros(self._sim_width * self._sim_length, dtype=np.float32)
        self._sim_verts = np.array(vertices, dtype=np.float32)
        self._sim_uvs = np.array(uvs, dtype=np.float32)
        self._sim_uvs = self._sim_uvs * self._grid_size

    def renderMesh(
        self,
        points: np.ndarray,
        indices: np.ndarray,
        uvs: np.ndarray,
        colors=None,
        update_topology: bool = False,
        update_default_op: bool = False,
    ) -> None:
        """
        Creates or updates a mesh prim with the given points and indices.

        Args:
            points (np.ndarray): array of points to set as the mesh vertices.
            indices (np.ndarray): array of indices to set as the mesh indices.
            uvs (np.ndarray): array of uvs to set as the mesh uvs.
            colors (np.ndarray): array of colors to set as the mesh colors.
            update_topology (bool): whether to update the mesh topology.
            update_default_op (bool): whether to update the default ops of the mesh.
        """

        mesh = UsdGeom.Mesh.Get(self._stage, self._mesh_path)
        if update_default_op:
            self._mesh_path = self._og_mesh_path + "_" + str(self._id)
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
            pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.Float2Array)
            pv.Set(uvs)
            pv.SetInterpolation("faceVarying")

        if colors:
            mesh.GetDisplayColorAttr().Set(colors)

        if update_default_op:
            self._id += 1
            pxr_utils.setDefaultOps(mesh, self._mesh_pos, self._mesh_rot, self._mesh_scale)

    def updateTerrainCollider(self):
        """
        Updates the terrain collider. This function should be called after the terrain mesh is updated.
        """

        pxr_utils.removeCollision(self._stage, self._mesh_path)
        pxr_utils.addCollision(self._stage, self._mesh_path, mode="meshSimplification")

    def update(self, update_collider=False) -> None:
        """
        Updates the terrain mesh. This function should be called after the DEM and mask are updated.
        """

        if update_collider:
            pxr_utils.deletePrim(self._stage, self._mesh_path)
            self._sim_verts[:, -1] = np.flip(self._DEM, 0).flatten()
            with wp.ScopedTimer("mesh update"):
                self.renderMesh(self._sim_verts, self._indices, self._sim_uvs, update_default_op=True)
            self.updateTerrainCollider()
            self.autoLabel()
            pxr_utils.applyMaterialFromPath(self._stage, self._mesh_path, self._texture_path)
        else:
            self._sim_verts[:, -1] = np.flip(self._DEM, 0).flatten()
            with wp.ScopedTimer("mesh update"):
                self.renderMesh(self._sim_verts, self._indices, self._sim_uvs)

    def randomizeTerrain(self) -> None:
        """
        Randomizes the terrain (update mesh, collider, semantic).
        """

        self._DEM, self._background_mask, self._crater_mask, self._crater_ejecta_mask, self._craters_data = self._G.randomize()
        self.update(update_collider=True)

    def deformTerrain(
        self, world_positions: np.ndarray, world_orientations: np.ndarray, contact_forces: np.ndarray
    ) -> None:
        """
        Deforms the terrain based on the given body transforms.

        Args:
            world_positions (np.ndarray): the world positions of the bodies.
            world_orientations (np.ndarray): the world orientations of the bodies.
            contact_forces (np.ndarray): the contact forces of the bodies.
        """

        self._DEM, self._background_mask, self._crater_mask, self._crater_ejecta_mask = self._G.deform(world_positions, world_orientations, contact_forces)
        self.update(update_collider=False)

    def loadTerrainByName(self, name: str) -> None:
        """
        Loads the terrain from the given name.

        Args:
            name (str): the name matching the dictionaty entry.
        """

        self.loadDEMAndMask(name)
        if self._augmentation:
            self._DEM, self._background_mask, self._crater_mask, self._crater_ejecta_mask, self._craters_data = self._G.augment(self._DEM, self._background_mask,
                                                                                                                                self._crater_mask, self._crater_ejecta_mask)
        else:
            self._G.register_terrain(self._DEM, self._crater_mask)
        self.update(update_collider=True)

    def loadTerrainId(self, idx: int) -> None:
        """
        Loads the terrain from the given name.

        Args:
            name (str): the name matching the dictionaty entry.

        Raises:
            AssertionError: if idx is out of bounds.
        """

        assert idx < len(self._dems), "idx out of bounds."
        name = list(self._dems.keys())[idx]
        self.loadTerrainByName(name)

    def autoLabel(self):
        prim_sd = PrimSemanticData(self._stage.GetPrimAtPath(self._mesh_path))
        prim_sd.add_entry("class", "ground")

    def getDEM(self):
        """
        Returns the DEM of the terrain.

        Returns:
            np.ndarray: the DEM of the terrain.
        """

        return self._DEM

    def getCraterMask(self):
        """
        Returns the crater mask of the terrain.

        Returns:
            np.ndarray: the crater mask of the terrain.
        """

        return self._crater_mask
    
    def getBackgroundMask(self):
        """
        Returns the uniform background mask of the terrain.

        Returns:
            np.ndarray: the uniform background mask of the terrain.
        """

        return self._background_mask
    
    def getCraterEjectaMask(self):
        """
        Returns the crater ejecta mask of the terrain.

        Returns:
            np.ndarray: the crater ejecta mask of the terrain.
        """

        return self._crater_ejecta_mask


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
