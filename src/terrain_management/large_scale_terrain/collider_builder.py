import numpy as np
import dataclasses
import os

from pxr import UsdGeom, Gf

from semantics.schema.editor import PrimSemanticData

from src.terrain_management.large_scale_terrain import pxr_utils


@dataclasses.dataclass
class ColliderBuilderCfg:
    resolution: float
    block_size: int
    collider_path: str
    base_name: str
    collider_mode: str
    semantic_label_name: str


# We will create 9 collider meshes that we will update as the robot moves around.
# x --- x --- x --- x
# |     |     |     |
# x --- x --- x --- x
# |     |robot|     |
# x --- x --- x --- x
# |     |     |     |
# x --- x --- x --- x
# The robot will always be in the center of the 3x3 grid.


class ColliderBuilder:
    def __init__(self, settings: ColliderBuilderCfg):
        self.settings = settings
        self.indices = []

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

    def buildBaseGrid(self):
        """
        Builds the grid of vertices and indices for the terrain mesh.
        Also generates the UVs coordinates for the terrain mesh."""

        vertices = []
        for y in np.linspace(
            0,
            self.settings.block_size,
            num=(self.settings.block_size / self.settings.resolution),
            endpoint=True,
        ):
            for x in np.linspace(
                0,
                self.settings.block_size,
                num=(self.settings.block_size / self.settings.resolution),
                endpoint=True,
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
                    self.indices.append(
                        self.gridIndex(x, y - 1, self.settings.block_size)
                    )
                    self.indices.append(self.gridIndex(x, y, self.settings.block_size))
                    self.indices.append(
                        self.gridIndex(x - 1, y - 1, self.settings.block_size)
                    )
                    # Second triangle.
                    self.indices.append(self.gridIndex(x, y, self.settings.block_size))
                    self.indices.append(
                        self.gridIndex(x - 1, y, self.settings.block_size)
                    )
                    self.indices.append(
                        self.gridIndex(x - 1, y - 1, self.settings.block_size)
                    )

        self.sim_grid = np.zeros(
            self.settings.block_size * self.settings.block_size, dtype=np.float32
        )
        self.sim_verts = np.array(vertices, dtype=np.float32)
        self.indices = np.array(self.indices).reshape(-1, 3)
        self.num_indices = [3] * len(self.indices)

    def label(self, mesh):
        prim_sd = PrimSemanticData(mesh.GetPrim())
        prim_sd.set_semantic_label(self.settings.semantic_label_name)

    def createNewCollider(self, position, map, uid):
        # Need map + 1 to have continuous terrain.
        # The getting the right block shape will be offloaded to the manager.

        # Pray the gods we didn't fuck up.
        self.sim_verts[:, -1] = map.T.flatten()
        mesh = UsdGeom.Mesh.Define(
            self.stage,
            os.path.join(
                self.settings.collider_path, self.settings.base_name, str(uid)
            ),
        )
        mesh.GetPointsAttr().Set(self.sim_verts)
        mesh.GetFaceVertexIndicesAttr().Set(self.indices)
        mesh.GetFaceVertexCountsAttr().Set(self.num_indices)

        pxr_utils.setXformOps(
            mesh, translate=Gf.Vec3d(position[0], position[1], position[1])
        )
        pxr_utils.addCollision(
            self.stage, mesh.GetPath(), mode=self.settings.collider_mode
        )

        self.label(mesh)

    def updateCollider(self, position, map, uid):
        self.sim_verts[:, -1] = map.T.flatten()
        mesh = UsdGeom.Mesh.Get(
            self.stage,
            os.path.join(
                self.settings.collider_path, self.settings.base_name, str(uid)
            ),
        )

        mesh.GetPointsAttr().Set(self.sim_verts)
        pxr_utils.setDefaultOps(mesh, position, (0, 0, 0, 1), (1, 1, 1))
        pxr_utils.removeCollision(self.stage, mesh.GetPath())
        pxr_utils.addCollision(
            self.stage, mesh.GetPath(), mode=self.settings.collider_mode
        )
