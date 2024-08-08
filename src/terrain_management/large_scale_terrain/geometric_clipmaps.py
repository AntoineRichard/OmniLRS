__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import Tuple
import dataclasses
import numpy as np
import warp as wp
import hashlib
import os

from src.terrain_management.large_scale_terrain.geometric_clipmaps_numba import (
    _buildMesh,
)
from src.terrain_management.large_scale_terrain.geometric_clipmaps_warp import (
    _preprocess,
    _bilinear_interpolation,
    _bicubic_interpolation,
    _get_values_wp_2x2,
    _get_values_wp_4x4,
)
from src.terrain_management.large_scale_terrain.utils import ScopedTimer


@dataclasses.dataclass
class GeoClipmapSpecs:
    """
    Args:
        startingLODLevel (int): starting level of detail.
        numMeshLODLevels (int): number of levels of detail.
        meshBaseLODExtentHeightfieldTexels (int): extent of the base level of detail.
        meshBackBonePath (str): path to the mesh backbone.
        source_resolution (float): resolution of the source DEM.
        minimum_target_resolution (float): minimum target resolution.
    """

    startingLODLevel: int = dataclasses.field(default_factory=int)
    numMeshLODLevels: int = dataclasses.field(default_factory=int)
    meshBaseLODExtentHeightfieldTexels: int = dataclasses.field(default_factory=int)
    meshBackBonePath: str = dataclasses.field(default_factory=str)
    source_resolution: float = dataclasses.field(default_factory=float)
    minimum_target_resolution: float = dataclasses.field(default_factory=float)


class GeoClipmap:
    """
    Class to create and update a geometric clipmap.
    This code makes intensive usage of JIT compiled methods using Numba and Warp.
    This is done to accelerate almost all the operations in this class.
    Numba is used to create the mesh backbone, while Warp is used to update the mesh.
    It provides two acceleration modes: "hybrid" and "gpu".
        - The "hybrid" mode uses the GPU to preprocess the data and the CPU to querry points
        from the DEM. The interpolation is then done on the GPU.
        - The "gpu" mode uses the GPU to perform all the operations.
    The "gpu" mode is faster than the "hybrid" mode, but it requires a GPU with a large
    amount of memory. The "hybrid" mode is slower, but it can be used on GPUs with less
    memory as the DEM is stored on the system memory. Ideally, we should work with a compressed
    DEM stored in GPU memory to reduce the memory footprint and accelerate the operations.
    The hybrid mode also creates a lot of overhead due to the data transfer between the CPU
    and the GPU.


    This code is inspired from: https://github.com/morgan3d/misc/tree/master/terrain
    """

    def __init__(
        self,
        specs: GeoClipmapSpecs,
        interpolation_method: str = "bilinear",
        acceleration_mode: str = "hybrid",
    ) -> None:
        """
        Args:
            specs (GeoClipmapSpecs): specifications for the clipmap.
            interpolation_method (str): method to use for interpolation.
            acceleration_mode (str): mode to use for acceleration.
        """

        self.specs = specs
        self.index_count = 0
        self.prev_indices = {}
        self.new_indices = {}
        self.points = []
        self.uvs = []
        self.indices = []
        self.dem = None
        self.dem_shape = None
        self.interpolation_method = interpolation_method
        self.acceleration_mode = acceleration_mode
        self.initMesh()

    def build(self, dem: np.ndarray, dem_shape: Tuple[int, int]) -> None:
        """
        Build the DEM sampler used to update the geometric clipmap.

        Args:
            dem (np.ndarray): DEM data.
            dem_shape (Tuple[int, int]): shape of the DEM.
        """

        self.DEM_sampler = DEMSampler(
            dem,
            dem_shape,
            self.specs,
            self.points,
            interpolation_method=self.interpolation_method,
            acceleration_mode=self.acceleration_mode,
        )

    @staticmethod
    def compute_hash(specs: GeoClipmapSpecs) -> str:
        """
        Compute a hash from the specifications. This hash is used to check if the mesh backbone
        has been generated with the same specifications.

        Args:
            specs (GeoClipmapSpecs): specifications for the clipmap.

        Returns:
            str: hash of the specifications
        """

        return hashlib.sha256(str(specs).encode("utf-8")).hexdigest()

    def buildMesh(self) -> None:
        """
        Build the mesh backbone. This method is called only once to generate the mesh backbone.
        It relies on Numba to accelerate the mesh generation.
        """

        with ScopedTimer("Complete mesh backbone generation"):
            with ScopedTimer("numba mesh backbone generation"):
                self.points, self.uvs, self.indices = _buildMesh(
                    self.specs.startingLODLevel,
                    self.specs.numMeshLODLevels,
                    self.specs.meshBaseLODExtentHeightfieldTexels,
                )
            with ScopedTimer("cast to numpy"):
                self.points = (
                    np.array(self.points) * 2 * self.specs.minimum_target_resolution
                )
                self.uvs = np.array(self.uvs) * 2 * self.specs.minimum_target_resolution
                self.indices = np.array(self.indices)

    def saveMesh(self) -> None:
        """
        Save the mesh backbone to a compressed numpy file.
        """

        np.savez_compressed(
            self.specs.meshBackBonePath,
            points=self.points,
            indices=self.indices,
            uvs=self.uvs,
            specs_hash=self.specs_hash,
        )

    def loadMesh(self) -> None:
        """
        Load the mesh backbone from a compressed numpy file. If the specifications have changed,
        the mesh backbone is rebuilt. Otherwise, the mesh backbone is loaded from the file.
        """

        data = np.load(self.specs.meshBackBonePath)
        if data["specs_hash"] != self.specs_hash:
            self.buildMesh()
            self.saveMesh()
        else:
            self.points = data["points"]
            self.indices = data["indices"]
            self.uvs = data["uvs"]

    def initMesh(self) -> None:
        """
        Initialize the mesh backbone. This method is called at the initialization of the class.
        It builds the mesh backbone if it does not exist, otherwise it loads the mesh backbone.
        """

        self.specs_hash = self.compute_hash(self.specs)
        if os.path.exists(self.specs.meshBackBonePath):
            self.loadMesh()
        else:
            self.buildMesh()
            self.saveMesh()
    
    def updateDEMBuffer(self) -> None:
        """
        Updates the DEM buffer. Note that since the DEM is passed to the DEM sampler as a reference,
        it is not required to feed it through that function.
        """

        self.DEM_sampler.updateDEM()

    def updateElevation(self, coordinates: Tuple[float, float]) -> None:
        """
        Update the elevation of the mesh backbone at the given coordinates.

        Args:
            coordinates (Tuple[float, float]): coordinates to update from (in meters).
        """
        self.DEM_sampler.getElevation(coordinates)


class DEMSampler:
    """
    Class to sample the DEM and update the mesh backbone.
    This class is used to update the mesh backbone with the elevation data from the DEM.
    It provides two acceleration modes: "hybrid" and "gpu".
        - The "hybrid" mode uses the GPU to preprocess the data and the CPU to querry points
        from the DEM. The interpolation is then done on the GPU.
        - The "gpu" mode uses the GPU to perform all the operations.
    The "gpu" mode is faster than the "hybrid" mode, but it requires a GPU with a large
    amount of memory. The "hybrid" mode is slower, but it can be used on GPUs with less
    memory as the DEM is stored on the system memory. Ideally, we should work with a compressed
    DEM stored in GPU memory to reduce the memory footprint and accelerate the operations.
    The interpolation method can be either "bilinear" or "bicubic".
    """

    def __init__(
        self,
        dem,
        dem_size,
        specs,
        points,
        interpolation_method="bilinear",
        acceleration_mode="hybrid",
    ) -> None:
        """
        Args:
            dem (np.ndarray): DEM data.
            dem_size (Tuple[int, int]): shape of the DEM.
            specs (GeoClipmapSpecs): specifications for the clipmap.
            points (np.ndarray): mesh backbone points.
            interpolation_method (str): method to use for interpolation. Can be "bilinear"
                or "bicubic".
            acceleration_mode (str): mode to use for acceleration. Can be "hybrid" or "gpu".
        """

        self.dem = dem  # Reference (read only)
        self.dem_size = dem_size  # Reference (read only)
        self.specs = specs  # Reference (read only)
        self.points = points  # Reference (read only)

        self.interpolation_method = interpolation_method
        self.acceleration_mode = acceleration_mode
        self.initialize_warp_buffers()

    def initialize_warp_buffers(self) -> None:
        """
        Initialize the buffers used by Warp to accelerate the operations.
        """

        if self.acceleration_mode == "hybrid":
            self.initialize_warp_buffers_hybrid_mode()
        elif self.acceleration_mode == "gpu":
            self.initialize_warp_buffers_gpu_mode()
        else:
            raise ValueError("Invalid acceleration mode")
        
    def updateDEM(self) -> None:
        """
        Update the DEM buffer.
        
        This method should only be called in GPU mode. In hybrid mode,
        warp is using a reference to the DEM data, so there is no need to update it.
        """

        if self.acceleration_mode == "gpu":
            with wp.ScopedTimer("update DEM data"):
                self.dem_wp.assign(self.dem.flatten())

    def initialize_warp_buffers_hybrid_mode(self) -> None:
        """
        Initialize the buffers used by Warp to accelerate the operations in the "hybrid" mode.
        """

        self.dem_wp = wp.array(
            self.dem, dtype=float, device="cpu", copy=False,
        )
        self.points_wp = wp.array(self.points[:, :2], dtype=wp.vec2f, device="cuda")
        self.x_wp = wp.zeros((self.points.shape[0]), dtype=float, device="cuda")
        self.y_wp = wp.zeros((self.points.shape[0]), dtype=float, device="cuda")
        self.x_wp_cpu = wp.zeros(
            (self.points.shape[0]), dtype=float, device="cpu", pinned=True
        )
        self.y_wp_cpu = wp.zeros(
            (self.points.shape[0]), dtype=float, device="cpu", pinned=True
        )
        self.x_delta = wp.zeros((self.points.shape[0]), dtype=float, device="cuda")
        self.y_delta = wp.zeros((self.points.shape[0]), dtype=float, device="cuda")
        self.dem_shape_wp = wp.vec2i(self.dem_size[0], self.dem_size[1])
        self.dem_shape_wp_f = wp.vec2f(self.dem_size[0], self.dem_size[1])
        self.coeffs_wp = wp.zeros(self.points.shape[0], dtype=wp.vec4f, device="cuda")
        self.z_cuda = wp.zeros(self.points.shape[0], dtype=float, device="cuda")

        if self.interpolation_method == "bilinear":
            self.q_cpu = wp.zeros(
                (self.points.shape[0]),
                dtype=wp.mat22f,
                device="cpu",
                pinned=True,
            )
            self.q_cuda = wp.zeros(
                (self.points.shape[0]), dtype=wp.mat22f, device="cuda"
            )
        elif self.interpolation_method == "bicubic":
            self.q_cpu = wp.zeros(
                (self.points.shape[0]),
                dtype=wp.mat44f,
                device="cpu",
                pinned=True,
            )
            self.q_cuda = wp.zeros(
                (self.points.shape[0]), dtype=wp.mat44f, device="cuda"
            )
        else:
            raise ValueError("Invalid interpolation method")

    def initialize_warp_buffers_gpu_mode(self) -> None:
        """
        Initialize the buffers used by Warp to accelerate the operations in the "gpu" mode.
        """

        self.dem_wp = wp.array(self.dem.flatten(), dtype=float, device="cuda")
        self.points_wp = wp.array(self.points[:, :2], dtype=wp.vec2f, device="cuda")
        self.x_wp = wp.zeros((self.points.shape[0]), dtype=float, device="cuda")
        self.y_wp = wp.zeros((self.points.shape[0]), dtype=float, device="cuda")
        self.x_delta = wp.zeros((self.points.shape[0]), dtype=float, device="cuda")
        self.y_delta = wp.zeros((self.points.shape[0]), dtype=float, device="cuda")
        self.dem_shape_wp = wp.vec2i(self.dem_size[0], self.dem_size[1])
        self.dem_shape_wp_f = wp.vec2f(self.dem_size[0], self.dem_size[1])
        self.coeffs_wp = wp.zeros(self.points.shape[0], dtype=wp.vec4f, device="cuda")
        self.z_cuda = wp.zeros(self.points.shape[0], dtype=float, device="cuda")

        if self.interpolation_method == "bilinear":
            self.q_cuda = wp.zeros(
                (self.points.shape[0]), dtype=wp.mat22f, device="cuda"
            )
        elif self.interpolation_method == "bicubic":
            self.q_cuda = wp.zeros(
                (self.points.shape[0]), dtype=wp.mat44f, device="cuda"
            )
        else:
            raise ValueError("Invalid interpolation method")

    def getElevation(self, position: np.ndarray) -> None:
        """
        Get the elevation of the mesh backbone at the given position.

        Args:
            position (np.ndarray): position to query the elevation from (in meters).
        """

        if self.acceleration_mode == "hybrid":
            self.getElevationHybrid(position)
        elif self.acceleration_mode == "gpu":
            self.getElevationGPU(position)
        else:
            raise ValueError("Invalid acceleration mode")

    def getElevationHybrid(self, position: np.ndarray) -> None:
        """
        Get the elevation of the mesh backbone at the given position using the "hybrid" mode.

        Args:
            position (np.ndarray): position to query the elevation from (in meters).
        """

        with wp.ScopedTimer("preprocess_Hybrid", active=True):
            position_in_pixel = position * (1.0 / self.specs.source_resolution)
            coords = wp.vec2f(position_in_pixel[0], position_in_pixel[1])
            wp.launch(
                kernel=_preprocess,
                dim=self.points.shape[0],
                inputs=[
                    self.x_wp,
                    self.y_wp,
                    self.points_wp,
                    coords,
                    self.specs.source_resolution,
                    self.dem_shape_wp_f,
                ],
                device="cuda",
            )
            self.x_wp_cpu.assign(self.x_wp)
            self.y_wp_cpu.assign(self.y_wp)
            wp.synchronize()
        if self.interpolation_method == "bilinear":
            self.bilinear_interpolation_hybrid()
        elif self.interpolation_method == "bicubic":
            self.bicubic_interpolation_hybrid()
        else:
            raise ValueError("Invalid interpolation method")

    def bilinear_interpolation_hybrid(self) -> None:
        """
        Perform bilinear interpolation using the "hybrid" mode.
        """

        with wp.ScopedTimer("get_values_wp_Hybrid", active=True):
            wp.launch(
                kernel=_get_values_wp_2x2,
                dim=self.points.shape[0],
                inputs=[
                    self.dem_wp.flatten(),
                    self.dem_shape_wp,
                    self.x_wp_cpu,
                    self.y_wp_cpu,
                    self.q_cpu,
                ],
                device="cpu",
            )
            self.q_cuda.assign(self.q_cpu)

        with wp.ScopedTimer("bilinear_interpolation_Hybrid", active=True):
            wp.launch(
                kernel=_bilinear_interpolation,
                dim=self.points.shape[0],
                inputs=[
                    self.x_wp,
                    self.y_wp,
                    self.q_cuda,
                    self.z_cuda,
                ],
            )
            self.points[:, -1] = self.z_cuda.numpy()

    def bicubic_interpolation_hybrid(self) -> None:
        """
        Perform bicubic interpolation using the "hybrid" mode.
        """

        with wp.ScopedTimer("get_values_wp_4x4_Hybrid", active=True):
            wp.launch(
                kernel=_get_values_wp_4x4,
                dim=self.points.shape[0],
                inputs=[
                    self.dem_wp.flatten(),
                    self.dem_shape_wp,
                    self.x_wp_cpu,
                    self.y_wp_cpu,
                    self.q_cpu,
                ],
                device="cpu",
            )
            self.q_cuda.assign(self.q_cpu)

        with wp.ScopedTimer("bicubic_interpolation_Hybrid", active=True):
            wp.launch(
                kernel=_bicubic_interpolation,
                dim=self.points.shape[0],
                inputs=[
                    self.x_wp,
                    self.y_wp,
                    self.q_cuda,
                    self.coeffs_wp,
                    self.z_cuda,
                ],
            )
            self.points[:, -1] = self.z_cuda.numpy()

    def getElevationGPU(self, position: np.ndarray) -> None:
        """
        Get the elevation of the mesh backbone at the given position using the "gpu" mode.

        Args:
            position (np.ndarray): position to query the elevation from (in meters).
        """

        with wp.ScopedTimer("preprocess_GPU", active=True):
            position_in_pixel = position * (1.0 / self.specs.source_resolution)
            coords = wp.vec2f(position_in_pixel[0], position_in_pixel[1])
            wp.launch(
                kernel=_preprocess,
                dim=self.points.shape[0],
                inputs=[
                    self.x_wp,
                    self.y_wp,
                    self.points_wp,
                    coords,
                    self.specs.source_resolution,
                    self.dem_shape_wp_f,
                ],
                device="cuda",
            )
        if self.interpolation_method == "bilinear":
            self.bilinear_interpolation_GPU()
        elif self.interpolation_method == "bicubic":
            self.bicubic_interpolation_GPU()
        else:
            raise ValueError("Invalid interpolation method")

    def bilinear_interpolation_GPU(self) -> None:
        """
        Perform bilinear interpolation using the "gpu" mode.
        """

        with wp.ScopedTimer("get_values_wp_GPU", active=True):
            wp.launch(
                kernel=_get_values_wp_2x2,
                dim=self.points.shape[0],
                inputs=[
                    self.dem_wp,
                    self.dem_shape_wp,
                    self.x_wp,
                    self.y_wp,
                    self.q_cuda,
                ],
                device="cuda",
            )

        with wp.ScopedTimer("bilinear_interpolation_GPU", active=True):
            wp.launch(
                kernel=_bilinear_interpolation,
                dim=self.points.shape[0],
                inputs=[
                    self.x_wp,
                    self.y_wp,
                    self.q_cuda,
                    self.z_cuda,
                ],
                device="cuda",
            )
            self.points[:, -1] = self.z_cuda.numpy()

    def bicubic_interpolation_GPU(self) -> None:
        """
        Perform bicubic interpolation using the "gpu" mode.
        """

        with wp.ScopedTimer("get_values_wp_4x4_GPU", active=True):
            wp.launch(
                kernel=_get_values_wp_4x4,
                dim=self.points.shape[0],
                inputs=[
                    self.dem_wp,
                    self.dem_shape_wp,
                    self.x_wp,
                    self.y_wp,
                    self.q_cuda,
                ],
                device="cuda",
            )

        with wp.ScopedTimer("bicubic_interpolation_GPU", active=True):
            wp.launch(
                kernel=_bicubic_interpolation,
                dim=self.points.shape[0],
                inputs=[
                    self.x_wp,
                    self.y_wp,
                    self.q_cuda,
                    self.coeffs_wp,
                    self.z_cuda,
                ],
                device="cuda",
            )
            self.points[:, -1] = self.z_cuda.numpy()


if __name__ == "__main__":
    wp.init()
    GCM = GeoClipmap(specs=GeoClipmapSpecs())
    GCM.buildMesh()
    print("num points", GCM.points.shape[0])
    print("num_uvs", GCM.uvs.shape[0])
    print("num_indices", GCM.indices.shape)
    print("min grid point", GCM.points.min(axis=0))
    print("max grid point", GCM.points.max(axis=0))
