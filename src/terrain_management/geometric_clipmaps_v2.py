__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from copy import copy
import warp as wp
import dataclasses
import numpy as np
import hashlib
import os


@wp.kernel
def preprocess(
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    points: wp.array(dtype=wp.vec2f),
    coord: wp.vec2f,
    mpp: float,
    dem_shape: wp.vec2f,
):
    tid = wp.tid()
    x[tid] = points[tid][0] / mpp + coord[0]
    y[tid] = points[tid][1] / mpp + coord[1]
    x[tid] = wp.atomic_min(x, tid, dem_shape[0] - 1.0)
    y[tid] = wp.atomic_min(y, tid, dem_shape[1] - 1.0)
    x[tid] = wp.atomic_max(x, tid, 0.0)
    y[tid] = wp.atomic_max(y, tid, 0.0)


@wp.kernel
def get_values_wp(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: wp.array(dtype=int),
    y: wp.array(dtype=int),
    out: wp.array(dtype=wp.vec4f),
):
    x_max = dem_shape[0]
    y_max = dem_shape[1]
    tid = wp.tid()
    out[tid][0] = dem[y_max * x[tid] + y[tid]]
    out[tid][1] = dem[y_max * wp.min(x[tid] + 1, x_max) + y[tid]]
    out[tid][2] = dem[y_max * wp.min(x[tid] + 2, x_max) + y[tid]]
    out[tid][3] = dem[y_max * wp.min(x[tid] + 3, x_max) + y[tid]]


@wp.func
def get_4x4_mat(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: float,
    y: float,
    out: wp.mat44f,
) -> wp.mat44f:
    x0 = wp.max(int(x) - 1, 0)
    y0 = wp.max(int(y) - 1, 0)
    x1 = wp.min(x0 + 1, dem_shape[0] - 1) * dem_shape[1]
    x2 = wp.min(x0 + 2, dem_shape[0] - 1) * dem_shape[1]
    x3 = wp.min(x0 + 3, dem_shape[0], -1) * dem_shape[1]
    x0 = x0 * dem_shape[1]
    y1 = wp.min(y0 + 1, dem_shape[1], -1)
    y2 = wp.min(y0 + 2, dem_shape[1], -1)
    y3 = wp.min(y0 + 3, dem_shape[1], -1)
    out[0, 0] = dem[x0 + y0]
    out[1, 0] = dem[x1 + y0]
    out[2, 0] = dem[x2 + y0]
    out[3, 0] = dem[x3 + y0]
    out[0, 1] = dem[x0 + y1]
    out[1, 1] = dem[x1 + y1]
    out[2, 1] = dem[x2 + y1]
    out[3, 1] = dem[x3 + y1]
    out[0, 2] = dem[x0 + y2]
    out[1, 2] = dem[x1 + y2]
    out[2, 2] = dem[x2 + y2]
    out[3, 2] = dem[x3 + y2]
    out[0, 3] = dem[x0 + y3]
    out[1, 3] = dem[x1 + y3]
    out[2, 3] = dem[x2 + y3]
    out[3, 3] = dem[x3 + y3]
    return out


@wp.kernel
def get_values_wp_4x4(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    out: wp.array(dtype=wp.mat44f),
):
    tid = wp.tid()
    out[tid] = get_4x4_mat(dem, dem_shape, x[tid], y[tid], out[tid])


@wp.func
def get_2x2_mat(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: float,
    y: float,
    out: wp.mat22f,
) -> wp.mat22f:
    x0 = int(x)
    y0 = int(y)
    x1 = wp.min(x0 + 1, dem_shape[0] - 1) * dem_shape[1]
    x0 = x0 * dem_shape[1]
    y1 = wp.min(y0 + 1, dem_shape[1] - 1)
    out[0, 0] = dem[x0 + y0]
    out[1, 0] = dem[x1 + y0]
    out[0, 1] = dem[x0 + y1]
    out[1, 1] = dem[x1 + y1]
    return out


@wp.kernel
def get_values_wp_2x2(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    out: wp.array(dtype=wp.mat22f),
):
    tid = wp.tid()
    out[tid] = get_2x2_mat(dem, dem_shape, x[tid], y[tid], out[tid])


@wp.func
def _bilinear_interpolator(
    x: float,
    y: float,
    q: wp.mat22f,
):
    x2 = x - wp.trunc(x)
    y2 = y - wp.trunc(y)
    return (
        (1.0 - x2) * (1.0 - y2) * q[0, 0]
        + x2 * (1.0 - y2) * q[1, 0]
        + (1.0 - x2) * y2 * q[0, 1]
        + x2 * y2 * q[1, 1]
    )


@wp.kernel
def _bilinear_interpolation(
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    q: wp.array(dtype=wp.mat22f),
    out: wp.array(dtype=float),
):
    tid = wp.tid()
    out[tid] = _bilinear_interpolator(x[tid], y[tid], q[tid])


@wp.func
def _cubic_interpolator(
    x: float,
    coeffs: wp.vec4f,
):
    x2 = x - wp.trunc(x)
    a = -0.75
    coeffs[0] = a * (x2 * (1.0 - x2 * (2.0 - x2)))
    coeffs[1] = a * (-2.0 + x2 * x2 * (5.0 - 3.0 * x2))
    coeffs[2] = a * (x2 * (-1.0 + x2 * (-4.0 + 3.0 * x2)))
    coeffs[3] = a * (x2 * x2 * (1.0 - x2))
    return coeffs


@wp.kernel
def _bicubic_interpolation(
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    q: wp.array(dtype=wp.mat44f),
    coeffs: wp.array(dtype=wp.vec4f),
    out: wp.array(dtype=float),
):
    tid = wp.tid()
    coeffs[tid] = _cubic_interpolator(x[tid], coeffs[tid])
    a0 = (
        q[tid][0, 0] * coeffs[tid][0]
        + q[tid][1, 0] * coeffs[tid][1]
        + q[tid][2, 0] * coeffs[tid][2]
        + q[tid][3, 0] * coeffs[tid][3]
    )
    a1 = (
        q[tid][0, 1] * coeffs[tid][0]
        + q[tid][1, 1] * coeffs[tid][1]
        + q[tid][2, 1] * coeffs[tid][2]
        + q[tid][3, 1] * coeffs[tid][3]
    )
    a2 = (
        q[tid][0, 2] * coeffs[tid][0]
        + q[tid][1, 2] * coeffs[tid][1]
        + q[tid][2, 2] * coeffs[tid][2]
        + q[tid][3, 2] * coeffs[tid][3]
    )
    a3 = (
        q[tid][0, 3] * coeffs[tid][0]
        + q[tid][1, 3] * coeffs[tid][1]
        + q[tid][2, 3] * coeffs[tid][2]
        + q[tid][3, 3] * coeffs[tid][3]
    )
    coeffs[tid] = _cubic_interpolator(y[tid], coeffs[tid])
    out[tid] = (
        a0 * coeffs[tid][0]
        + a1 * coeffs[tid][1]
        + a2 * coeffs[tid][2]
        + a3 * coeffs[tid][3]
    )


@dataclasses.dataclass
class GeoClipmapSpecs:
    startingLODLevel: int = 0
    numMeshLODLevels: int = 5
    meshBaseLODExtentHeightfieldTexels: int = 256
    meshBackBonePath: str = "terrain_mesh_backbone.npz"
    source_resolution: float = 5.0
    minimum_target_resolution: float = 5.0


def Point3(x, y, z):
    return np.array([x, y, z])


class GeoClipmap:
    # This code is inspired from: https://github.com/morgan3d/misc/tree/master/terrain
    def __init__(
        self,
        specs: GeoClipmapSpecs,
        interpolation_method="bilinear",
        acceleration_mode="hybrid",
    ):
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
        self.specs_hash = self.compute_hash(self.specs)
        self.initMesh()

    def build(self, dem, dem_shape):
        self.DEM_sampler = DEMSampler(
            dem,
            dem_shape,
            self.specs,
            self.points,
            interpolation_method=self.interpolation_method,
            acceleration_mode=self.acceleration_mode,
        )

    def gridIndex(self, x, y, stride):
        return y * stride + x

    @staticmethod
    def compute_hash(specs):
        return hashlib.sha256(str(specs).encode("utf-8")).hexdigest()

    def querryPointIndex(self, point):
        hash = str(point[:2])
        if hash in self.prev_indices.keys():
            index = self.prev_indices[hash]
        elif hash in self.new_indices.keys():
            index = self.new_indices[hash]
        else:
            index = copy(self.index_count)
            self.points.append(point)
            self.new_indices[hash] = index
            self.index_count += 1
        return index

    def addTriangle(self, A, B, C):
        A_idx = self.querryPointIndex(A)
        B_idx = self.querryPointIndex(B)
        C_idx = self.querryPointIndex(C)
        self.indices.append(A_idx)
        self.indices.append(B_idx)
        self.indices.append(C_idx)
        self.uvs.append(A[:2])
        self.uvs.append(B[:2])
        self.uvs.append(C[:2])

    def buildMesh(self):
        print("Building the mesh backbone, this may take time...")
        for level in range(0, self.specs.numMeshLODLevels):
            print(
                "Generating level "
                + str(level + 1)
                + " out of "
                + str(self.specs.numMeshLODLevels)
                + "..."
            )
            step = 1 << level
            if level == 0:
                prevStep = 0
            else:
                prevStep = max(0, (1 << (level - 1)))
            halfStep = prevStep

            g = self.specs.meshBaseLODExtentHeightfieldTexels / 2
            L = float(level)

            # Pad by one element to hide the gap to the next level
            pad = 0
            radius = int(step * (g + pad))
            for y in range(-radius, radius, step):
                for x in range(-radius, radius, step):
                    if max(abs(x + halfStep), abs(y + halfStep)) >= g * prevStep:
                        # Cleared the cutout from the previous level. Tessellate the
                        # square.

                        #   A-----B-----C
                        #   | \   |   / |
                        #   |   \ | /   |
                        #   D-----E-----F
                        #   |   / | \   |
                        #   | /   |   \ |
                        #   G-----H-----I

                        A = Point3(float(x), float(y), L)
                        C = Point3(float(x + step), A[1], L)
                        G = Point3(A[0], float(y + step), L)
                        I = Point3(C[0], G[1], L)

                        B = (A + C) * 0.5
                        D = (A + G) * 0.5
                        F = (C + I) * 0.5
                        H = (G + I) * 0.5

                        E = (A + I) * 0.5

                        # Stitch the border into the next level

                        if x == -radius:
                            #   A-----B-----C
                            #   | \   |   / |
                            #   |   \ | /   |
                            #   |     E-----F
                            #   |   / | \   |
                            #   | /   |   \ |
                            #   G-----H-----I
                            self.addTriangle(E, A, G)
                        else:
                            self.addTriangle(E, A, D)
                            self.addTriangle(E, D, G)

                        if y == (radius - step):
                            self.addTriangle(E, G, I)
                        else:
                            self.addTriangle(E, G, H)
                            self.addTriangle(E, H, I)

                        if x == (radius - step):
                            self.addTriangle(E, I, C)
                        else:
                            self.addTriangle(E, I, F)
                            self.addTriangle(E, F, C)

                        if y == -radius:
                            self.addTriangle(E, C, A)
                        else:
                            self.addTriangle(E, C, B)
                            self.addTriangle(E, B, A)
            self.prev_indices = copy(self.new_indices)
            self.new_indices = {}
        self.points = np.array(self.points) * 2 * self.specs.minimum_target_resolution
        self.uvs = np.array(self.uvs) * 2 * self.specs.minimum_target_resolution
        self.indices = np.array(self.indices)

    def saveMesh(self):
        np.savez_compressed(
            self.specs.meshBackBonePath,
            points=self.points,
            indices=self.indices,
            uvs=self.uvs,
            specs_hash=self.specs_hash,
        )

    def loadMesh(self):
        data = np.load(self.specs.meshBackBonePath)
        if data["specs_hash"] != self.specs_hash:
            self.buildMesh()
            self.saveMesh()
        else:
            self.points = data["points"]
            self.indices = data["indices"]
            self.uvs = data["uvs"]

    def initMesh(self):
        # Cache the mesh backbone between runs because it is expensive to generate
        if os.path.exists(self.specs.meshBackBonePath):
            self.loadMesh()
        else:
            self.buildMesh()
            self.saveMesh()

    def updateElevation(self, coordinates):
        self.DEM_sampler.getElevation(coordinates)


class DEMSampler:
    def __init__(
        self,
        dem,
        dem_size,
        specs,
        points,
        interpolation_method="bilinear",
        acceleration_mode="hybrid",
    ):
        self.dem = dem  # Reference (read only)
        self.dem_size = dem_size  # Reference (read only)
        self.specs = specs  # Reference (read only)
        self.points = points  # Reference (read only)

        self.interpolation_method = interpolation_method
        self.acceleration_mode = acceleration_mode
        self.initialize_warp_buffers()

    def initialize_warp_buffers(self):
        if self.acceleration_mode == "hybrid":
            self.initialize_warp_buffers_hybrid_mode()
        elif self.acceleration_mode == "gpu":
            self.initialize_warp_buffers_gpu_mode()
        else:
            raise ValueError("Invalid acceleration mode")

    def initialize_warp_buffers_hybrid_mode(self):
        self.dem_wp = wp.array(
            self.dem.flatten(), dtype=float, device="cpu", pinned=True
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

    def initialize_warp_buffers_gpu_mode(self):
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

    def getElevation(self, position):
        if self.acceleration_mode == "hybrid":
            self.getElevationHybrid(position)
        elif self.acceleration_mode == "gpu":
            self.getElevationGPU(position)
        else:
            raise ValueError("Invalid acceleration mode")

    def getElevationHybrid(self, position):
        with wp.ScopedTimer("preprocess_Hybrid", active=True):
            position_in_pixel = position * (1.0 / self.specs.source_resolution)
            coords = wp.vec2f(position_in_pixel[0], position_in_pixel[1])
            wp.launch(
                kernel=preprocess,
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
        if self.interpolation_method == "bilinear":
            self.bilinear_interpolation_hybrid()
        elif self.interpolation_method == "bicubic":
            self.bicubic_interpolation_hybrid()
        else:
            raise ValueError("Invalid interpolation method")

    def bilinear_interpolation_hybrid(self):
        with wp.ScopedTimer("get_values_wp_Hybrid", active=True):
            wp.launch(
                kernel=get_values_wp_2x2,
                dim=self.points.shape[0],
                inputs=[
                    self.dem_wp,
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

    def bicubic_interpolation_hybrid(self):
        with wp.ScopedTimer("get_values_wp_4x4_Hybrid", active=True):
            wp.launch(
                kernel=get_values_wp_4x4,
                dim=self.points.shape[0],
                inputs=[
                    self.dem_wp,
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

    def getElevationGPU(self, position):
        with wp.ScopedTimer("preprocess_GPU", active=True):
            position_in_pixel = position * (1.0 / self.specs.source_resolution)
            coords = wp.vec2f(position_in_pixel[0], position_in_pixel[1])
            wp.launch(
                kernel=preprocess,
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
        if self.interpolation_method == "bilinear":
            self.bilinear_interpolation_GPU()
        elif self.interpolation_method == "bicubic":
            self.bicubic_interpolation_GPU()
        else:
            raise ValueError("Invalid interpolation method")

    def bilinear_interpolation_GPU(self):
        with wp.ScopedTimer("get_values_wp_GPU", active=True):
            wp.launch(
                kernel=get_values_wp_2x2,
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

    def bicubic_interpolation_GPU(self):
        with wp.ScopedTimer("get_values_wp_4x4_GPU", active=True):
            wp.launch(
                kernel=get_values_wp_4x4,
                dim=self.points.shape[0],
                inputs=[
                    self.dem_wp,
                    self.dem_shape_wp,
                    self.x_wp,
                    self.y_wp,
                    self.q_cpu,
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
    specs = GeoClipmapSpecs()
    clipmap = GeoClipmap(specs)
