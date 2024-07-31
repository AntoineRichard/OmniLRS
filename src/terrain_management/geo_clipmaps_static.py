__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

# This code is based on: https://github.com/morgan3d/misc/tree/master/terrain
# Original author: Morgan McGuire, http://cs.williams.edu/~morgan

from copy import copy
import warp as wp
import dataclasses
import numpy as np
from numba import njit, prange
import hashlib
import time
import os


@njit(parallel=True)
def get_values(dem, x, y):
    output = np.empty((x.shape[0] * 4), dtype=dem.dtype)
    x_max = dem.shape[0]
    for i in prange(x.shape[0]):
        output[i * 4] = dem[x[i], y[i]]
        output[i * 4 + 1] = dem[min(x[i] + 1, x_max), y[i]]
        output[i * 4 + 2] = dem[min(x[i] + 2, x_max), y[i]]
        output[i * 4 + 3] = dem[min(x[i] + 3, x_max), y[i]]
    return output


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

    # x_int[tid] = wp.trunc(x[tid])
    # y_int[tid] = wp.trunc(y[tid])

    # x_delta[tid] = x - x_int[tid]
    # y_delta[tid] = x - y_int[tid]


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
    dem: wp.array(dtype=float), dem_shape: wp.vec2i, x: float, y: float, out: wp.mat44f
) -> wp.mat44f:
    x0 = int(x)
    y0 = int(y)
    x1 = wp.min(x0 + 1, dem_shape[0]) * dem_shape[1]
    x2 = wp.min(x0 + 2, dem_shape[0]) * dem_shape[1]
    x3 = wp.min(x0 + 3, dem_shape[0]) * dem_shape[1]
    x0 = x0 * dem_shape[1]
    y1 = wp.min(y0 + 1, dem_shape[1])
    y2 = wp.min(y0 + 2, dem_shape[1])
    y3 = wp.min(y0 + 3, dem_shape[1])
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


@wp.kernel
def _linear_interpolation(
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    q11: wp.array(dtype=float),
    q12: wp.array(dtype=float),
    q21: wp.array(dtype=float),
    q22: wp.array(dtype=float),
    out: wp.array(dtype=float),
):
    tid = wp.tid()
    out[tid] = ((1.0 - x[tid]) * q11[tid] + x[tid] * q21[tid]) * (1.0 - y[tid]) + (
        (1.0 - x[tid]) * q12[tid] + x[tid] * q22[tid]
    ) * y[tid]


@wp.func
def _cubic_interpolation(
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
    coeffs[tid] = _cubic_interpolation(x[tid], coeffs[tid])
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
    # a1 = wp.dot(q[tid][:, 1], coeffs[tid])
    # a2 = wp.dot(q[tid][:, 2], coeffs[tid])
    # a3 = wp.dot(q[tid][:, 3], coeffs[tid])
    coeffs[tid] = _cubic_interpolation(y[tid], coeffs[tid])
    out[tid] = (
        a0 * coeffs[tid][0]
        + a1 * coeffs[tid][1]
        + a2 * coeffs[tid][2]
        + a3 * coeffs[tid][3]
    )


@dataclasses.dataclass
class GeoClipmapSpecs:
    numMeshLODLevels: int = 6
    meshBaseLODExtentHeightfieldTexels: int = 256
    meshBackBonePath: str = "terrain_mesh_backbone.npz"
    demPath: str = "assets/Terrains/SouthPole/NPD_final_adj_5mpp_surf/dem.npy"
    meters_per_pixel: float = 5.0
    meters_per_texel: float = 1.0
    z_scale: float = 1.0


def Point3(x, y, z):
    return np.array([x, y, z])


class GeoClipmap:
    def __init__(self, specs: GeoClipmapSpecs):
        self.specs = specs
        self.index_count = 0
        self.prev_indices = {}
        self.new_indices = {}
        self.points = []
        self.uvs = []
        self.indices = []

        self.specs_hash = self.compute_hash(self.specs)

        self.initMesh()
        self.loadDEM()
        self.initial_position = [0, 0]
        self.initialize_warp_buffers()

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
        self.points = np.array(self.points) * 2 * self.specs.meters_per_texel
        self.uvs = np.array(self.uvs) * 2 * self.specs.meters_per_texel
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

    def loadDEM(self):
        self.dem = np.load(self.specs.demPath) * self.specs.z_scale
        self.dem_size = self.dem.shape

    def getElevation(self, position):
        # s = time.time()
        # x = (self.points[:, 0] / self.specs.meters_per_pixel) + position_in_pixel[0]
        # y = (self.points[:, 1] / self.specs.meters_per_pixel) + position_in_pixel[1]

        # x = np.minimum(x, self.dem.shape[0] - 1)
        # y = np.minimum(y, self.dem.shape[1] - 1)
        # x = np.maximum(x, 0)
        # y = np.maximum(y, 0)
        # e = time.time()
        # print("Time to compute the positions: ", e - s)

        with wp.ScopedTimer("preprocess", active=True):
            position_in_pixel = position * (1.0 / self.specs.meters_per_pixel)
            coords = wp.vec2f(position_in_pixel[0], position_in_pixel[1])
            wp.launch(
                kernel=preprocess,
                dim=self.points.shape[0],
                inputs=[
                    self.x_wp,
                    self.y_wp,
                    self.points_wp,
                    coords,
                    self.specs.meters_per_pixel,
                    self.dem_shape_wp_f,
                ],
                device="cuda",
            )
            self.x_wp_cpu.assign(self.x_wp)
            self.y_wp_cpu.assign(self.y_wp)

        self.bicubic_interpolation()
        # bicubic interpolation is broken

    def linear_interpolation(self):
        x1 = np.trunc(x).astype(int)
        y1 = np.trunc(y).astype(int)

        x2 = np.minimum(x1 + 1, self.dem_size[0] - 1)
        y2 = np.minimum(y1 + 1, self.dem_size[1] - 1)
        dx = x - x1
        dy = y - y1

        q11 = self.dem[x1, y1]
        q12 = self.dem[x1, y2]
        q21 = self.dem[x2, y1]
        q22 = self.dem[x2, y2]

        z = wp.zeros(x.shape[0], dtype=float)
        with wp.ScopedTimer("linear_interpolation", active=True):
            wp.launch(
                kernel=_linear_interpolation,
                dim=self.points.shape[0],
                inputs=[
                    wp.array(dx, dtype=float),
                    wp.array(dy, dtype=float),
                    wp.array(q11, dtype=float),
                    wp.array(q12, dtype=float),
                    wp.array(q21, dtype=float),
                    wp.array(q22, dtype=float),
                    z,
                ],
            )
        return z

    def initialize_warp_buffers(self):
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
        self.x_int_cpu = wp.zeros(
            (self.points.shape[0]), dtype=int, device="cpu", pinned=True
        )
        self.y_int_cpu = wp.zeros(
            (self.points.shape[0]), dtype=int, device="cpu", pinned=True
        )
        self.x_delta = wp.zeros((self.points.shape[0]), dtype=float, device="cuda")
        self.y_delta = wp.zeros((self.points.shape[0]), dtype=float, device="cuda")
        self.q_cpu = wp.zeros(
            (self.points.shape[0]),
            dtype=wp.mat44f,
            device="cpu",
            pinned=True,
        )
        self.q_cuda = wp.zeros((self.points.shape[0]), dtype=wp.mat44f, device="cuda")
        self.dem_shape_wp = wp.vec2i(self.dem_size[0], self.dem_size[1])
        self.dem_shape_wp_f = wp.vec2f(self.dem_size[0], self.dem_size[1])
        self.coeffs_wp = wp.zeros(self.points.shape[0], dtype=wp.vec4f, device="cuda")
        self.z_cuda = wp.zeros(self.points.shape[0], dtype=float, device="cuda")

    def bicubic_interpolation(self):  # , x, y):

        # s = time.time()
        # x1 = np.maximum(np.trunc(x).astype(int) - 1, 0)
        # y1 = np.maximum(np.trunc(y).astype(int) - 1, 0)
        # x2 = np.minimum(x1 + 0, self.dem_size[0])
        # y2 = np.minimum(y1 + 0, self.dem_size[1])
        # x3 = np.minimum(x1 + 1, self.dem_size[0] - 1)
        # y3 = np.minimum(y1 + 1, self.dem_size[1] - 1)
        # x4 = np.minimum(x1 + 2, self.dem_size[0] - 2)
        # y4 = np.minimum(y1 + 2, self.dem_size[1] - 2)
        # dx = wp.from_numpy(x - x2, dtype=float, device="cuda")
        # dy = wp.from_numpy(y - y2, dtype=float, device="cuda")
        # e = time.time()
        # print("Time to cast vectors: ", e - s)

        # self.x_int.assign(x1)
        # self.y_int.assign(y1)
        with wp.ScopedTimer("get_values_wp_4x4", active=True):
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
        # q = self.q_cpu.numpy()
        # q1 = q[:, :, 0]
        # q2 = q[:, :, 1]
        # q3 = q[:, :, 2]
        # q4 = q[:, :, 3]

        # q11 = self.dem[x1, y1]
        # q12 = self.dem[x2, y1]
        # q13 = self.dem[x3, y1]
        # q14 = self.dem[x4, y1]
        # q21 = self.dem[x1, y2]
        # q22 = self.dem[x2, y2]
        # q23 = self.dem[x3, y2]
        # q24 = self.dem[x4, y2]
        # q31 = self.dem[x1, y3]
        # q32 = self.dem[x2, y3]
        # q33 = self.dem[x3, y3]
        # q34 = self.dem[x4, y3]
        # q41 = self.dem[x1, y4]
        # q42 = self.dem[x2, y4]
        # q43 = self.dem[x3, y4]
        # q44 = self.dem[x4, y4]
        # q1 = get_values(self.dem, x1, y1)
        # q2 = get_values(self.dem, x1, y2)
        # q3 = get_values(self.dem, x1, y3)
        # q4 = get_values(self.dem, x1, y4)

        # s = time.time()
        # q1 = wp.from_numpy(q1, dtype=wp.vec4f, device="cuda")
        # q2 = wp.from_numpy(q2, dtype=wp.vec4f, device="cuda")
        # q3 = wp.from_numpy(q3, dtype=wp.vec4f, device="cuda")
        # q4 = wp.from_numpy(q4, dtype=wp.vec4f, device="cuda")
        # q1 = wp.from_numpy(
        #    np.array([q11, q12, q13, q14]).T, dtype=wp.vec4f, device="cuda"
        # )
        # q2 = wp.from_numpy(
        #    np.array([q21, q22, q23, q24]).T, dtype=wp.vec4f, device="cuda"
        # )
        # q3 = wp.from_numpy(
        #    np.array([q31, q32, q33, q34]).T, dtype=wp.vec4f, device="cuda"
        # )
        # q4 = wp.from_numpy(
        #    np.array([q41, q42, q43, q44]).T, dtype=wp.vec4f, device="cuda"
        # )
        # e = time.time()
        # print("Time to send matrices to GPU: ", e - s)

        with wp.ScopedTimer("bicubic_interpolation", active=True):
            wp.launch(
                kernel=_bicubic_interpolation,
                dim=self.points.shape[0],
                inputs=[self.x_wp, self.y_wp, self.q_cuda, self.coeffs_wp, self.z_cuda],
            )
            self.points[:, -1] = self.z_cuda.numpy()


if __name__ == "__main__":
    from matplotlib import pyplot as plt
    from mpl_toolkits.mplot3d import axes3d, Axes3D  # <-- Note the capitalization!

    wp.init()
    specs = GeoClipmapSpecs()
    clipmap = GeoClipmap(specs)
    with wp.ScopedTimer("render", active=True):
        clipmap.getElevation(np.array([8192 * 1, 0, 8192 * 1]))

    print(clipmap.points.shape)
    print(clipmap.indices.shape)
    print(clipmap.uvs.shape)
    ax = plt.figure().add_subplot(projection="3d")
    ax.scatter(clipmap.points[:, 0], clipmap.points[:, 1], clipmap.points[:, 2])
    # plt.scatter(points[:,0], points[:,2])
    # plt.axes("equal")
    plt.show()
