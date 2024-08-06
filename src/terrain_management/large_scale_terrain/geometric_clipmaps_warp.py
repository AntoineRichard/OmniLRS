__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

import warp as wp


@wp.kernel
def _preprocess(
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    points: wp.array(dtype=wp.vec2f),
    coord: wp.vec2f,
    mpp: float,
    dem_shape: wp.vec2f,
):
    """
    Pre-process the coordinates of the points to be queried.
    It converts the pixel coordinates to meters and adds the offset.
    It also clamps the coordinates to the DEM shape.

    Args:
        x (wp.array(dtype=float)): x coordinates of the points (this is the output).
        y (wp.array(dtype=float)): y coordinates of the points (this is the output).
        points (wp.array(dtype=wp.vec2f)): points to query.
        coord (wp.vec2f): offset to add to the coordinates.
        mpp (float): meters per pixel.
        dem_shape (wp.vec2f): shape of the DEM.
    """

    tid = wp.tid()
    x[tid] = points[tid][0] / mpp + coord[0]
    y[tid] = points[tid][1] / mpp + coord[1]
    x[tid] = wp.atomic_min(x, tid, dem_shape[0] - 1.0)
    y[tid] = wp.atomic_min(y, tid, dem_shape[1] - 1.0)
    x[tid] = wp.atomic_max(x, tid, 0.0)
    y[tid] = wp.atomic_max(y, tid, 0.0)


@wp.func
def _get_4x4_mat(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: float,
    y: float,
    out: wp.mat44f,
) -> wp.mat44f:
    """
    Gets a patch of 4x4 values from the DEM centered at the given coordinates (x, y).
    The function automatically clamps the coordinates to the DEM shape.

    Args:
        dem (wp.array(dtype=float)): DEM.
        dem_shape (wp.vec2i): shape of the DEM.
        x (float): x coordinate.
        y (float): y coordinate.
        out (wp.mat44f): output matrix.
    """

    x0 = wp.max(int(x) - 1, 0)
    y0 = wp.max(int(y) - 1, 0)
    x1 = wp.min(x0 + 1, dem_shape[0] - 1) * dem_shape[1]
    x2 = wp.min(x0 + 2, dem_shape[0] - 1) * dem_shape[1]
    x3 = wp.min(x0 + 3, dem_shape[0] - 1) * dem_shape[1]
    x0 = x0 * dem_shape[1]
    y1 = wp.min(y0 + 1, dem_shape[1] - 1)
    y2 = wp.min(y0 + 2, dem_shape[1] - 1)
    y3 = wp.min(y0 + 3, dem_shape[1] - 1)
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
def _get_values_wp_4x4(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    out: wp.array(dtype=wp.mat44f),
):
    """
    Gets all the 4x4 DEM patches from the array of x and y coordinates.

    Args:
        dem (wp.array(dtype=float)): DEM.
        dem_shape (wp.vec2i): shape of the DEM.
        x (wp.array(dtype=float)): x coordinates.
        y (wp.array(dtype=float)): y coordinates.
        out (wp.array(dtype=wp.mat44f)): output
    """

    tid = wp.tid()
    out[tid] = _get_4x4_mat(dem, dem_shape, x[tid], y[tid], out[tid])


@wp.func
def _get_2x2_mat(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: float,
    y: float,
    out: wp.mat22f,
) -> wp.mat22f:
    """
    Gets a patch of 2x2 values from the DEM centered at the given coordinates (x, y).
    The function automatically clamps the coordinates to the DEM shape.

    Args:
        dem (wp.array(dtype=float)): DEM.
        dem_shape (wp.vec2i): shape of the DEM.
        x (float): x coordinate.
        y (float): y coordinate.
        out (wp.mat22f): output matrix.
    """

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
def _get_values_wp_2x2(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    out: wp.array(dtype=wp.mat22f),
):
    """
    Gets all the 2x2 DEM patches from the array of x and y coordinates.

    Args:
        dem (wp.array(dtype=float)): DEM.
        dem_shape (wp.vec2i): shape of the DEM.
        x (wp.array(dtype=float)): x coordinates.
        y (wp.array(dtype=float)): y coordinates.
        out (wp.array(dtype=wp.mat22f)): output
    """

    tid = wp.tid()
    out[tid] = _get_2x2_mat(dem, dem_shape, x[tid], y[tid], out[tid])


@wp.func
def _bilinear_interpolator(
    x: float,
    y: float,
    q: wp.mat22f,
):
    """
    Bilinear interpolation of a single point.

    Args:
        x (float): x coordinate.
        y (float): y coordinate.
        q (wp.mat22f): 2x2 matrix.
    """

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
    """
    Bilinear interpolation of all the points in the array.

    Args:
        x (wp.array(dtype=float)): x coordinates.
        y (wp.array(dtype=float)): y coordinates.
        q (wp.array(dtype=wp.mat22f)): 2x2 matrices.
        out (wp.array(dtype=float)): output.
    """

    tid = wp.tid()
    out[tid] = _bilinear_interpolator(x[tid], y[tid], q[tid])


@wp.func
def _cubic_interpolator(
    x: float,
    coeffs: wp.vec4f,
):
    """
    Cubic interpolation of a single point.

    Args:
        x (float): x coordinate.
        coeffs (wp.vec4f): coefficients.
    """

    x2 = x - wp.trunc(x)
    a = -0.5
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
    """
    Bicubic interpolation of all the points in the array.

    Args:
        x (wp.array(dtype=float)): x coordinates.
        y (wp.array(dtype=float)): y coordinates.
        q (wp.array(dtype=wp.mat44f)): 4x4 matrices.
        coeffs (wp.array(dtype=wp.vec4f)): coefficients.
        out (wp.array(dtype=float)): output.
    """

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
