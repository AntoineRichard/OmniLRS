__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "develop_ment"

import datetime
import pickle
import cv2
from typing import List, Tuple
from matplotlib import pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import rotate
from scipy.spatial.transform import Rotation as R
import warp as wp

from src.configurations.procedural_terrain_confs import (
    CraterGeneratorConf,
    CraterDistributionConf,
    BaseTerrainGeneratorConf,
    DeformationEngineConf,
    MoonYardConf,
)


class CraterGenerator:
    """
    Generates craters DEM from a set of spline profiles and randomizes their 2D appearance
    """

    def __init__(
        self,
        cfg: CraterGeneratorConf,
    ) -> None:
        """
        Args:
            profiles_path (str): path to the pickle file containing the spline profiles.
            seed (int, optional): random seed. Defaults to 42.
            min_xy_ratio (float, optional): minimum xy ratio of the crater. Defaults to 0.85.
            max_xy_ratio (float, optional): maximum xy ratio of the crater. Defaults to 1.
            resolution (float, optional): resolution of the DEM (in meters per pixel). Defaults to 0.01.
            pad_size (int, optional): size of the padding to add to the DEM. Defaults to 500.
            random_rotation (bool, optional): whether to randomly rotate the craters. Defaults to True.
            z_scale (float, optional): scale of the craters. Defaults to 1."""

        self._profiles_path = cfg.profiles_path
        self._resolution = cfg.resolution
        self._min_xy_ratio = cfg.min_xy_ratio
        self._max_xy_ratio = cfg.max_xy_ratio
        self._z_scale = cfg.z_scale
        self._random_rotation = cfg.random_rotation
        self._pad_size = cfg.pad_size
        self._profiles = None
        self._rng = np.random.default_rng(cfg.seed)

        self.loadProfiles()

    def loadProfiles(self) -> None:
        """
        Loads the half crater spline profiles from a pickle file."""

        with open(self._profiles_path, "rb") as handle:
            self._profiles = pickle.load(handle)

    def sat_gaussian(
        self, x: np.ndarray, mu1: float, mu2: float, std: float
    ) -> np.ndarray:
        """
        Saturates a gaussian function to its maximum between mu1 and mu2 with a standard deviation of std.

        Args:
            x (np.ndarray): input array.
            mu1 (float): gaussian mu lower bound.
            mu2 (float): gaussian mu upper bound.
            std (float): standard deviation.

        Returns:
            np.ndarray: saturated gaussian."""

        shape = x.shape
        x = x.flatten()
        x[x < mu1] = np.exp(-0.5 * ((x[x < mu1] - mu1) / std) ** 2)
        x[x > mu2] = np.exp(-0.5 * ((x[x > mu2] - mu2) / std) ** 2)
        x[(x >= mu1) & (x <= mu2)] = 1.0
        x = x / (std * np.sqrt(2 * np.pi))
        x = x.reshape(shape)
        return x

    def centeredDistanceMatrix(self, n: int) -> Tuple[np.ndarray, int]:
        """
        Generates a distance matrix centered at the center of the matrix.

        Args:
            n (int): size of the matrix

        Returns:
            tuple: distance matrix, size of the matrix."""

        # Makes sure the matrix size is odd
        n = n + ((n % 2) == 0)

        # Generates a profile to deform the crater
        tmp_y = self._rng.uniform(0.95, 1, 9)
        tmp_y = np.concatenate([tmp_y, [tmp_y[0]]], axis=0)
        tmp_x = np.linspace(0, 1, tmp_y.shape[0])
        s = CubicSpline(tmp_x, tmp_y, bc_type=((1, 0.0), (1, 0.0)))

        # Generates a profile to add marks that converges toward the center of the crater
        tmp_y = self._rng.uniform(0.0, 0.01, 45)
        tmp_y = np.concatenate([tmp_y, [tmp_y[0]]], axis=0)
        tmp_x = np.linspace(0, 1, tmp_y.shape[0])
        s2 = CubicSpline(tmp_x, tmp_y, bc_type=((1, 0.0), (1, 0.0)))

        # Generates the deformation matrix
        m = np.zeros([n, n])
        x, y = np.meshgrid(np.linspace(-1, 1, n), np.linspace(-1, 1, n))
        theta = np.arctan2(y, x)
        fac = s(theta / (2 * np.pi) + 0.5)
        # Generates the marks matrix
        marks = s2(theta / (2 * np.pi) + 0.5) * n / 2 * self._rng.uniform(0, 1)

        # Generates the distance matrix
        sx = self._rng.uniform(self._min_xy_ratio, self._max_xy_ratio)
        sy = 1.0
        x, y = np.meshgrid(range(n), range(n))
        m = np.sqrt(
            ((x - (n / 2) + 1) * 1 / sx) ** 2 + ((y - (n / 2) + 1) * 1 / sy) ** 2
        )

        # Deforms the distance matrix
        m = m * fac

        # Smoothes the marks such that they are not present on the outer edge of the crater
        sat = self.sat_gaussian(m, 0.15 * n / 2, 0.45 * n / 2, 0.05 * n / 2)
        sat = (sat - sat.min()) / (sat.max() - sat.min())

        # Adds the marks to the distance matrix
        m = m + marks * sat

        # Rotate the matrix
        theta = int(self._rng.uniform(0, 360))
        m = rotate(m, theta, reshape=False, cval=n / 2)

        # Saturate the matrix such that the distance is not greater than the radius of the crater
        m[m > n / 2] = n / 2
        return m, n

    def applyProfile(
        self, profile: CubicSpline, distance: np.ndarray, size: int
    ) -> np.ndarray:
        """
        Applies a profile to the distance matrix.

        Args:
            profile (CubicSpline): profile to apply.
            distance (np.ndarray): distance matrix.
            size (int): size of the matrix.

        Returns:
            np.ndarray: crater DEM."""

        crater = profile(2 * distance / size)
        return crater

    def getProfile(self, index: int) -> CubicSpline:
        """
        Gets a random profile from the list of profiles.

        Args:
            index (int): index of the profile to get.

        Returns:
            CubicSpline: profile."""

        profile = None
        if index < -1:
            raise ValueError("Unknown profile")
        elif index == -1:
            profile = self._rng.choice(self._profiles)
        elif index < len(self._profiles):
            profile = self._profiles[i]
        else:
            raise ValueError("Unknown profile")
        return profile

    def generateCrater(self, size: int, s_index: int = -1) -> Tuple[np.ndarray, int]:
        """
        Generates a crater DEM.

        Args:
            size (float): size of the crater.
            s_index (int): index of the profile to use.

        Returns:
            tuple: crater DEM, size of the crater."""

        distance, size = self.centeredDistanceMatrix(size)
        profile = self.getProfile(s_index)
        crater = (
            self.applyProfile(profile, distance, size)
            * size
            / 2.0
            * self._z_scale
            * self._resolution
        )
        return crater, size

    def generateCraters(
        self, DEM: np.ndarray, coords: np.ndarray, radius: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generates a DEM with craters.

        Args:
            DEM (np.ndarray): DEM to add craters to.
            coords (np.ndarray): coordinates of the craters (in meters).

        Returns:
            np.ndarray: unpadded DEM with craters, and matching mask."""

        DEM_padded = np.zeros(
            (self._pad_size * 2 + DEM.shape[0], self._pad_size * 2 + DEM.shape[1])
        )
        mask_padded = np.ones_like(DEM_padded)
        DEM_padded[
            self._pad_size : -self._pad_size, self._pad_size : -self._pad_size
        ] = DEM
        for coord, rad in zip(coords, radius):
            rad = int(rad * 2 / self._resolution)
            coord = coord / self._resolution
            c, rad = self.generateCrater(int(rad))
            coord2 = (coord + self._pad_size).astype(np.int64)
            coord = (coord - rad / 2 + self._pad_size).astype(np.int64)
            DEM_padded[coord[0] : coord[0] + rad, coord[1] : coord[1] + rad] += c
            mask_padded = cv2.circle(
                mask_padded, (coord2[1], coord2[0]), int(rad / 4), 0, -1
            )
        mask_padded[: self._pad_size + 1, :] = 0
        mask_padded[:, : self._pad_size + 1] = 0
        mask_padded[-self._pad_size - 1 :, :] = 0
        mask_padded[:, -self._pad_size - 1 :] = 0
        return (
            DEM_padded[
                self._pad_size : -self._pad_size, self._pad_size : -self._pad_size
            ],
            mask_padded[
                self._pad_size : -self._pad_size, self._pad_size : -self._pad_size
            ],
        )


class Distribute:
    """
    Distributes craters on a DEM using a Poisson process with hardcore rejection."""

    def __init__(
        self,
        cfg: CraterDistributionConf,
    ) -> None:
        """
        Args:
            x_size (float, optional): size of the DEM in the x direction (in meters). Defaults to 10.
            y_size (float, optional): size of the DEM in the y direction (in meters). Defaults to 10.
            densities (float, optional): densities of the craters (in units per square meters). Defaults to [0.25,1.5,5].
            radius (list, optional): min and max radii of the craters (in meters). Defaults to [(1.5,2.5),(0.75,1.5),(0.25,0.5)].
            num_repeat (int, optional): number of times to repeat the hardcore rejection. Defaults to 0.
            seed (int, optional): random seed. Defaults to 42."""

        self._x_max = cfg.x_size
        self._y_max = cfg.y_size
        self._densities = cfg.densities
        self._radius = cfg.radius
        self._area = self._x_max * self._y_max
        self._num_repeat = cfg.num_repeat
        self._rng = np.random.default_rng(cfg.seed)

    def sampleFromPoisson(
        self, l: float, r_minmax: Tuple[float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Samples from a Poisson process.

        Args:
            l (float): density of the Poisson process (in units per square meters).
            r_minmax (tuple): minimum and maximum radius of the craters (in meters).

        Returns:
            tuple: coordinates and radius of the craters"""

        num_points = self._rng.poisson(self._area * l)
        radius = self._rng.uniform(r_minmax[0], r_minmax[1], num_points)
        x_coords = self._rng.uniform(0, self._x_max, num_points)
        y_coords = self._rng.uniform(0, self._y_max, num_points)
        return np.stack([x_coords, y_coords]).T, radius

    def hardcoreRejection(
        self, coords: np.ndarray, radius: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Performs hardcore rejection on the craters.

        Args:
            coords (np.ndarray): coordinates of the craters (in meters).
            radius (np.ndarray): radii of the craters (in meters).

        Returns:
            tuple: coordinates of the craters (in meters)."""

        mark_age = self._rng.uniform(0, 1, coords.shape[0])
        boole_keep = np.zeros(mark_age.shape[0], dtype=bool)
        for i in range(mark_age.shape[0]):
            dist_tmp = np.linalg.norm(coords[i] - coords, axis=-1)
            in_disk = (dist_tmp < radius[i]) & (dist_tmp > 0)
            if len(mark_age[in_disk]) == 0:
                boole_keep[i] = True
            else:
                boole_keep[i] = all(mark_age[i] < mark_age[in_disk])
        return coords[boole_keep], radius[boole_keep]

    def checkPrevious(
        self, new_coords: np.ndarray, radius: np.ndarray, prev_coords: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Checks if the new craters are not in the previous craters.

        Args:
            new_coords (np.ndarray): coordinates of the new craters.
            radius (np.ndarray): radii of the new craters.
            prev_coords (np.ndarray): coordinates of the previous craters.

        Returns:
            tuple: the coordinates of the new craters, the radii of the new craters."""

        boole_keep = np.ones(new_coords.shape[0], dtype=bool)
        if prev_coords is None:
            boole_keep = np.ones(new_coords.shape[0], dtype=bool)
        else:
            for i in range(prev_coords[0].shape[0]):
                dist_tmp = np.linalg.norm(prev_coords[0][i] - new_coords, axis=-1)
                in_disk = (dist_tmp < prev_coords[1][i]) & (dist_tmp > 0)
                boole_keep[in_disk] = False
        return new_coords[boole_keep], radius[boole_keep]

    def simulateHCPoissonProcess(
        self, l: float, r_minmax: Tuple[float], prev_coords: np.ndarray = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simulates a hardcore Poisson process.

        Args:
            l (float): density of the Poisson process (in units per square meters).
            r_minmax (tuple): minimum and maximum radius of the craters (in meters).
            prev_coords (np.ndarray): coordinates of the previous craters (in meters).

        Returns:
            tuple: coordinates of the craters, radii of the craters."""

        coords, radius = self.sampleFromPoisson(l, r_minmax)
        for _ in range(self._num_repeat):
            coords, radius = self.hardcoreRejection(coords, radius)
            new_coords, new_radius = self.sampleFromPoisson(l, r_minmax)
            coords = np.concatenate([coords, new_coords])
            radius = np.concatenate([radius, new_radius])
            self.checkPrevious(coords, radius, prev_coords)
        coords, radius = self.hardcoreRejection(coords, radius)
        coords, radius = self.checkPrevious(coords, radius, prev_coords)
        return coords, radius

    def run(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Runs the hardcore Poisson process for all the densities and radius in order.

        Returns:
            tuple: coordinates of the craters, radii of the craters."""

        prev_coords = None
        for d, r_minmax in zip(self._densities, self._radius):
            new_coords, new_radius = self.simulateHCPoissonProcess(
                d, r_minmax, prev_coords
            )
            if prev_coords is not None:
                prev_coords = (
                    np.concatenate([prev_coords[0], new_coords], axis=0),
                    np.concatenate([prev_coords[1], new_radius], axis=0),
                )
            else:
                prev_coords = (new_coords, new_radius)
        return prev_coords


class BaseTerrainGenerator:
    """
    Generates a random terrain DEM."""

    def __init__(
        self,
        cfg: BaseTerrainGeneratorConf,
    ) -> None:
        """
        Args:
            x_size (float, optional): size of the DEM in the x direction (in meters). Defaults to 10.
            y_size (float, optional): size of the DEM in the y direction (in meters). Defaults to 10.
            resolution (float, optional): resolution of the DEM (in meters per pixel). Defaults to 0.01.
            max_elevation (float, optional): maximum elevation of the DEM (in meters). Defaults to 0.5.
            min_elevation (float, optional): minimum elevation of the DEM (in meters). Defaults to -0.25.
            seed (int, optional): random seed. Defaults to 42.
            z_scale (float, optional): scale of the DEM. Defaults to 50."""

        self._min_elevation = cfg.min_elevation
        self._max_elevation = cfg.max_elevation
        self._x_size = int(cfg.x_size / cfg.resolution)
        self._y_size = int(cfg.y_size / cfg.resolution)
        self._DEM = np.zeros((self._x_size, self._y_size), dtype=np.float32)
        self._rng = np.random.default_rng(cfg.seed)
        self._z_scale = cfg.z_scale

    def generateRandomTerrain(
        self, is_lab: bool = False, is_yard: bool = False
    ) -> np.ndarray:
        """
        Generates a random terrain DEM. With some of its borders at 0 height
        to align with the catawalks in the lab.

        Args:
            is_lab (bool): whether the DEM is in a lab or not.
            is_yard (bool): whether the DEM is in a yard or not.

        Returns:
            DEM (np.ndarray): random terrain DEM."""

        # Generate low frequency noise to simulate large scale terrain features.
        if is_lab:
            lr_noise = np.zeros((4, 4))
            lr_noise[:-1, 1:] = self._rng.uniform(
                self._min_elevation, self._max_elevation, [3, 3]
            )
        elif is_yard:
            lr_noise = np.zeros((7, 7))
            lr_noise[1:-1, 1:-1] = self._rng.uniform(
                self._min_elevation, self._max_elevation, [5, 5]
            )
        else:
            lr_noise = self._rng.uniform(
                self._min_elevation, self._max_elevation, [4, 4]
            )
        hr_noise = cv2.resize(
            lr_noise, (self._y_size, self._x_size), interpolation=cv2.INTER_CUBIC
        )
        self._DEM += hr_noise
        # Generate high frequency noise to simulate small scale terrain features.
        lr_noise = self._rng.uniform(
            self._min_elevation * 0.01, self._max_elevation * 0.01, [100, 100]
        )
        hr_noise = cv2.resize(
            lr_noise, (self._y_size, self._x_size), interpolation=cv2.INTER_CUBIC
        )
        self._DEM += hr_noise
        # Normalize the DEM between 0 and 1 and scale it to the desired elevation range.
        # self._DEM = (self._DEM - self._DEM.min()) / (self._DEM.max() - self._DEM.min())
        # Scale the DEM to the desired elevation range.
        # self._DEM = self._DEM * (self._max_elevation - self._min_elevation) + self._min_elevation
        return self._DEM * self._z_scale

class DeformationEngine:
    """
    Terrain deformation class. 
    Idea is to deform the part of terrain where wheel contacted. 
    """
    def __init__(self, deformation_engine:DeformationEngineConf)-> None:
        """
        Args:
            deformation_engine (DeformationEngineConf): deformation engine configuration.
        """
        self.terrain_resolution = deformation_engine.terrain_resolution
        self.terrain_width = deformation_engine.terrain_width
        self.terrain_height = deformation_engine.terrain_height
        self.wheel_width = deformation_engine.wheel_params.wheel_width
        self.wheel_radius = deformation_engine.wheel_params.wheel_radius
        self.deform_offset = deformation_engine.deform_constraint.deform_offset
        self.deform_decay_ratio = deformation_engine.deform_constraint.deform_decay_ratio
        self.force_distribution = deformation_engine.force_distribution
        self.boundary_distribution = deformation_engine.boundary_distribution
        self.force_depth_slope = deformation_engine.force_depth_slope
        self.force_depth_intercept = deformation_engine.force_depth_intercept

        self.sim_width = self.terrain_width / self.terrain_resolution
        self.sim_height = self.terrain_height / self.terrain_resolution
        
        # create null variable
        self.profile = None
        self.force_dist = None
        self.boundary_dist = None

        self.create_profile()
        self.get_force_distribution()
        self.get_boundary_distribution()

    def create_profile(self):
        """
        Create a profile of trace in wheel frame (FLU)
        Returns:
            profile (np.ndarray): profile of wheel deformation (num_point_sample, 2)
        """
        xs = np.linspace(-self.wheel_radius, self.wheel_radius, int(2*self.wheel_radius/self.terrain_resolution)+1) + self.deform_offset
        ys = np.linspace(-self.wheel_width/2, self.wheel_width/2, int(self.wheel_width/self.terrain_resolution)+1)
        X, Y = np.meshgrid(xs, ys)
        self.profile = np.column_stack([X.flatten(), Y.flatten()])
        self.profile_width = X.shape[0]
        self.profile_height = X.shape[1]
    
    def get_force_distribution(self)-> None:
        """
        Get force distribution over profile from single contact force.
            uniform: force is uniformly distributed over profile.
            sinusoidal: force is distributed sinusoidally over profile.
        force_dist (np.ndarray): force distribution over profile (num_point_sample, 1)
        """
        if self.force_distribution.distribution == "uniform":
            self.force_dist = self.uniform_force_generator()
        elif self.force_distribution.distribution == "sinusoidal":
            self.force_dist = self.sinusoidal_force_generator()
    
    ### Force distribution functions ###
    def uniform_force_generator(self)-> np.ndarray:
        """
        Generate uniform force distribution over profile.
        Return: 
            force_dist (np.ndarray): force distribution over profile (num_point_sample,)
        """
        force_dist = np.ones(self.profile.shape[0], dtype=np.float32)
        return force_dist
    
    def sinusoidal_force_generator(self)-> np.ndarray:
        """
        Generate sinusoidal force distribution over profile.
        Return: 
            force_dist (np.ndarray): force distribution over profile (num_point_sample,)
        """
        force_dist_x = np.cos(2*self.force_distribution.wave_frequency*np.pi * np.linspace(-1, 1, self.profile_height)) # create force distribution on x axis
        force_dist = force_dist_x[None, :].repeat(self.profile_width, axis=0).reshape(-1) # clone force_x to y axis
        return force_dist
    
    ### Boundary distribution functions ###
    def get_boundary_distribution(self)->None:
        """
        Get boundary distribution over profile.
            uniform: boundary distribution is uniformly distributed over profile.
            parabolic: boundary distribution is parabolically distributed over profile.
        boundary_dist (np.ndarray): boundary distribution over profile (num_point_sample, )
        """
        if self.boundary_distribution.distribution == "uniform":
            self.boundary_dist = self.uniform_boundary_generator()
        elif self.boundary_distribution.distribution == "parabolic":
            self.boundary_dist = self.parabolic_boundary_generator()
        elif self.boundary_distribution.distribution == "trapezoidal":
            self.boundary_dist = self.trapezoidal_boundary_generator()
    
    def uniform_boundary_generator(self):
        """
        Generate uniform boundary distribution over profile.
        Return: 
            boundary_dist (np.ndarray): boundary distribution over profile (num_point_sample, )
        """
        boundary_dist = np.ones(self.profile.shape[0])
        return boundary_dist
    
    def parabolic_boundary_generator(self):
        """
        Generate parabolic boundary distribution over profile.
        Return: 
            boundary_dist (np.ndarray): boundary distribution over profile (num_point_sample, )
        """
        y = np.linspace(-1, 1, self.profile_width)
        boundary_dist_y = y**2 - 1
        boundary_dist = boundary_dist_y[None, :].repeat(self.profile_height, axis=1).reshape(-1)
        return boundary_dist
    
    def trapezoidal_boundary_generator(self):
        """
        Generate trapezoidal boundary distribution over profile.
        Return: 
            boundary_dist (np.ndarray): boundary distribution over profile (num_point_sample, )
        """
        tan = np.tan(self.boundary_distribution.angle_of_repose)
        y = np.linspace(-1, 1, self.profile_width)
        mask = (np.abs(y) >= 1 - (1/tan)).astype(np.float32)
        boundary_dist_y = mask * (tan * np.abs(y) - tan + 1) - 1
        boundary_dist = boundary_dist_y[None, :].repeat(self.profile_height, axis=1).reshape(-1)
        return boundary_dist

    def get_profile_projection(self, body_transforms:np.ndarray)-> np.ndarray:
        """
        Get x, y positions of profile in global coordinate.
        Args:
            body_transforms (np.ndarray): body transform of rover's wheels (n, 4, 4)
            n is the number of wheels.
        Returns:
            projection_points (np.ndarray): projected pixel coordinates of rover's wheels (num_points, 2)
            num_points = n * num_point_sample
        """
        projection_points = []
        for i in range(body_transforms.shape[0]):
            body_transform = body_transforms[i]
            heading = R.from_matrix(body_transform[:3, :3]).as_euler("zyx")[0]
            projection_point = np.array(
                [self.profile[:, 0]*np.cos(heading) - self.profile[:, 1]*np.sin(heading), 
                self.profile[:, 0]*np.sin(heading) + self.profile[:, 1]*np.cos(heading)]).T + body_transform[:2, 3]
            projection_points.append(projection_point)
        return np.concatenate(projection_points)
    
    @staticmethod
    def linear_func(force:np.ndarray, slope:float, intercept:float)-> np.ndarray:
        return slope * force + intercept
    
    def get_deformation_depth(self, normal_forces:np.ndarray, model:str="linear")-> np.ndarray:
        """
        Calculate deformation depth (with sign) from force distribution.
        Args:
            normal_forces (np.ndarray): contact force fields (n,)
        Returns:
            depth (np.ndarray): deformation depth (num_points, 1)
            num_points = n * num_point_sample
        """
        if model == "linear":
            forces = np.concatenate(
                [self.linear_func(
                    normal_force, self.force_depth_slope, self.force_depth_intercept
                    ) * self.boundary_dist * self.force_dist for normal_force in normal_forces])
            return forces
        else:
            raise ValueError("Unknown model")
    
    def deform(self, DEM:np.ndarray, multipass:np.ndarray, body_transforms:np.ndarray, normal_forces:np.ndarray)-> np.ndarray:
        """
        Args:
            DEM (np.ndarray): DEM to deform
            body_transforms (np.ndarray): projected coordinates of rover's wheels (n, 4, 4)
            normal_forces (np.ndarray): (dynamic) contact forces applied on rover's wheels (n, 3)
            n is the number of wheels.
        """
        profile_points = self.get_profile_projection(body_transforms=body_transforms)
        deformation_depth = self.get_deformation_depth(normal_forces=normal_forces)
        for i, profile_point in enumerate(profile_points):
            x = int(profile_point[0]/self.terrain_resolution)
            y = int(self.sim_height - profile_point[1]/self.terrain_resolution)
            DEM[y, x] += deformation_depth[i] * ((self.deform_decay_ratio)**multipass[y, x])
            multipass[y, x] += 1
        return DEM, multipass

class GenerateProceduralMoonYard:
    """
    Generates a random terrain DEM with craters."""

    def __init__(
        self,
        moon_yard: MoonYardConf,
    ) -> None:
        """
        Args:
            crater_profiles_path (str): path to the pickle file containing the spline profiles.
            x_size (float, optional): size of the DEM in the x direction (in meters). Defaults to 10.
            y_size (float, optional): size of the DEM in the y direction (in meters). Defaults to 6.5.
            resolution (float, optional): resolution of the DEM (in meters per pixel). Defaults to 0.01.
            max_elevation (float, optional): maximum elevation of the DEM (in meters). Defaults to 0.25.
            min_elevation (float, optional): minimum elevation of the DEM (in meters). Defaults to -0.025.
            z_scale (float, optional): scale of the DEM. Defaults to 1.
            pad (int, optional): size of the padding to add to the DEM. Defaults to 500.
            num_repeat (float, optional): number of times to repeat the hardcore rejection. Defaults to 0.
            densities (float, optional): densities of the craters (in units per square meters). Defaults to [0.025,0.05,0.5].
            radius (list, optional): min and max radii of the craters (in meters). Defaults to [(1.5,2.5),(0.75,1.5),(0.25,0.5)].
            is_lab (bool, optional): whether the DEM is in a lab or not. Defaults to False.
            is_yard (bool, optional): whether the DEM is in a yard or not. Defaults to False.
            seed (int, optional): random seed. Defaults to 42."""

        self.T = BaseTerrainGenerator(moon_yard.base_terrain_generator)
        self.D = Distribute(moon_yard.crater_distribution)
        self.G = CraterGenerator(moon_yard.crater_generator)
        self.is_lab = moon_yard.is_lab
        self.is_yard = moon_yard.is_yard
        
        self.DE = DeformationEngine(moon_yard.deformation_engine)
        self._dem_init = None
        self._mask = None
        self._dem_delta = None
        self._multipass = None
    
    def randomize(self) -> np.ndarray:
        DEM = self.T.generateRandomTerrain(is_lab=self.is_lab, is_yard=self.is_yard)
        mask = np.ones_like(DEM)
        coords, radius = self.D.run()
        DEM, mask = self.G.generateCraters(DEM, coords, radius)
        self._dem_init = DEM
        self._dem_delta = np.zeros_like(DEM)
        self._mask = mask
        self._multipass = np.zeros_like(mask)
        return DEM, mask
    
    def register_terrain(self, DEM:np.ndarray, mask:np.ndarray):
        self._dem_init = DEM
        self._dem_delta = np.zeros_like(DEM)
        self._mask = mask
        self._multipass = np.zeros_like(mask)
    
    def deform(self, body_transforms:np.ndarray, contact_forces:np.ndarray)-> np.ndarray:
        """
        Args:
            body_transforms(numpy.ndarray): body to world transform of rover's wheels (N, 4, 4)
            contact_forces(numpy.ndarray): contact forces of rover's wheels (N, 3)
        """
        self._dem_delta, self._multipass = self.DE.deform(
            self._dem_delta, self._multipass, body_transforms, contact_forces[:, 2])
        return self._dem_init + self._dem_delta, self._mask


if __name__ == "__main__":
    craters_profiles_path = "crater_spline_profiles.pkl"
    start = datetime.datetime.now()
    G = GenerateProceduralMoonYard(craters_profiles_path)
    DEMs = [G.randomize() for i in range(4)]
    end = datetime.datetime.now()

    print((end - start).total_seconds())
    for DEM, mask in DEMs:
        plt.figure()
        plt.imshow(DEM, cmap="jet")
        plt.figure()
        plt.imshow(mask, cmap="jet")
    plt.show()