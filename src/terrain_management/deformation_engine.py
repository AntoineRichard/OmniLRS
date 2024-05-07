__author__ = "Junnosuke Kamohara"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple
from matplotlib import pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

from src.configurations.procedural_terrain_confs import (
    FootprintConf, 
    DeformConstrainConf,
    DepthDistributionConf,
    BoundaryDistributionConf,
    DeformationEngineConf,
)

class FootprintProfileGenerator:
    def __init__(
            self, 
            footprint_conf:FootprintConf, 
            deform_constrain_conf:DeformConstrainConf, 
            terrain_resolution:float
            )-> None:
        """
        Footprint generator class.
        Args:
            footprint_conf (FootprintConf): footprint configuration.
            deform_constrain_conf (DeformConstrainConf): deformation constrain configuration.
            terrain_resolution (float): terrain resolution in meter/pixel.
        """
        self.terrain_resolution = terrain_resolution
        self.profile_width = footprint_conf.width
        self.profile_height = footprint_conf.height
        self.x_deform_offset = deform_constrain_conf.x_deform_offset
        self.y_deform_offset = deform_constrain_conf.x_deform_offset
        self.profile = None

    def create_profile(self)->Tuple[np.ndarray, int, int]:
        """
        Create footprint profile.
        Returns:
            profile (np.ndarray): footprint profile in local frame.
            profile_px_width (int): profile width in pixel.
            profile_px_height (int): profile height in pixel.
        """
        x = np.linspace(-self.profile_height/2, self.profile_height/2, int(self.profile_height/self.terrain_resolution)+1) + self.x_deform_offset
        y = np.linspace(-self.profile_width/2, self.profile_width/2, int(self.profile_width/self.terrain_resolution)+1) + self.y_deform_offset
        xx, yy = np.meshgrid(x, y)
        self.profile = np.column_stack([xx.flatten(), yy.flatten()])
        self.profile_px_width = xx.shape[0]
        self.profile_px_height = yy.shape[1]
        return self.profile, self.profile_px_width, self.profile_px_height
    
    def plot_profile(self)->None:
        """
        Plot footprint profile.
        """
        plt.scatter(self.profile[:, 0], self.profile[:, 1])
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("Footprint Profile in local frame.")

### Depth Distribution Generators ###
class DepthDistributionGenerator:
    """
    Base depth distribution generator class.
    """
    def __init__(
            self, 
            depth_distribution:DepthDistributionConf,
            footprint: FootprintConf,
            footprint_profile_width:int,
            footprint_profile_height:int,
            )-> None:
        """
        Args:
            depth_distribution (DepthDistributionConf): depth distribution configuration.
            footprint (FootprintConf): footprint configuration.
            footprint_profile_width (int): footprint profile width in pixel.
            footprint_profile_height (int): footprint profile height in pixel.
        """
        self.depth_distribution = depth_distribution
        self.footprint = footprint
        self.footprint_profile_width = footprint_profile_width
        self.footprint_profile_height = footprint_profile_height
    
    def get_depth_distribution_yslice(self)-> np.ndarray:
        """
        Get depth distribution on x axis.
        Returns:
            depth_dist_x (np.ndarray): depth distribution on x axis."""
        raise NotImplementedError
    
    def get_depth_distribution(self)-> np.ndarray:
        """
        Get depth distribution.
        Returns:
            depth_dist (np.ndarray): depth distribution.
        """
        raise NotImplementedError
    
    def plot_depth_distribution(self):
        """
        Plot depth distribution.
        """
        raise NotImplementedError

class UniformDepthDistributionGenerator(DepthDistributionGenerator):
    """
    Uniform depth distribution generator class.
    """
    def __init__(
            self, 
            depth_distribution:DepthDistributionConf,
            footprint: FootprintConf,
            footprint_profile_width:int,
            footprint_profile_height:int,
            )-> None:
        """
        Args:
            depth_distribution (DepthDistributionConf): depth distribution configuration.
            footprint (FootprintConf): footprint configuration.
            footprint_profile_width (int): footprint profile width in pixel.
            footprint_profile_height (int): footprint profile height in pixel.
        """
        super().__init__(depth_distribution, footprint, footprint_profile_width, footprint_profile_height)
    
    def get_depth_distribution_yslice(self)-> np.ndarray:
        """
        Get depth distribution on x axis.
        Returns:
            depth_dist_x (np.ndarray): depth distribution on x axis
        """
        return np.ones(self.footprint_profile_height, dtype=np.float32)
    
    def get_depth_distribution(self) -> np.ndarray:
        """
        Get depth distribution.
        Returns:
            depth_dist (np.ndarray): depth distribution.
        """
        depth_dist_x = self.get_depth_distribution_yslice()
        depth_dist = depth_dist_x[None, :].repeat(self.footprint_profile_width, axis=0).reshape(-1) # clone depth_x to y axis
        self.depth_dist = depth_dist
        return depth_dist
    
    def plot_depth_distribution(self):
        """
        Plot depth distribution.
        """
        x = (self.footprint.height/2) * np.linspace(-1, 1, self.footprint_profile_height)
        y = self.get_depth_distribution_yslice()
        plt.plot(x, y)
        plt.xlabel("x [m]")
        plt.ylabel("depth mask")
        plt.title("XZ cross section distribution")

class SinusoidalDepthDistributionGenerator(DepthDistributionGenerator):
    """
    Sinusoidal depth distribution generator class.
    """
    def __init__(
            self, 
            depth_distribution:DepthDistributionConf,
            footprint: FootprintConf,
            footprint_profile_width:int,
            footprint_profile_height:int,
            )-> None:
        """
        Args:
            depth_distribution (DepthDistributionConf): depth distribution configuration.
            footprint (FootprintConf): footprint configuration.
            footprint_profile_width (int): footprint profile width in pixel.
            footprint_profile_height (int): footprint profile height in pixel.
        """
        super().__init__(depth_distribution, footprint, footprint_profile_width, footprint_profile_height)
    
    def get_depth_distribution_yslice(self)-> np.ndarray:
        """
        Get depth distribution on x axis.
        Returns:
            depth_dist_x (np.ndarray): depth distribution on x axis
        """
        return np.cos(self.depth_distribution.wave_frequency*np.pi * np.linspace(-1, 1, self.footprint_profile_height)) # create depth distribution on x axis
    
    def get_depth_distribution(self) -> np.ndarray:
        """
        Get depth distribution.
        Returns:
            depth_dist (np.ndarray): depth distribution.
        """
        depth_dist_x = self.get_depth_distribution_yslice()
        depth_dist = depth_dist_x[None, :].repeat(self.footprint_profile_width, axis=0).reshape(-1) # clone depth_x to y axis
        self.depth_dist = depth_dist
        return depth_dist
    
    def plot_depth_distribution(self):
        """
        Plot depth distribution.
        """
        x = (self.footprint.height/2) * np.linspace(-1, 1, self.footprint_profile_height)
        y = self.get_depth_distribution_yslice()
        plt.plot(x, y)
        plt.xlabel("x [m]")
        plt.ylabel("depth mask")
        plt.title("XZ cross section distribution")

class TrapezoidalDepthDistributionGenerator(DepthDistributionGenerator):
    """
    Trapezoidal depth distribution generator class.
    """
    def __init__(
            self, 
            depth_distribution:DepthDistributionConf,
            footprint: FootprintConf,
            footprint_profile_width:int,
            footprint_profile_height:int,
            )-> None:
        """
        Args:
            depth_distribution (DepthDistributionConf): depth distribution configuration.
            footprint (FootprintConf): footprint configuration.
            footprint_profile_width (int): footprint profile width in pixel.
            footprint_profile_height (int): footprint profile height in pixel.
        """
        super().__init__(depth_distribution, footprint, footprint_profile_width, footprint_profile_height)
    
    def trapezoid_wave(self, period:float=1.0, amplitude:float=1.0)-> np.ndarray:
        """
        Generate trapezoidal waveform.
        Args:
            period (float): period of waveform.
            amplitude (float): amplitude of waveform.
        Returns:
            y (np.ndarray): waveform.
        """
        t = np.linspace(-1, 1, self.footprint_profile_height)
        # Calculate segments based on the period
        rise_time = 0.25 * period
        high_time = 0.25 * period
        fall_time = 0.25 * period
        low_time = 0.25 * period

        # Shift t by half a period to center the waveform around x=0
        t_shifted = t + 0.5 * period
        
        # Initialize the waveform value
        y = np.zeros_like(t_shifted)
        
        # Compute the waveform
        for i, ti in enumerate(t_shifted):
            if ti % period < rise_time:
                # Rising edge
                y[i] = (ti % period) / rise_time * amplitude
            elif ti % period < rise_time + high_time:
                # High level
                y[i] = amplitude
            elif ti % period < rise_time + high_time + fall_time:
                # Falling edge
                y[i] = amplitude - ((ti % period - rise_time - high_time) / fall_time * amplitude)
            else:
                # Low level
                y[i] = 0
                
        return y
   
    def get_depth_distribution_yslice(self)-> np.ndarray:
        """
        Get depth distribution on x axis.
        Returns:
            depth_dist_x (np.ndarray): depth distribution on x axis
        """
        return -1 + self.trapezoid_wave(2/self.depth_distribution.wave_frequency, 2) # create depth distribution on x axis
    
    def get_depth_distribution(self) -> np.ndarray:
        """
        Get depth distribution.
        Returns:
            depth_dist (np.ndarray): depth distribution.
        """
        depth_dist_x = self.get_depth_distribution_yslice()
        depth_dist = depth_dist_x[None, :].repeat(self.footprint_profile_width, axis=0).reshape(-1) # clone depth_x to y axis
        self.depth_dist = depth_dist
        return depth_dist
    
    def plot_depth_distribution(self):
        """
        Plot depth distribution.
        """
        x = (self.footprint.height/2) * np.linspace(-1, 1, self.footprint_profile_height)
        y = self.get_depth_distribution_yslice()
        plt.plot(x, y)
        plt.xlabel("x [m]")
        plt.ylabel("depth mask")
        plt.title("XZ cross section of depth distribution")

### Boundary Distribution Generators ###

class BoundaryDistributionGenerator:
    """
    Base boundary distribution generator class.
    """
    def __init__(
            self, 
            boundary_distribution:BoundaryDistributionConf,
            footprint: FootprintConf, 
            footprint_profile_width:int,
            footprint_profile_height:int,
            )-> None:
        """
        Args:
            boundary_distribution (BoundaryDistributionConf): boundary distribution configuration.
            footprint (FootprintConf): footprint configuration.
            footprint_profile_width (int): footprint profile width in pixel.
            footprint_profile_height (int): footprint profile height in pixel.
        """
        self.boundary_distribution = boundary_distribution
        self.footprint = footprint
        self.footprint_profile_width = footprint_profile_width
        self.footprint_profile_height = footprint_profile_height
    
    def get_boundary_distribution_xslice(self)-> np.ndarray:
        """
        Get boundary distribution on x axis.
        Returns:
            boundary_dist_x (np.ndarray): boundary distribution on x axis.
        """
        raise NotImplementedError
    
    def get_boundary_distribution(self)-> np.ndarray:
        """
        Get boundary distribution.
        Returns:
            boundary_dist (np.ndarray): boundary distribution.
        """
        raise NotImplementedError
    
    def plot_boundary_distribution(self):
        """
        Plot boundary distribution.
        """
        raise NotImplementedError

class UniformBoundaryDistributionGenerator(BoundaryDistributionGenerator):
    """
    Uniform boundary distribution generator class.
    """
    def __init__(
            self, 
            boundary_distribution:BoundaryDistributionConf,
            footprint: FootprintConf,
            footprint_profile_width:int,
            footprint_profile_height:int,
            )-> None:
        """
        Args:
            boundary_distribution (BoundaryDistributionConf): boundary distribution configuration.
            footprint (FootprintConf): footprint configuration.
            footprint_profile_width (int): footprint profile width in pixel.
            footprint_profile_height (int): footprint profile height in pixel."""
        super().__init__(boundary_distribution, footprint, footprint_profile_width, footprint_profile_height)
    
    def get_boundary_distribution_xslice(self)-> np.ndarray:
        """
        Get boundary distribution on x axis.
        Returns:
            boundary_dist_x (np.ndarray): boundary distribution on x axis.
        """
        return -np.ones(self.footprint_profile_width, dtype=np.float32)
    
    def get_boundary_distribution(self) -> np.ndarray:
        """
        Get boundary distribution.
        Returns:
            boundary_dist (np.ndarray): boundary distribution.
        """
        boundary_dist_y = self.get_boundary_distribution_xslice()
        boundary_dist = boundary_dist_y[None, :].repeat(self.footprint_profile_height, axis=1).reshape(-1) # clone boundary_y to x axis
        self.boundary_dist = boundary_dist
        return boundary_dist
    
    def plot_boundary_distribution(self):
        """
        Plot boundary distribution.
        """
        x = (self.footprint.width/2) * np.linspace(-1, 1, self.footprint_profile_width)
        y = self.get_boundary_distribution_xslice()
        plt.plot(x, y)
        plt.xlabel("y [m]")
        plt.ylabel("depth mask")
        plt.title("YZ cross section distribution")

class ParabolicBoundaryDistributionGenerator(BoundaryDistributionGenerator):
    """
    Parabolic boundary distribution generator class.
    """
    def __init__(
            self, 
            boundary_distribution:BoundaryDistributionConf,
            footprint: FootprintConf,
            footprint_profile_width:int,
            footprint_profile_height:int,
            )-> None:
        """
        Args:
            boundary_distribution (BoundaryDistributionConf): boundary distribution configuration.
            footprint (FootprintConf): footprint configuration.
            footprint_profile_width (int): footprint profile width in pixel.
            footprint_profile_height (int): footprint profile height in pixel.
        """
        super().__init__(boundary_distribution, footprint, footprint_profile_width, footprint_profile_height)
    
    def get_boundary_distribution_xslice(self)-> np.ndarray:
        """
        Get boundary distribution on x axis.
        Returns:
            boundary_dist_x (np.ndarray): boundary distribution on x axis.
        """
        y = np.linspace(-1, 1, self.footprint_profile_width)
        return y**2 - 1
    
    def get_boundary_distribution(self) -> np.ndarray:
        """
        Get boundary distribution.
        Returns:
            boundary_dist (np.ndarray): boundary distribution.
        """
        boundary_dist_y = self.get_boundary_distribution_xslice()
        boundary_dist = boundary_dist_y[None, :].repeat(self.footprint_profile_height, axis=1).reshape(-1) # clone boundary_y to x axis
        self.boundary_dist = boundary_dist
        return boundary_dist
    
    def plot_boundary_distribution(self):
        """
        Plot boundary distribution.
        """
        x = (self.footprint.width/2) * np.linspace(-1, 1, self.footprint_profile_width)
        y = self.get_boundary_distribution_xslice()
        plt.plot(x, y)
        plt.xlabel("y [m]")
        plt.ylabel("depth mask")
        plt.title("YZ cross section distribution")

class TrapezoidalBoundaryDistributionGenerator(BoundaryDistributionGenerator):
    """
    Trapezoidal boundary distribution generator class.
    """
    def __init__(
            self, 
            boundary_distribution:BoundaryDistributionConf,
            footprint: FootprintConf,
            footprint_profile_width:int,
            footprint_profile_height:int,
            )-> None:
        """
        Args:
            boundary_distribution (BoundaryDistributionConf): boundary distribution configuration.
            footprint (FootprintConf): footprint configuration.
            footprint_profile_width (int): footprint profile width in pixel.
            footprint_profile_height (int): footprint profile height in pixel.
        """
        super().__init__(boundary_distribution, footprint, footprint_profile_width, footprint_profile_height)
    
    def get_boundary_distribution_xslice(self)-> np.ndarray:
        """
        Get boundary distribution on x axis.
        Returns:
            boundary_dist_x (np.ndarray): boundary distribution on x axis.
        """
        tan = np.tan(self.boundary_distribution.angle_of_repose)
        y = np.linspace(-1, 1, self.footprint_profile_width)
        mask = (np.abs(y) >= 1 - (1/tan)).astype(np.float32)
        return mask * (tan * np.abs(y) - tan + 1) - 1
    
    def get_boundary_distribution(self) -> np.ndarray:
        """
        Get boundary distribution.
        Returns:
            boundary_dist (np.ndarray): boundary distribution.
        """
        boundary_dist_y = self.get_boundary_distribution_xslice()
        boundary_dist = boundary_dist_y[None, :].repeat(self.footprint_profile_height, axis=1).reshape(-1) # clone boundary_y to x axis
        self.boundary_dist = boundary_dist
        return boundary_dist
    
    def plot_boundary_distribution(self):
        """
        Plot boundary distribution.
        """
        x = (self.footprint.width/2) * np.linspace(-1, 1, self.footprint_profile_width)
        y = self.get_boundary_distribution_xslice()
        plt.plot(x, y)
        plt.xlabel("y [m]")
        plt.ylabel("depth mask")
        plt.title("YZ cross section distribution")

class DeformationEngine:
    """
    Terrain deformation engine class.
    """
    def __init__(self, deformation_engine:DeformationEngineConf)-> None:
        """
        Args:
            deformation_engine (DeformationEngineConf): deformation engine configuration.
        """
        self.terrain_resolution = deformation_engine.terrain_resolution
        self.terrain_width = deformation_engine.terrain_width
        self.terrain_height = deformation_engine.terrain_height
        self.sim_width = self.terrain_width / self.terrain_resolution
        self.sim_height = self.terrain_height / self.terrain_resolution
        
        self.footprint = deformation_engine.footprint
        self.deform_constrain = deformation_engine.deform_constrain
        self.depth_distribution = deformation_engine.depth_distribution
        self.boundary_distribution = deformation_engine.boundary_distribution
        self.force_depth_regression = deformation_engine.force_depth_regression

        # generate footprint profile 
        self.footprint_profile_generator = FootprintProfileGenerator(
            footprint_conf=deformation_engine.footprint,
            deform_constrain_conf=deformation_engine.deform_constrain,
            terrain_resolution=self.terrain_resolution
        )
        self.profile, self.footprint_profile_width, self.footprint_profile_height = self.footprint_profile_generator.create_profile()

        # generate depth distribution
        if self.depth_distribution.distribution == "uniform":
            self.depth_distribution_generator = UniformDepthDistributionGenerator(
                depth_distribution=self.depth_distribution,
                footprint=self.footprint, 
                footprint_profile_width=self.footprint_profile_width,
                footprint_profile_height=self.footprint_profile_height
            )
            self.depth_dist = self.depth_distribution_generator.get_depth_distribution()

        elif self.depth_distribution.distribution == "sinusoidal":
            self.depth_distribution_generator = SinusoidalDepthDistributionGenerator(
                depth_distribution=self.depth_distribution,
                footprint=self.footprint,
                footprint_profile_width=self.footprint_profile_width,
                footprint_profile_height=self.footprint_profile_height
            )
            self.depth_dist = self.depth_distribution_generator.get_depth_distribution()
        
        elif self.depth_distribution.distribution == "trapezoidal":
            self.depth_distribution_generator = TrapezoidalDepthDistributionGenerator(
                depth_distribution=self.depth_distribution,
                footprint=self.footprint,
                footprint_profile_width=self.footprint_profile_width,
                footprint_profile_height=self.footprint_profile_height
            )
            self.depth_dist = self.depth_distribution_generator.get_depth_distribution()
        
        # generate boundary distribution
        if self.boundary_distribution.distribution == "uniform":
            self.boundary_distribution_generator = UniformBoundaryDistributionGenerator(
                boundary_distribution=self.boundary_distribution,
                footprint=self.footprint,
                footprint_profile_width=self.footprint_profile_width,
                footprint_profile_height=self.footprint_profile_height
            )
            self.boundary_dist = self.boundary_distribution_generator.get_boundary_distribution()

        elif self.boundary_distribution.distribution == "parabolic":
            self.boundary_distribution_generator = ParabolicBoundaryDistributionGenerator(
                boundary_distribution=self.boundary_distribution,
                footprint=self.footprint,
                footprint_profile_width=self.footprint_profile_width,
                footprint_profile_height=self.footprint_profile_height
            )
            self.boundary_dist = self.boundary_distribution_generator.get_boundary_distribution()

        elif self.boundary_distribution.distribution == "trapezoidal":
            self.boundary_distribution_generator = TrapezoidalBoundaryDistributionGenerator(
                boundary_distribution=self.boundary_distribution,
                footprint=self.footprint,
                footprint_profile_width=self.footprint_profile_width,
                footprint_profile_height=self.footprint_profile_height
            )
            self.boundary_dist = self.boundary_distribution_generator.get_boundary_distribution()

    def get_footprint_profile_in_global(self, body_transforms:np.ndarray)-> np.ndarray:
        """
        Get x, y positions of profile in global coordinate.
        Args:
            body_transforms (np.ndarray): body transform of robot's links (n, 4, 4)
            ---
            n is the number of links.
        Returns:
            projection_points (np.ndarray): projected pixel coordinates of robot's links (num_points, 2)
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
    
    def force_depth_regression_model(self, normal_forces:np.ndarray)-> np.ndarray:
        """
        Calculate deformation depth from force distribution.
        Args:
            normal_forces (np.ndarray): contact force fields (n,)
        Returns:
            amplitude (np.ndarray): deformation depth amplitude (n,)
            mean_value (np.ndarray): mean deformation depth (n,)
        """
        amplitude = self.force_depth_regression.amplitude_slope * normal_forces + \
            self.force_depth_regression.amplitude_intercept
        mean_value = self.force_depth_regression.mean_slope * normal_forces + \
            self.force_depth_regression.mean_intercept
        return amplitude, mean_value
    
    def get_deformation_depth(self, normal_forces:np.ndarray)-> np.ndarray:
        """
        Calculate deformation depth from force distribution.
        Args:
            normal_forces (np.ndarray): contact force fields (n,)
        Returns:
            depth (np.ndarray): deformation depth (num_points, 1)
            ---
            num_points = n * num_point_sample
        """
        depth = []
        for normal_force in normal_forces:
            amplitude, mean_value = self.force_depth_regression_model(normal_force)
            depth.append(
                self.boundary_dist * (amplitude * self.depth_dist - mean_value)
            )
        depth = np.concatenate(depth)
        return depth
    
    def deform(self, DEM:np.ndarray, num_pass:np.ndarray, body_transforms:np.ndarray, normal_forces:np.ndarray)-> np.ndarray:
        """
        Args:
            DEM (np.ndarray): DEM to deform
            body_transforms (np.ndarray): projected coordinates of robot's link (n, 4, 4)
            normal_forces (np.ndarray): (dynamic) contact forces applied on robot's link (n, 3)
            ---
            n is the number of wheels.
        Returns:
            DEM (np.ndarray): deformed DEM
            num_pass (np.ndarray): number of passes on each pixel
        """
        profile_points = self.get_footprint_profile_in_global(body_transforms=body_transforms)
        deformation_depth = self.get_deformation_depth(normal_forces=normal_forces)
        for i, profile_point in enumerate(profile_points):
            x = int(profile_point[0]/self.terrain_resolution)
            y = int(self.sim_height - profile_point[1]/self.terrain_resolution)
            DEM[y, x] += deformation_depth[i] * ((self.deform_constrain.deform_decay_ratio)**num_pass[y, x])
            num_pass[y, x] += 1
        return DEM, num_pass

if __name__ == '__main__':
    # Confs
    footprint_conf = FootprintConf(width=0.1, height=0.18)
    deform_constrain_conf = DeformConstrainConf(
        horizontal_deform_offset=0., 
        vertical_deform_offset=0.0, 
        deform_decay_ratio=0.01
    )
    depth_distribution_conf = DepthDistributionConf(
        distribution="sinusoidal",
        wave_frequency=4.14
    )
    boundary_distribution_conf = BoundaryDistributionConf(
        distribution="trapezoidal",
        angle_of_repose=1.047
    )
    
    # Objects
    footprint = FootprintProfileGenerator(
        footprint_conf, 
        deform_constrain_conf, 
        0.001)
    
    footprint_profile, footprint_width, footprint_height = footprint.create_profile()
    
    # Depth distribution
    uniform_depth_distribution = UniformDepthDistributionGenerator(
        depth_distribution_conf,
        footprint_conf, 
        footprint_width,
        footprint_height
    )
    sinusoidal_depth_distribution = SinusoidalDepthDistributionGenerator(
        depth_distribution_conf,
        footprint_conf, 
        footprint_width,
        footprint_height
    )
    trapezoidal_depth_distribution = TrapezoidalDepthDistributionGenerator(
        depth_distribution_conf,
        footprint_conf, 
        footprint_width,
        footprint_height
    )
    
    # Boundary distribution
    uniform_boundary_distribution = UniformBoundaryDistributionGenerator(
        boundary_distribution_conf,
        footprint_conf, 
        footprint_width,
        footprint_height
    )
    
    parabolic_boundary_distribution = ParabolicBoundaryDistributionGenerator(
        boundary_distribution_conf,
        footprint_conf, 
        footprint_width,
        footprint_height
    )
    
    trapezoidal_boundary_distribution = TrapezoidalBoundaryDistributionGenerator(
        boundary_distribution_conf,
        footprint_conf, 
        footprint_width,
        footprint_height
    )
    
    # # Plot footprint
    # footprint.plot_profile()
    
    # Plot depth distribution
    plt.figure(figsize=(10, 5))
    uniform_depth_distribution.plot_depth_distribution()
    sinusoidal_depth_distribution.plot_depth_distribution()
    trapezoidal_depth_distribution.plot_depth_distribution()
    plt.legend(["uniform", "sinusoidal", "trapezoidal"], loc="best")
    plt.show()
    
    # Plot boundary distribution 
    plt.figure(figsize=(10, 5))
    uniform_boundary_distribution.plot_boundary_distribution()
    parabolic_boundary_distribution.plot_boundary_distribution()
    trapezoidal_boundary_distribution.plot_boundary_distribution()
    plt.legend(["uniform", "parabolic", "trapezoidal"], loc="best")
    plt.show()