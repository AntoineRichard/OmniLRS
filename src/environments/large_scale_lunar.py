__author__ = "Antoine Richard"
__copyright__ = "Copyright 2024, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple, Union, Dict
import omni.kit.actions.core
import numpy as np
import math
import omni
import carb

from omni.isaac.core.utils.stage import add_reference_to_stage

from pxr import UsdGeom, UsdLux, Gf, Usd

from src.configurations.stellar_engine_confs import StellarEngineConf
from src.configurations.rendering_confs import FlaresConf
from src.stellar.stellar_engine import StellarEngine
from src.physics.terramechanics_parameters import RobotParameter, TerrainMechanicalParameter
from src.physics.terramechanics_solver import TerramechanicsSolver
from WorldBuilders.pxr_utils import setDefaultOps
from assets import get_assets_path


class LargeScaleLunarController:
    """
    This class is used to control the lab interactive elements.
    """

    def __init__(
        self,
        lunaryard_settings: LunaryardConf = None,
        rocks_settings: Dict = None,
        flares_settings: FlaresConf = None,
        stellar_engine_settings: StellarEngineConf = None,
        **kwargs,
    ) -> None:
        """
        Initializes the lab controller. This class is used to control the lab interactive elements.
        Including:
            - Sun position, intensity, radius, color.
            - Terrains randomization or using premade DEMs.
            - Rocks random placements.

        Args:
            lunaryard_settings (LunaryardConf): The settings of the lab.
            rocks_settings (Dict): The settings of the rocks.
            flares_settings (FlaresConf): The settings of the flares.
            terrain_manager (TerrainManagerConf): The settings of the terrain manager.
            stellar_engine_settings (StellarEngineConf): The settings of the stellar engine.
            **kwargs: Arbitrary keyword arguments.
        """

        self.stage = omni.usd.get_context().get_stage()
        self.stage_settings = lunaryard_settings
        self.flare_settings = flares_settings
        if stellar_engine_settings is not None:
            self.SE = StellarEngine(stellar_engine_settings)
            self.enable_stellar_engine = True
        else:
            self.enable_stellar_engine = False
        self.TS = TerramechanicsSolver(
            robot_param=RobotParameter(),
            terrain_param=TerrainMechanicalParameter(),
        )
        self.dem = None
        self.mask = None
        self.scene_name = "/Lunaryard"
        self.deformation_conf = terrain_manager.moon_yard.deformation_engine

    def load(self) -> None:
        """
        Loads the lab interactive elements in the stage.
        Creates the instancer for the rocks, and generates the terrain.
        """

        scene_path = get_assets_path() + "/USD_Assets/environments/Lunaryard.usd"
        # Loads the Lunalab
        add_reference_to_stage(scene_path, self.scene_name)
        # Fetches the interactive elements
        self.collectInteractiveAssets()
        # Generates the instancer for the rocks
        self.RM.build(self.dem, self.mask)
        # Loads the DEM and the mask
        self.switchTerrain(self.stage_settings.terrain_id)
        self.enableLensFlare(self.flare_settings.enable)
        if self.enable_stellar_engine:
            self.SE.setLatLon(self.stage_settings.coordinates.latitude, self.stage_settings.coordinates.longitude)

    def addRobotManager(self, robotManager):
        self.robotManager = robotManager

    def getLuxAssets(self, prim: "Usd.Prim") -> None:
        """
        Returns the UsdLux prims under a given prim.

        Args:
            prim (Usd.Prim): The prim to be searched.

        Returns:
            list: A list of UsdLux prims.
        """

        lights = []
        for prim in Usd.PrimRange(prim):
            if prim.IsA(UsdLux.DistantLight):
                lights.append(prim)
            if prim.IsA(UsdLux.SphereLight):
                lights.append(prim)
            if prim.IsA(UsdLux.CylinderLight):
                lights.append(prim)
            if prim.IsA(UsdLux.DiskLight):
                lights.append(prim)
        return lights

    def setAttributeBatch(self, prims: List["Usd.Prim"], attr: str, val: Union[float, int]) -> None:
        """
        Sets the value of an attribute for a list of prims.

        Args:
            prims (list): A list of prims.
            attr (str): The name of the attribute.
            val (Union[float,int]): The value to be set.
        """

        for prim in prims:
            prim.GetAttribute(attr).Set(val)

    def collectInteractiveAssets(self) -> None:
        """
        Collects the interactive assets from the stage and assigns them to class variables.
        """

        # Projector
        self._projector_prim = self.stage.GetPrimAtPath(self.stage_settings.projector_path)
        self._projector_xform = UsdGeom.Xformable(self._projector_prim)
        self._projector_lux = self.getLuxAssets(self._projector_prim)

        # Earth
        self._earth_prim = self.stage.GetPrimAtPath(self.stage_settings.earth_path)
        self._earth_xform = UsdGeom.Xformable(self._earth_prim)

    # ==============================================================================
    # Stellar engine control
    # ==============================================================================

    def setCoordinates(self, latlong: Tuple[float, float]) -> None:
        """
        Sets the coordinates of the lab.

        Args:
            latlong (Tuple[float,float]): The latitude and longitude of the scene on the moon.
        """

        if self.enable_stellar_engine:
            self.SE.setLatLon(latlong[1], latlong[0])

    def setTime(self, time: float) -> None:
        """
        Sets the time of the stellar engine.

        Args:
            time (float): The time in seconds.
        """

        if self.enable_stellar_engine:
            self.SE.setTime(time)

    def setTimeScale(self, scale: float) -> None:
        """
        Sets the time scale of the stellar engine.

        Args:
            scale (float): The time scale.
        """

        if self.enable_stellar_engine:
            self.SE.setTimeScale(scale)

    def updateStellarEngine(self, dt: float) -> None:
        """
        Updates the sun and earth pose.

        Args:
            dt (float): The time step.
        """

        if self.enable_stellar_engine:
            update = self.SE.update(dt)
            if update:
                earth_pos = self.SE.getLocalPosition("earth")
                alt, az, dist = self.SE.getAltAz("sun")
                quat = self.SE.convertAltAzToQuat(alt, az)

                self.setSunPose(((0, 0, 0), quat))
                self.setEarthPose((earth_pos, (0, 0, 0, 1)))

    # ==============================================================================
    # Earth control
    # ==============================================================================

    def setEarthPose(self, pose: Tuple[List[float], List[float]]) -> None:
        """
        Sets the pose of the earth.

        Args:
            pose (Tuple[List[float],List[float]]): The pose of the earth. The first element is the position, the second is the quaternion.
                                                   The position is in meters, the quaternion is in (x,y,z,w) format.
        """

        position = pose[0]
        quat = pose[1]
        rotation = [quat[0], quat[1], quat[2], quat[3]]
        position = [position[0], position[1], position[2]]
        setDefaultOps(self._earth_xform, position, rotation, scale=(1, 1, 1))

    # ==============================================================================
    # Sun control
    # ==============================================================================
    def setSunPose(self, pose: Tuple[List[float], List[float]]) -> None:
        """
        Sets the pose of the sun.

        Args:
            pose (Tuple[List[float],List[float]]): The pose of the projector. The first element is the position, the second is the quaternion.
                                                   The position is in meters, the quaternion is in (x,y,z,w) format.
        """

        position = pose[0]
        quat = pose[1]
        rotation = [quat[0], quat[1], quat[2], quat[3]]
        position = [position[0], position[1], position[2]]

        setDefaultOps(self._projector_xform, position, rotation, scale=(1, 1, 1))

    def setSunIntensity(self, intensity: float) -> None:
        """
        Sets the intensity of the sun.

        Args:
            intensity (float): The intensity of the projector (arbitrary unit).
        """

        self.setAttributeBatch(self._projector_lux, "intensity", intensity)

    def setSunColor(self, color: List[float]) -> None:
        """
        Sets the color of the projector.

        Args:
            color (List[float]): The color of the projector (RGB).
        """

        color = Gf.Vec3d(color[0], color[1], color[2])
        self.setAttributeBatch(self._projector_lux, "color", color)

    # ==============================================================================
    # Render control
    # ==============================================================================
    def enableRTXRealTime(self, data: int = 0) -> None:
        """
        Enables the RTX real time renderer. The ray-traced render.

        Args:
            data (int, optional): Not used. Defaults to 0.
        """

        action_registry = omni.kit.actions.core.get_action_registry()
        action = action_registry.get_action("omni.kit.viewport.actions", "set_renderer_rtx_realtime")
        action.execute()

    def enableRTXInteractive(self, data: int = 0) -> None:
        """
        Enables the RTX interactive renderer. The path-traced render.

        Args:
            data (int, optional): Not used. Defaults to 0.
        """

        action_registry = omni.kit.actions.core.get_action_registry()
        action = action_registry.get_action("omni.kit.viewport.actions", "set_renderer_rtx_pathtracing")
        action.execute()

    def enableLensFlare(self, data: bool) -> None:
        """
        Enables the lens flare effect.

        Args:
            data (bool): True to enable the lens flare, False to disable it.
        """

        settings = carb.settings.get_settings()
        if data:
            settings.set("/rtx/post/lensFlares/enabled", True)
            self.setFlareScale(self.flare_settings.scale)
            self.setFlareNumBlades(self.flare_settings.blades)
            self.setFlareApertureRotation(self.flare_settings.aperture_rotation)
            self.setFlareSensorAspectRatio(self.flare_settings.sensor_aspect_ratio)
            self.setFlareSensorDiagonal(self.flare_settings.sensor_diagonal)
            self.setFlareFstop(self.flare_settings.fstop)
            self.setFlareFocalLength(self.flare_settings.focal_length)
        else:
            settings.set("/rtx/post/lensFlares/enabled", False)

    def setFlareScale(self, value: float) -> None:
        """
        Sets the scale of the lens flare.

        Args:
            value (float): The scale of the lens flare.
        """

        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/flareScale", value)

    def setFlareNumBlades(self, value: int) -> None:
        """
        Sets the number of blades of the lens flare.
        A small number will create sharp spikes, a large number will create a smooth circle.

        Args:
            value (int): The number of blades of the lens flare.
        """

        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/blades", int(value))

    def setFlareApertureRotation(self, value: float) -> None:
        """
        Sets the rotation of the lens flare.

        Args:
            value (float): The rotation of the lens flare.
        """

        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/apertureRotation", value)

    def setFlareSensorDiagonal(self, value: float) -> None:
        """
        Sets the sensor diagonal of the lens flare.

        Args:
            value (float): The sensor diagonal of the lens flare.
        """

        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/sensorDiagonal", value)

    def setFlareSensorAspectRatio(self, value: float) -> None:
        """
        Sets the sensor aspect ratio of the lens flare.

        Args:
            value (float): The sensor aspect ratio of the lens flare.
        """

        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/sensorAspectRatio", value)

    def setFlareFstop(self, value: float) -> None:
        """
        Sets the f-stop of the lens flare.

        Args:
            value (float): The f-stop of the lens flare.
        """

        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/fNumber", value)

    def setFlareFocalLength(self, value: float):
        """
        Sets the focal length of the lens flare.

        Args:
            value (float): The focal length of the lens flare.
        """

        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/focalLength", value)

    def deformTerrain(self) -> None:
        """
        Deforms the terrain.
        Args:
            world_poses (np.ndarray): The world poses of the contact points.
            contact_forces (np.ndarray): The contact forces in local frame reported by rigidprimview.
        """
        world_poses = []
        contact_forces = []
        for rrg in self.robotManager.robots_RG.values():
            world_poses.append(rrg.get_world_poses())
            contact_forces.append(rrg.get_net_contact_forces())
        world_poses = np.concatenate(world_poses, axis=0)
        contact_forces = np.concatenate(contact_forces, axis=0)

        self.T.deformTerrain(body_transforms=world_poses, contact_forces=contact_forces)
        self.loadDEM()
        self.RM.updateImageData(self.dem, self.mask)
