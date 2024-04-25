__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple, Union
import omni.kit.actions.core
import numpy as np
import omni
import carb
import cv2
import os

from omni.isaac.core.utils.stage import add_reference_to_stage

from pxr import UsdGeom, UsdLux, Gf, Usd

from WorldBuilders.pxr_utils import setTransform, getTransform, createInstancerAndCache, setInstancerParameters
from src.terrain_management.terrain_manager import TerrainManager
from WorldBuilders.Mixer import *

# ==============================================================================
#                               LAB GLOBAL VARIABLES
# ==============================================================================

WORKINGDIR = os.path.dirname(__file__)

# USD path of the lab interactive elements
PROJECTOR_PATH = "/Lunaryard/Sun"

# Default lab size
LAB_X_MIN = 0.5
LAB_X_MAX = 976*3 - 0.5
LAB_Y_MIN = 0.5
LAB_Y_MAX = 976*3 - 0.5

# Default terrain resolution
MPP = 3.0
# Default rock foot print
ROCK_FOOT_PRINT = 0.1

# Default lens flare parameters
FLARE = {"Scale":0.43,
         "Blades":3,
         "ApertureRotation":0.0,
         "SensorDiagonal":22.0,
         "SensorAspectRatio":2.81,
         "Fstop":0.3,
         "FocalLength":1.0}

terrain_settings = {"crater_spline_profiles": "crater_spline_profiles.pkl",
                    "dems_path": "Terrains/Abel_E_crater",
                    "sim_length": LAB_Y_MAX - LAB_Y_MIN + 1,
                    "sim_width": LAB_X_MAX - LAB_X_MIN + 1,
                    "lab_x_min": LAB_X_MIN,
                    "lab_x_max": LAB_X_MAX,
                    "lab_y_min": LAB_Y_MIN,
                    "lab_y_max": LAB_Y_MAX,
                    "resolution": MPP,
                    "mesh_pos": (0.0, 0.0, 0.0),
                    "mesh_rot": (0.0, 0.0, 0.0, 1.0),
                    "mesh_scale": (1.0, 1.0, 1.0),
                    "max_elevation": 0.5,
                    "min_elevation": -0.5,
                    "z_scale": 1,
                    "pad":500,
                    "num_repeat":0,
                    "densities": [0.025, 0.05, 0.5],
                    "radius": [(1.5,2.5), (0.75,1.5), (0.25,0.5)],
                    "root_path": "/Lunaryard",
                    "texture_path": "/Lunaryard/Looks/Basalt",
                    "seed": 42,
                    "is_yard": True,
                    "is_lab": False}

# ==============================================================================

class LabController:
    """
    This class is used to control the lab interactive elements."""

    def __init__(self, terrain_settings: dict = terrain_settings) -> None:
        """
        Initializes the lab controller. This class is used to control the lab interactive elements.
        Including:
            - Projector position, intensity, radius, color.
            - Room lights intensity, radius, color.
            - Curtains open or closed.
            - Terrains randomization or using premade DEMs.
            - Rocks random placements.
            
        Args:
            terrain_settings (dict): The settings for the terrain manager."""
        
        self.stage = omni.usd.get_context().get_stage()
        self.terrain_settings = terrain_settings
        self.T = TerrainManager(**terrain_settings)
        self.active_dem = None
        self.active_mask = None
        self.dem = None
        self.mask = None
        self.mixer = None
        self.scene_name = "/Lunaryard"

    def load(self) -> None:
        """
        Loads the lab interactive elements in the stage.
        Creates the instancer for the rocks, and generates the terrain."""

        scene_path = WORKINGDIR+"/Lunaryard.usd"
        # Loads the Lunalab
        add_reference_to_stage(scene_path, self.scene_name)
        # Fetches the interactive elements
        self.collectInteractiveAssets()
        # Generates the instancer for the rocks
        self.createRockInstancer()
        # Loads the DEM and the mask
        self.switchTerrain(0)
        # Setups the auto labeling
        self.autoLabel()
    
    def createRockInstancer(self) -> None:
        """
        Creates the instancer for the rocks."""

        # Creates a list of pathes of all the assets to be loaded
        root = WORKINGDIR+"/Assets/Rocks/Apollo_rocks"
        assets = os.listdir(root)
        assets = [[os.path.join(root, asset, i) for i in os.listdir(os.path.join(root,asset)) if i.split('.')[-1]=="usd"] for asset in assets]
        assets = [item for sublist in assets for item in sublist]
        # Creates the instancer
        self._rocks_prim = self.stage.DefinePrim(self.scene_name+"/Rocks", "Xform")
        self._rock_instancer = createInstancerAndCache(self.stage, self.scene_name+"/Rocks/instancer", assets)
        self._rocks_instancer_prim = self.stage.GetPrimAtPath(self.scene_name+"/Rocks/instancer")
        #self._rocks_prim.GetAttribute("visibility").Set("invisible")

    def createSampler(self) -> None:
        """
        Creates the sampler for the rocks."""

        H,W = self.dem.shape
        # Erodes the spawning masks such that the rocks do not spawn on forbidden regions.
        kernel_radius = 4
        kernel = np.zeros((9, 9), np.uint8)                                                                                      
        kernel = cv2.circle(kernel,(4,4),kernel_radius,1,-1)
        num_iterations = int((ROCK_FOOT_PRINT / 0.01) / kernel_radius)
        if num_iterations:
            self.mask = cv2.erode(self.mask, kernel, iterations=num_iterations)

        # Generates the requests to be sent to the procedural rock placement.
        # Positiion based on the mask
        xy_mask = Image_T(data=self.mask, mpp_resolution=self.terrain_settings["resolution"], output_space=2)
        xy_sampler = HardCoreUniformSampler_T(min=(self.terrain_settings["lab_x_min"], self.terrain_settings["lab_y_min"]),
                                              max=(self.terrain_settings["lab_x_max"], self.terrain_settings["lab_y_max"]),
                                              randomization_space=2, seed=42, core_radius=ROCK_FOOT_PRINT, num_repeat=2)
        xy_sampler = ThomasClusterSampler_T(lambda_parent=0.0001, lambda_daughter=100, sigma=0.2,
                                                 randomization_space=2, seed=42)
        req_pos_xy = UserRequest_T(p_type = Position_T(), sampler=xy_sampler, layer=xy_mask, axes=["x","y"])
        # Random yaw
        rpy_layer = RollPitchYaw_T(rmax=0, rmin=0, pmax=0, pmin=0, ymax=np.pi*2, ymin=0)
        rpy_sampler = UniformSampler_T(randomization_space=3, seed=42)
        req_ori = UserRequest_T(p_type = Orientation_T(), sampler=rpy_sampler, layer=rpy_layer, axes=["x", "y", "z", "w"])
        # DEM clipper
        image_layer = Image_T(output_space=1)
        image_clipper = ImageClipper_T(randomization_space=1, resolution=(H, W), mpp_resolution=self.terrain_settings["resolution"], data=self.dem)
        req_pos_z = UserRequest_T(p_type = Position_T(), sampler=image_clipper, layer=image_layer, axes=["z"])
        # Scale range and distribution
        uni = UniformSampler_T(randomization_space=1)
        line = Line_T(xmin=0.1, xmax=0.4)
        req_scale = UserRequest_T(p_type = Scale_T(), sampler=uni, layer=line, axes=["xyz"])
        # Builds an executation graph to apply the requests.
        requests = [req_pos_xy, req_pos_z, req_ori, req_scale]
        self.mixer = RequestMixer(requests)

    def getLuxAssets(self, prim: "Usd.Prim") -> None:
        """
        Returns the UsdLux prims under a given prim.
        
        Args:
            prim (Usd.Prim): The prim to be searched.
            
        Returns:
            list: A list of UsdLux prims."""

        lights = []
        for prim in Usd.PrimRange(prim):
            if prim.IsA(UsdLux.SphereLight):
                lights.append(prim)
            if prim.IsA(UsdLux.CylinderLight):   	
                lights.append(prim)
            if prim.IsA(UsdLux.DiskLight):   	
                lights.append(prim)
        return lights
    
    def setAttributeBatch(self, prims: List["Usd.Prim"], attr:str, val:Union[float,int]) -> None:
        """
        Sets the value of an attribute for a list of prims.
        
        Args:
            prims (list): A list of prims.
            attr (str): The name of the attribute.
            val (Union[float,int]): The value to be set."""
        
        for prim in prims:
            prim.GetAttribute(attr).Set(val)
    
    def loadDEM(self) -> None:
        """
        Loads the DEM and the mask from the TerrainManager."""

        self.dem = self.T.getDEM()
        self.mask = self.T.getMask()

    def autoLabel(self) -> None:
        """
        Automatically labels the rocks."""

        #for prim in Usd.PrimRange(self.stage.GetPrimAtPath("/Lunalab/Rocks/instancer/cache")):
            #if str(prim.GetPath()).split("/")[-1].split("_")[0] == "instance":
            #    prim_sd = PrimSemanticData(prim)
            #    prim_sd.add_entry("class", "rock")
        pass


    def collectInteractiveAssets(self) -> None:
        """
        Collects the interactive assets from the stage and assigns them to class variables."""

        # Projector
        self._projector_prim = self.stage.GetPrimAtPath(PROJECTOR_PATH)
        self._projector_xform = UsdGeom.Xformable(self._projector_prim)
        self._projector_lux = self.getLuxAssets(self._projector_prim)
        #self._projector_flare = self.stage.GetPrimAtPath(PROJECTOR_SHADER_PATH)

# ==============================================================================
# Sun control
# ==============================================================================
    def setSunPose(self, pose:Tuple[List[float],List[float]]) -> None:
        """
        Sets the pose of the sun.
        
        Args:
            pose (Tuple[List[float],List[float]]): The pose of the projector. The first element is the position, the second is the quaternion."""
        
        position = pose[0]
        quat = pose[1]
        rotation = Gf.Rotation(Gf.Vec3d(quat[0],quat[1],quat[2]), quat[3])
        position = Gf.Vec3d(position[0], position[1], position[2])
        transform = getTransform(rotation,position)
        setTransform(self._projector_xform, transform)

    def setSunIntensity(self, intensity: float) -> None:
        """
        Sets the intensity of the sun.
        
        Args:
            intensity (float): The intensity of the projector (arbitrary unit)."""
        
        self.setAttributeBatch(self._projector_lux, "intensity", intensity)

    def setSunColor(self, color: List[float]) -> None:
        """
        Sets the color of the projector.
        
        Args:
            color (List[float]): The color of the projector (RGB)."""
        
        color = Gf.Vec3d(color[0], color[1], color[2])
        self.setAttributeBatch(self._projector_lux, "color", color) 

# ==============================================================================
# Terrain control
# ==============================================================================
    def switchTerrain(self, flag:int) -> None:
        """
        Switches the terrain to a new DEM.
        
        Args:
            flag (int): The id of the DEM to be loaded. If negative, a random DEM is generated."""
        
        if flag < 0:
            self.T.randomizeTerrain()
        else:
            self.T.loadTerrainId(flag)
        self.loadDEM()
        self.createSampler()
        self.randomizeRocks(500)

    def enableRocks(self, flag: bool) -> None:
        """
        Turns the rocks on or off.
        
        Args:
            flag (bool): True to turn the rocks on, False to turn them off."""
        
        if flag:
            self._rocks_prim.GetAttribute("visibility").Set("visible")
        else:
            self._rocks_prim.GetAttribute("visibility").Set("invisible")

    def randomizeRocks(self, num:int=8) -> None:
        """
        Randomizes the placement of the rocks.
        
        Args:
            num (int): The number of rocks to be placed."""
        
        num = int(num)
        if num == 0:
            num += 1
        attributes = self.mixer.executeGraph(num)
        position = attributes["xformOp:translation"]
        scale = attributes["xformOp:scale"]
        orientation = attributes["xformOp:orientation"]
        setInstancerParameters(self.stage, self.scene_name+"/Rocks/instancer", pos=position, scale=scale, quat=orientation)

    def placeRocks(self, file_path) -> None:
        pass

# ==============================================================================
# Render control
# ==============================================================================
    def enableRTXRealTime(self, data:int=0) -> None:
        """
        Enables the RTX real time renderer. The ray-traced render.
        
        Args:
            data (int, optional): Not used. Defaults to 0."""
        
        action_registry = omni.kit.actions.core.get_action_registry()
        action = action_registry.get_action("omni.kit.viewport.actions", "set_renderer_rtx_realtime")
        action.execute()

    def enableRTXInteractive(self, data:int=0) -> None:
        """
        Enables the RTX interactive renderer. The path-traced render.
        
        Args:
            data (int, optional): Not used. Defaults to 0."""
        
        action_registry = omni.kit.actions.core.get_action_registry()
        action = action_registry.get_action("omni.kit.viewport.actions", "set_renderer_rtx_pathtracing")
        action.execute()

    def enableLensFlare(self, data:bool) -> None:
        """
        Enables the lens flare effect.
        
        Args:
            data (bool): True to enable the lens flare, False to disable it."""
        
        settings = carb.settings.get_settings()
        if data:
            settings.set("/rtx/post/lensFlares/enabled", True)
            self.setFlareScale(FLARE["Scale"])
            self.setFlareNumBlades(FLARE["Blades"])
            self.setFlareApertureRotation(FLARE["ApertureRotation"])
            self.setFlareSensorAspectRatio(FLARE["SensorAspectRatio"])
            self.setFlareSensorDiagonal(FLARE["SensorDiagonal"])
            self.setFlareFstop(FLARE["Fstop"])
            self.setFlareFocalLength(FLARE["FocalLength"])
        else:
            settings.set("/rtx/post/lensFlares/enabled", False)

    def setFlareScale(self, value:float) -> None:
        """
        Sets the scale of the lens flare.
        
        Args:
            value (float): The scale of the lens flare."""

        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/flareScale", value)

    def setFlareNumBlades(self, value:int) -> None:
        """
        Sets the number of blades of the lens flare.
        A small number will create sharp spikes, a large number will create a smooth circle.
        
        Args:
            value (int): The number of blades of the lens flare."""
        
        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/blades", int(value))

    def setFlareApertureRotation(self, value:float) -> None:
        """
        Sets the rotation of the lens flare.
        
        Args:
            value (float): The rotation of the lens flare."""
        
        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/apertureRotation", value)

    def setFlareSensorDiagonal(self, value:float) -> None:
        """
        Sets the sensor diagonal of the lens flare.
        
        Args:
            value (float): The sensor diagonal of the lens flare."""
        
        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/sensorDiagonal", value)

    def setFlareSensorAspectRatio(self, value:float) -> None:
        """
        Sets the sensor aspect ratio of the lens flare.
        
        Args:
            value (float): The sensor aspect ratio of the lens flare."""
        
        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/sensorAspectRatio", value)

    def setFlareFstop(self, value:float) -> None:
        """
        Sets the f-stop of the lens flare.
        
        Args:
            value (float): The f-stop of the lens flare."""

        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/fNumber", value)

    def setFlareFocalLength(self, value:float):
        """
        Sets the focal length of the lens flare.
        
        Args:
            value (float): The focal length of the lens flare."""

        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/focalLength", value)
