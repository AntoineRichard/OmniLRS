__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from WorldBuilders.Types import *
from WorldBuilders.Mixer import RequestMixer
from typing import Any
from WorldBuilders.pxr_utils import createInstancerAndCache, setInstancerParameters
from src.labeling.instancer import CustomInstancer
from assets import get_assets_path
import omni
import os


class TypeFactory:
    """
    TypeFactory class. It allows to register types and create objects of the registered types."""

    def __init__(self):
        self.types = {}
    
    def register_type(self, type_name:str,
                            worldbuilder_type,
                            ) -> None:
        """
        Register a type.
        
        Args:
            type_name (str): The name of the type.
            worldbuilder_type (WorldBuilder): The type of the worldbuilder."""
        
        self.types[type_name] = worldbuilder_type

    def __call__(self, name:str = None,
                       **kwargs,
                       ) -> Any:
        """
        Create an object of a specific type.
        
        Args:
            name (str, optional): The name of the type. Defaults to None.
            **kwargs: The arguments of the type."""
        
        if name is None:
            raise ValueError("Incorrect type name passed. Available types are: {}".format(self.types.keys()))
        else:
            return self.types[name](**kwargs)
        
samplerTypeFactory = TypeFactory()
samplerTypeFactory.register_type("Uniform", UniformSampler_T)
samplerTypeFactory.register_type("HardCoreUniform", HardCoreUniformSampler_T)
samplerTypeFactory.register_type("Normal", NormalSampler_T)
samplerTypeFactory.register_type("MaternCluster", MaternClusterPointSampler_T)
samplerTypeFactory.register_type("HardCoreMaternCluster", HardCoreMaternClusterPointSampler_T)
samplerTypeFactory.register_type("Poisson", PoissonPointSampler_T)
samplerTypeFactory.register_type("LinearInterpolation", LinearInterpolationSampler_T)
samplerTypeFactory.register_type("ThomasCluster", ThomasClusterSampler_T)
samplerTypeFactory.register_type("HardCoreThomasCluster", HardCoreThomasClusterSampler_T)
samplerTypeFactory.register_type("Image", ImageClipper_T)
samplerTypeFactory.register_type("Normal", NormalMapClipper_T)

attributeFactory = TypeFactory()
attributeFactory.register_type("Position", Position_T)
attributeFactory.register_type("Orientation", Orientation_T)
attributeFactory.register_type("Scale", Scale_T)

layerFactory = TypeFactory()
layerFactory.register_type("Line", Line_T)
layerFactory.register_type("Circle", Circle_T)
layerFactory.register_type("plane", Plane_T)
layerFactory.register_type("Disk", Disk_T)
layerFactory.register_type("Cube", Cube_T)
layerFactory.register_type("Cylinder", Cylinder_T)
layerFactory.register_type("Sphere", Sphere_T)
layerFactory.register_type("Cone", Cone_T)
layerFactory.register_type("Torus", Torus_T)
layerFactory.register_type("Image", Image_T)
layerFactory.register_type("NormalMap", NormalMap_T)
layerFactory.register_type("RollPitchYaw", RollPitchYaw_T)

def requestGenerator(request: dict) -> Any:
    attr = attributeFactory(request["attribute"])
    layer = layerFactory(**request["layer"])
    sampler = samplerTypeFactory(**request["sampler"])
    return UserRequest_T(p_type=attr, layer=layer, sampler=sampler, axes=request["axes"])

def getMixer(requests: dict) -> RequestMixer:
    return RequestMixer([requestGenerator(request) for request in requests.values()])

def addImageData(requests: dict, image: np.ndarray, mask: np.ndarray = None):
    for name in requests.keys():
        if requests[name]["layer"]["name"] == "Image":
            requests[name]["layer"]["data"] = mask
        if requests[name]["sampler"]["name"] == "Image":
            requests[name]["sampler"]["data"] = image
            requests[name]["sampler"]["resolution"] = image.shape[:2]
    return requests

def generateMixer(requests, image, mask):
    requests = addImageData(requests, image, mask)
    return getMixer(requests)

class OGInstancer:
    """
    The Original Gangster: the point instancer."""

    def __init__(self, instancer_path, asset_list, seed):
        self.instancer_path = instancer_path
        self.stage = omni.usd.get_context().get_stage()
        self.prototypes = asset_list
        createInstancerAndCache(self.stage, self.instancer_path, self.prototypes)
        self.rng = np.random.default_rng(seed=seed)
    
    def setInstanceParameter(self, position:np.ndarray,
                                   orientation:np.ndarray,
                                   scale: np.ndarray,
                                   **kwargs) -> None:
        """
        Set the instancer's parameters. It sets the position, orientation, and scale of the instances.
        
        Args:
            position (np.ndarray): The position of the instances.
            orientation (np.ndarray): The orientation of the instances.
            scale (np.ndarray): The scale of the instances.
            **kwargs: Extra arguments."""

        ids = self.rng.integers(0, len(self.prototypes), position.shape[0])
        setInstancerParameters(self.stage, self.instancer_path, position, quat=orientation, scale=scale, ids=ids, **kwargs)

class RockManager:
    def __init__(self, rocks_settings:dict = None, instancers_path: str = None, **kwargs):
        self.stage = omni.usd.get_context().get_stage()

        self.instancers = {}
        self.settings = {}
        self.mixers = {}

        self.instancers_path = instancers_path
        self.mappings = {"xformOp:translation": "position", "xformOp:orientation": "orientation", "xformOp:scale": "scale"}

        for name, settings in rocks_settings.items():
            self.settings[name] = settings

    def build(self, image: np.ndarray, mask: np.ndarray) -> None:
        """
        Builds the rock manager.
        It first creates the mixers, and then creates the instancers.
        Finally the mixers are exectuted, and the instancers are set to the randomized parameters.
        
        Args:
            image (np.ndarray): The image data.
            mask (np.ndarray): The mask data."""

        self.createInstancers()

    def updateImageData(self, image: np.ndarray, mask:np.ndarray) -> None:
        """
        Creates the mixers with the image and mask data loaded in.
        The image and mask are used to place assets in the scene.
        
        Args:
            image (np.ndarray): The image data.
            mask (np.ndarray): The mask data."""

        self.image = image
        for name, settings in self.settings.items():
            print(settings)
            self.mixers[name] = generateMixer(settings["requests"], image, mask)
    
    def createInstancers(self):
        """
        Creates as many instancers as there are types of rocks.
        There are two types of instancers, point instancers and custom instancers.
        The point instancers are the original instancers from USD.
        The custom instancers are instancers that enable the use of synthetic data generation."""

        for name, settings in self.settings.items():
            rock_assets = []
            for collection in settings["collections"]:
                root = get_assets_path()+"/USD_Assets/rocks/"+collection
                assets = os.listdir(root)
                assets = [[os.path.join(root, asset, i) for i in os.listdir(os.path.join(root,asset)) if i.split('.')[-1]=="usd"] for asset in assets]
                assets = [item for sublist in assets for item in sublist]
                rock_assets += assets

            self.stage.DefinePrim(self.instancers_path, "Xform")

            self.stage.DefinePrim(os.path.join(self.instancers_path,name), "Xform")
            if settings["use_point_instancer"]:
                self.instancers[name] = OGInstancer(os.path.join(self.instancers_path, name, "instancer"), rock_assets, seed=settings["seed"])
            else:
                self.instancers[name] = CustomInstancer(os.path.join(self.instancers_path, name, "instancer"), rock_assets, seed=settings["seed"])

    def randomizeInstancers(self, num):
        """
        Runs the mixers, collects the randomized parameters, and sets them to the instancers."""
        
        for name, instancer in self.instancers.items():
            output = self.mixers[name].executeGraph(num)
            print(output)
            output = {self.mappings[key]: value for key, value in output.items()}
            instancer.setInstanceParameter(**output)

    def setVisible(self, flag: bool,
                         ) -> None:
        """
        Set the visibility of the instancer.
        
        Args:
            flag (bool): The visibility flag."""

        instancers_prim = self.stage.GetPrimAtPath(self.instancers_path)
        if flag:
            instancers_prim.GetAttribute("visibility").Set("visible")
        else:
            instancers_prim.GetAttribute("visibility").Set("invisible")