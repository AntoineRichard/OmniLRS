__author__ = "Antoine Richard, Junnosuke Kahamora"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple, Dict, Union
import omni.kit.actions.core
import random, string
from pxr import Usd
import omni
import os

# import omni.replicator.core as rep
from src.labeling.rep_utils import writerFactory
from src.configurations.auto_labeling_confs import AutoLabelingConf
import omni.replicator.core as rep


class AutonomousLabeling:
    """
    AutonomousLabeling class. It is used to generate synthetic data from a USD scene."""

    def __init__(
        self,
        cfg: AutoLabelingConf,
        **kwargs,
    ) -> None:
        """
        Initialize the AutonomousLabeling class.

        Args:
            prim_path (str): The path of the USD scene.
            camera_name (str, optional): The name of the camera. Defaults to "camera".
            camera_resolution (Tuple[int,int], optional): The resolution of the camera. Defaults to (640, 480).
            data_dir (str, optional): The directory where the synthetic data will be stored. Defaults to ".".
            annotator_list (List[str], optional): The list of annotators that will be used to generate the synthetic data.
                                                  Defaults to ["rgb, instance_segmentation, semantic_segmentation"].
            image_format (str, optional): The format of the images. Defaults to "png".
            annot_format (str, optional): The format of the annotations. Defaults to "json".
            element_per_folder (int, optional): The number of elements per folder. Defaults to 10000.
            add_noise_to_rgb (bool, optional): Whether to add noise to the RGB data. Defaults to False.
            sigma (float, optional): The standard deviation of the noise. Defaults to 5.0.
            seed (int, optional): The seed used to generate the random numbers. Defaults to 42.
        """

        # Camera parameters
        print(cfg.prim_path)
        self.camera_name = cfg.camera_name
        self.camera_resolution = cfg.camera_resolution

        # Data storage parameters
        self.data_hash = "".join(
            random.sample(string.ascii_letters + string.digits, 16)
        )
        self.data_dir = os.path.join(cfg.data_dir, self.data_hash)

        # Synthetic data parameters
        self.annotator_list = cfg.annotator_list
        self.add_noise_to_rgb = cfg.add_noise_to_rgb
        self.sigma = cfg.sigma
        self.seed = cfg.seed
        self.image_format = cfg.image_format
        self.annot_format = cfg.annot_format
        self.element_per_folder = cfg.element_per_folder
        writer_cfg = self.formatWriterConfig()
        self.synthetic_writers = {
            name: writerFactory(name, **writer_cfg) for name in cfg.annotator_list
        }
        self.loggers = {
            "rgb": self.enableRGBData,
            "instance_segmentation": self.enableInstanceData,
            "semantic_segmentation": self.enableSemanticData,
        }

        self.stage = omni.usd.get_context().get_stage()
        self.meta_prim = self.stage.GetPrimAtPath(cfg.prim_path)
        self.annotator = {}
        self.synth_counter = 0

    def formatWriterConfig(self) -> Dict[str, Union[float, bool, int, str]]:
        cfg = {}
        cfg["root_path"] = self.data_dir
        cfg["element_per_folder"] = self.element_per_folder
        cfg["image_format"] = self.image_format
        cfg["annot_format"] = self.annot_format
        cfg["add_noise"] = self.add_noise_to_rgb
        cfg["sigma"] = self.sigma
        cfg["seed"] = self.seed
        return cfg

    def findCameraForAnnotation(self, camera_name: str) -> None:
        """
        Find the camera prim and path of the camera that will be used to generate the synthetic data.

        Args:
            camera_name (str): The name of the camera."""

        for prim in Usd.PrimRange(self.meta_prim):
            if prim.GetName() == camera_name:
                self.camera_prim = prim
                self.camera_path = str(prim.GetPath())

    def load(self) -> None:
        """
        Finds the camera and enables the collection of RGB, instance segmentation, and semantic segmentation.
        """

        self.findCameraForAnnotation(self.camera_name)
        self.setCamera(self.camera_path, self.camera_resolution)
        for annotator in self.annotator_list:
            self.loggers[annotator]()

    def setCamera(self, camera_path: str, res=(640, 480)) -> None:
        """
        Set the camera resolution that will be used to generate the synthetic data.

        Args:
            camera_path (str): The path to the camera.
            res (tuple, optional): The resolution of the camera. Defaults to (640, 480).
        """

        self.render_product = rep.create.render_product(camera_path, res)

    def enableRGBData(self) -> None:
        """
        Enable the collection of RGB data."""

        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.annotator["rgb"] = rgb_annot
        rgb_annot.attach([self.render_product])

    def enableSemanticData(self) -> None:
        """
        Enable the collection of semantic segmentation data."""

        semantic_annot = rep.AnnotatorRegistry.get_annotator(
            "semantic_segmentation", init_params={"colorize": True}
        )
        self.annotator["semantic_segmentation"] = semantic_annot
        semantic_annot.attach([self.render_product])

    def enableInstanceData(self) -> None:
        """
        Enable the collection of instance segmentation data."""

        instance_annotator = rep.AnnotatorRegistry.get_annotator(
            "instance_segmentation", init_params={"colorize": True}
        )
        self.annotator["instance_segmentation"] = instance_annotator
        instance_annotator.attach([self.render_product])

    def record(self) -> None:
        """
        Record a frame of synthetic data."""

        for name, annotator in self.annotator.items():
            self.synthetic_writers[name].write(annotator.get_data())
        self.synth_counter += 1
