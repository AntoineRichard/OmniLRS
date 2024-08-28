import omni
import copy
import math
import os

import numpy as np
from omni.isaac.kit import SimulationApp
from scipy.spatial.transform import Rotation as SSTR

R_Z90 = np.array(
    [
        [math.cos(math.pi / 2), -math.sin(math.pi / 2), 0],
        [math.sin(math.pi / 2), math.cos(math.pi / 2), 0],
        [0, 0, 1],
    ]
)
R_X90 = np.array(
    [
        [1, 0, 0],
        [0, math.cos(math.pi / 2), -math.sin(math.pi / 2)],
        [0, math.sin(math.pi / 2), math.cos(math.pi / 2)],
    ]
)


def loadSandMaterial(asset_path: str):
    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        # mtl_url="http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Natural/Sand.mdl",
        mtl_url=asset_path + "/Textures/LunarRegolith8k.mdl",
        mtl_name="LunarRegolith8k",
        mtl_path="/Looks/LunarRegolith8k",
    )


def EMAquat(q1, q2, alpha):
    dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]
    if dot < 0:
        alpha2 = -alpha
    else:
        alpha2 = copy.copy(alpha)
    x = q1[0] * (1 - alpha2) + q2[0] * alpha2
    y = q1[1] * (1 - alpha2) + q2[1] * alpha2
    z = q1[2] * (1 - alpha2) + q2[2] * alpha2
    w = q1[3] * (1 - alpha2) + q2[3] * alpha2
    s = math.sqrt(x * x + y * y + z * z + w * w)
    return x / s, y / s, z / s, w / s


cfg = {
    "renderer": "PathTracing",
    "headless": False,
    "samples_per_pixel_per_frame": 32,
    "max_bounces": 6,
    "max_specular_transmission_bounces": 6,
    "max_volume_bounces": 4,
    "subdiv_refinement_level": 0,
}
# cfg = {
#    "headless": True,
# }
simulation_app = SimulationApp(cfg)

HRDEMCfg_D = {
    "num_blocks": 4,
    "block_size": 50,
    "pad_size": 10.0,
    "max_blocks": int(1e7),
    "seed": 42,
    "resolution": 0.05,
    "z_scale": 1.0,
    "source_resolution": 5.0,
    "resolution": 0.025,
    "interpolation_padding": 2,
    "generate_craters": True,
}
CWMCfg_D = {
    "num_workers": 8,
    "input_queue_size": 400,
    "output_queue_size": 30,
}
IWMCfg_D = {
    "num_workers": 1,
    "input_queue_size": 400,
    "output_queue_size": 30,
}
CraterDBCfg_D = {
    "block_size": 50,
    "max_blocks": 7,
    "save_to_disk": False,
    "write_to_disk_interval": 100,
}
CGCfg_D = {
    "profiles_path": "assets/Terrains/crater_spline_profiles.pkl",
    "min_xy_ratio": 0.85,
    "max_xy_ratio": 1.0,
    "random_rotation": True,
    "seed": 42,
    "num_unique_profiles": 10000,
}
CDDCfg_D = {
    "densities": [0.025, 0.05, 0.5],
    "radius": [[1.5, 2.5], [0.75, 1.5], [0.25, 0.5]],
    "num_repeat": 1,
    "seed": 42,
}
CraterSamplerCfg_D = {
    "block_size": 50,
    "crater_gen_cfg": CGCfg_D,
    "crater_dist_cfg": CDDCfg_D,
}
CraterBuilderCfg_D = {
    "block_size": 50,
    "pad_size": 10.0,
    "resolution": 0.025,
    "z_scale": 1.0,
}
ICfg = {
    "source_resolution": 5.0,
    "target_resolution": 0.025,
    "source_padding": 2,
    "method": "bicubic",
}
HRDEMGenCfg_D = {
    "high_res_dem_cfg": HRDEMCfg_D,
    "crater_db_cfg": CraterDBCfg_D,
    "crater_sampler_cfg": CraterSamplerCfg_D,
    "crater_builder_cfg": CraterBuilderCfg_D,
    "interpolator_cfg": ICfg,
    "crater_worker_manager_cfg": CWMCfg_D,
    "interpolator_worker_manager_cfg": IWMCfg_D,
}

MMCfg_D = {
    "folder_path": "assets/Terrains/SouthPole",
    "lr_dem_name": "crater",
    "hrdem_settings": HRDEMGenCfg_D,
}

NGCMMCfg_D = {
    "num_texels_per_level": 384,
    "target_res": 0.02,
    "fine_interpolation_method": "bilinear",
    "coarse_interpolation_method": "bicubic",
    "fine_acceleration_mode": "hybrid",
    "coarse_acceleration_mode": "gpu",
    "profiling": True,
}
RDBCfg_D = {
    "block_size": 50,
    "max_blocks": int(1e7),
    "save_to_disk": False,
    "write_to_disk_interval": 1000,
}
RSCfg_1_D = {
    "block_size": 50,
    "rock_dist_cfg": {
        "position_distribution": {
            "name": "thomas_point_process",
            "parent_density": 0.04,
            "child_density": 100,
            "sigma": 3,
            "seed": 42,
        },
        "scale_distribution": {
            "name": "uniform",
            "min": 0.02,
            "max": 0.05,
            "seed": 42,
        },
        "seed": 42,
        "num_rock_id": 49,
    },
}
RSCfg_2_D = {
    "block_size": 50,
    "rock_dist_cfg": {
        "position_distribution": {
            "name": "thomas_point_process",
            "parent_density": 0.01,
            "child_density": 25,
            "sigma": 3,
            "seed": 42,
        },
        "scale_distribution": {
            "name": "uniform",
            "min": 0.05,
            "max": 0.2,
            "seed": 42,
        },
        "seed": 43,
        "num_rock_id": 49,
    },
}
RGCfg_1_D = {
    "rock_db_cfg": RDBCfg_D,
    "rock_sampler_cfg": RSCfg_1_D,
    "rock_assets_folder": "assets/USD_Assets/rocks/small",
    "instancer_name": "very_small_rock_instancer",
    "seed": 42,
    "block_span": 0,
    "block_size": 50,
}
RGCfg_2_D = {
    "rock_db_cfg": RDBCfg_D,
    "rock_sampler_cfg": RSCfg_2_D,
    "rock_assets_folder": "assets/USD_Assets/rocks/small",
    "instancer_name": "large_rock_instancer",
    "seed": 42,
    "block_span": 1,
    "block_size": 50,
}
RMCfg_D = {
    "rock_gen_cfgs": [RGCfg_1_D, RGCfg_2_D],
    "instancers_path": "/World/rock_instancers",
    "seed": 42,
    "block_size": 50,
}


if __name__ == "__main__":
    from omni.isaac.core import World
    from omni.usd import get_context
    from pxr import UsdLux, UsdGeom, Gf, UsdShade, Vt, Sdf, Usd

    from WorldBuilders.pxr_utils import setDefaultOps, addDefaultOps, createXform

    from src.terrain_management.large_scale_terrain.map_manager import MapManagerCfg
    from src.terrain_management.large_scale_terrain.high_resolution_DEM_generator import (
        HighResDEMGenCfg,
    )
    from src.terrain_management.large_scale_terrain_manager import (
        NestedGeometricClipMapManagerCfg,
        LargeScaleTerrainManagerCfg,
        LargeScaleTerrainManager,
    )
    from src.terrain_management.large_scale_terrain.rock_manager import RockManagerCfg
    from src.labeling.auto_label import AutonomousLabeling
    from src.configurations.auto_labeling_confs import AutoLabelingConf
    from omni.isaac.sensor import Camera

    def setXformOp(prim: Usd.Prim, value, property: UsdGeom.XformOp.Type) -> None:
        """
        Sets a transform operatios on a prim.

        Args:
            prim (Usd.Prim): The prim to set the transform operation.
            value: The value of the transform operation.
            property (UsdGeom.XformOp.Type): The type of the transform operation.
        """

        xform = UsdGeom.Xformable(prim)
        op = None
        for xformOp in xform.GetOrderedXformOps():
            if xformOp.GetOpType() == property:
                op = xformOp
        if op:
            xform_op = op
        else:
            xform_op = xform.AddXformOp(property, UsdGeom.XformOp.PrecisionDouble, "")
        xform_op.Set(value)

    def setScale(prim: Usd.Prim, value: Gf.Vec3d) -> None:
        """
        Sets the scale of a prim.

        Args:
            prim (Usd.Prim): The prim to set the scale.
            value (Gf.Vec3d): The value of the scale.
        """

        setXformOp(prim, value, UsdGeom.XformOp.TypeScale)

    def setTranslate(prim: Usd.Prim, value: Gf.Vec3d) -> None:
        """
        Sets the translation of a prim.

        Args:
            prim (Usd.Prim): The prim to set the translation.
            value (Gf.Vec3d): The value of the translation.
        """

        setXformOp(prim, value, UsdGeom.XformOp.TypeTranslate)

    def setRotateXYZ(prim: Usd.Prim, value: Gf.Vec3d) -> None:
        """
        Sets the rotation of a prim.

        Args:
            prim (Usd.Prim): The prim to set the rotation.
            value (Gf.Vec3d): The value of the rotation.
        """

        setXformOp(prim, value, UsdGeom.XformOp.TypeRotateXYZ)

    def setOrient(prim: Usd.Prim, value: Gf.Quatd) -> None:
        """
        Sets the rotation of a prim.

        Args:
            prim (Usd.Prim): The prim to set the rotation.
            value (Gf.Quatd): The value of the rotation.
        """

        setXformOp(prim, value, UsdGeom.XformOp.TypeOrient)

    def setTransform(prim, value: Gf.Matrix4d) -> None:
        """
        Sets the transform of a prim.

        Args:
            prim (Usd.Prim): The prim to set the transform.
            value (Gf.Matrix4d): The value of the transform.
        """

        setXformOp(prim, value, UsdGeom.XformOp.TypeTransform)

    def setXformOps(
        prim,
        translate: Gf.Vec3d = Gf.Vec3d([0, 0, 0]),
        orient: Gf.Quatd = Gf.Quatd(1, Gf.Vec3d([0, 0, 0])),
        scale: Gf.Vec3d = Gf.Vec3d([1, 1, 1]),
    ) -> None:
        """
        Sets the transform of a prim.

        Args:
            prim (Usd.Prim): The prim to set the transform.
            translate (Gf.Vec3d): The value of the translation.
            orient (Gf.Quatd): The value of the rotation.
            scale (Gf.Vec3d): The value of the scale.
        """

        setTranslate(prim, translate)
        setOrient(prim, orient)
        setScale(prim, scale)

    def getTransform(prim: Usd.Prim, parent: Usd.Prim) -> Gf.Matrix4d:
        """
        Gets the transform of a prim relative to its parent.

        Args:
            prim (Usd.Prim): The prim to get the transform.
            parent (Usd.Prim): The parent of the prim.
        """

        return UsdGeom.XformCache(0).ComputeRelativeTransform(prim, parent)[0]

    def buildRealSenseRGB(stage, camera_path):
        rs_d455 = stage.DefinePrim(camera_path, "Xform")
        setXformOps(rs_d455, Gf.Vec3d(0, 0, 0), Gf.Quatd(1, (0, 0, 0)), Gf.Vec3d(1, 1, 1))
        # Mono
        camera_mono = UsdGeom.Camera.Define(stage, camera_path + "/mono")
        camera_mono.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100000))
        camera_mono.GetFocalLengthAttr().Set(1.93)
        camera_mono.GetFocusDistanceAttr().Set(0.0)
        camera_mono.GetFStopAttr().Set(0.0)
        camera_mono.GetHorizontalApertureAttr().Set(3.896)
        camera_mono.GetVerticalApertureAttr().Set(2.453)
        camera_mono_prim = camera_mono.GetPrim()
        setXformOps(camera_mono_prim, Gf.Vec3d(0, -0.0115, 0), Gf.Quatd(0.5, (0.5, -0.5, -0.5)), Gf.Vec3d(1, 1, 1))
        # Left
        camera_left = UsdGeom.Camera.Define(stage, camera_path + "/left")
        camera_left.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100000))
        camera_left.GetFocalLengthAttr().Set(1.93)
        camera_left.GetFocusDistanceAttr().Set(0.0)
        camera_left.GetFStopAttr().Set(0.0)
        camera_left.GetHorizontalApertureAttr().Set(3.896)
        camera_left.GetVerticalApertureAttr().Set(2.453)
        camera_left_prim = camera_left.GetPrim()
        setXformOps(camera_left_prim, Gf.Vec3d(0, 0.0475, 0), Gf.Quatd(0.5, (0.5, -0.5, -0.5)), Gf.Vec3d(1, 1, 1))
        # Right
        camera_right = UsdGeom.Camera.Define(stage, camera_path + "/right")
        camera_right.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100000))
        camera_right.GetFocalLengthAttr().Set(1.93)
        camera_right.GetFocusDistanceAttr().Set(0.0)
        camera_right.GetFStopAttr().Set(0.0)
        camera_right.GetHorizontalApertureAttr().Set(3.896)
        camera_right.GetVerticalApertureAttr().Set(2.453)
        camera_right_prim = camera_right.GetPrim()
        setXformOps(camera_right_prim, Gf.Vec3d(0, -0.0475, 0), Gf.Quatd(0.5, (0.5, -0.5, -0.5)), Gf.Vec3d(1, 1, 1))
        # Depth
        camera_depth = UsdGeom.Camera.Define(stage, camera_path + "/depth")
        camera_depth.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100000))
        camera_depth.GetFocalLengthAttr().Set(1.93)
        camera_depth.GetFocusDistanceAttr().Set(0.0)
        camera_depth.GetFStopAttr().Set(0.0)
        camera_depth.GetHorizontalApertureAttr().Set(3.896)
        camera_depth.GetVerticalApertureAttr().Set(2.453)
        camera_depth_prim = camera_depth.GetPrim()
        setXformOps(camera_depth_prim, Gf.Vec3d(0, 0, 0), Gf.Quatd(0.5, (0.5, -0.5, -0.5)), Gf.Vec3d(1, 1, 1))
        return rs_d455

    # attr = camera.GetPrim().CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token)
    # if attr.GetMetadata("allowedTokens") is None:
    #    attr.SetMetadata(
    #        "allowedTokens",
    #        [
    #            "pinhole",
    #            "fisheyeOrthographic",
    #            "fisheyeEquidistant",
    #            "fisheyeEquisolid",
    #            "fisheyePolynomial",
    #            "fisheyeSpherical",
    #            "fisheyeKannalaBrandtK3",
    #            "fisheyeRadTanThinPrism",
    #            "omniDirectionalStereo",
    #        ],
    #    )
    # properties = [
    #    "fthetaPolyA",
    #    "fthetaPolyB",
    #    "fthetaPolyC",
    #    "fthetaPolyD",
    #    "fthetaPolyE",
    #    "fthetaCx",
    #    "fthetaCy",
    #    "fthetaWidth",
    #    "fthetaHeight",
    #    "fthetaMaxFov",
    # ]
    # for property_name in properties:
    #    if camera.GetPrim().GetAttribute(property_name).Get() is None:
    #        camera.GetPrim().CreateAttribute(property_name, Sdf.ValueTypeNames.Float)

    # camera.GetPrim().GetAttribute("cameraProjectionType").Set(Vt.Token("pinhole"))
    # camera.GetPrim().GetAttribute("fthetaMaxFov").Set(98.0)
    # return camera

    def get_intrinsics_matrix(camera_prim) -> np.ndarray:
        """
        Returns:
            np.ndarray: the intrinsics of the camera (used for calibration)
        """
        focal_length = camera_prim.GetAttribute("focalLength").Get() / 10.0
        (width, height) = (1280, 720)
        horizontal_aperture = camera_prim.GetAttribute("horizontalAperture").Get() / 10.0
        vertical_aperture = (camera_prim.GetAttribute("horizontalAperture").Get() / 10.0) * (float(height) / width)
        fx = width * focal_length / horizontal_aperture
        fy = height * focal_length / vertical_aperture
        cx = width * 0.5
        cy = height * 0.5
        return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype="float32")

    def bindMaterial(stage, mtl_prim_path, prim_path):
        mtl_prim = stage.GetPrimAtPath(mtl_prim_path)
        prim = stage.GetPrimAtPath(prim_path)
        shade = UsdShade.Material(mtl_prim)
        UsdShade.MaterialBindingAPI(prim).Bind(shade, UsdShade.Tokens.strongerThanDescendants)

    world = World(stage_units_in_meters=1.0)
    stage = get_context().get_stage()
    asset_path = os.path.join(os.getcwd(), "assets")
    loadSandMaterial(asset_path)

    # Let there be light
    light = UsdLux.DistantLight.Define(stage, "/World/sun")
    light.CreateIntensityAttr(1000.0)
    addDefaultOps(light.GetPrim())
    setDefaultOps(light.GetPrim(), (0, 0, 0), (0.65, 0, 0, 0.76), (1, 1, 1))

    camera = buildRealSenseRGB(stage, "/World/camera")

    left_light = UsdLux.DiskLight.Define(stage, "/World/camera/left_light")
    left_light.CreateRadiusAttr(0.05)
    left_light.CreateIntensityAttr(10000000.0)
    left_light.CreateColorAttr(Gf.Vec3f(0.87, 0.97, 0.97))
    left_shaping_api = UsdLux.ShapingAPI(left_light.GetPrim())
    left_shaping_api.CreateShapingIesFileAttr().Set(asset_path + "/Textures/RobotProjector.ies")
    left_shaping_api.CreateShapingIesNormalizeAttr().Set(True)
    setXformOps(
        left_light.GetPrim(),
        Gf.Vec3d(0, 0.5, 0),
        Gf.Quatd(0.5, (0.5, -0.5, -0.5)),
        Gf.Vec3d(1, 1, 1),
    )
    right_light = UsdLux.DiskLight.Define(stage, "/World/camera/right_light")
    right_light.CreateRadiusAttr(0.05)
    right_light.CreateIntensityAttr(10000000.0)
    right_light.CreateColorAttr(Gf.Vec3f(0.87, 0.97, 0.97))
    right_shaping_api = UsdLux.ShapingAPI(right_light.GetPrim())
    right_shaping_api.CreateShapingIesFileAttr().Set(asset_path + "/Textures/RobotProjector.ies")
    right_shaping_api.CreateShapingIesNormalizeAttr().Set(True)
    setXformOps(
        right_light.GetPrim(),
        Gf.Vec3d(0.0, -0.5, 0),
        Gf.Quatd(0.5, (0.5, -0.5, -0.5)),
        Gf.Vec3d(1, 1, 1),
    )

    C = 0
    R = 50
    render_substeps = 4
    max_displacement = 2.0 / 30
    acquisition_rate = 1
    perimeter = 2 * np.pi * R
    rotation_rate = int(perimeter / max_displacement)
    spiral_rate = 0.0 / rotation_rate
    theta = np.linspace(0, 2 * np.pi, rotation_rate)
    i = 0
    i2 = 1.0

    x = C + i2 * R * math.cos(theta[i])
    y = C + i2 * R * math.sin(theta[i])

    initial_position = (x, y)

    LSTMCfg_D = {
        "map_name": "Site20_final_adj_5mpp_surf",
        "pixel_coordinates": (0, 0),
        "ll_coordinates": (0, 0),
        "meters_coordinates": initial_position,
        "coordinate_format": "meters",
        "visual_mesh_update_threshold": 2.0,
    }
    AL_1_Cfg_D = {
        "num_images": 10000000,
        "prim_path": "/World",
        "camera_name": "mono",
        "camera_resolution": (1280, 720),
        "data_dir": "data",
        "annotator_list": ["rgb", "pose"],
        "image_format": "png",
        "annot_format": "json",
        "element_per_folder": 10000000,
        "add_noise_to_rgb": False,
        "sigma": 0.0,
        "seed": 42,
    }
    AL_2_Cfg_D = {
        "num_images": 10000000,
        "prim_path": "/World",
        "camera_name": "left",
        "camera_resolution": (1280, 720),
        "data_dir": "data",
        "annotator_list": ["ir", "pose"],
        "image_format": "png",
        "annot_format": "json",
        "element_per_folder": 10000000,
        "add_noise_to_rgb": False,
        "sigma": 0.0,
        "seed": 42,
    }
    AL_3_Cfg_D = {
        "num_images": 10000000,
        "prim_path": "/World",
        "camera_name": "right",
        "camera_resolution": (1280, 720),
        "data_dir": "data",
        "annotator_list": ["ir", "pose"],
        "image_format": "png",
        "annot_format": "json",
        "element_per_folder": 10000000,
        "add_noise_to_rgb": False,
        "sigma": 0.0,
        "seed": 42,
    }
    AL_4_Cfg_D = {
        "num_images": 10000000,
        "prim_path": "/World",
        "camera_name": "depth",
        "camera_resolution": (1280, 720),
        "data_dir": "data",
        "annotator_list": ["depth", "pose"],
        "image_format": "png",
        "annot_format": "json",
        "element_per_folder": 10000000,
        "add_noise_to_rgb": False,
        "sigma": 0.0,
        "seed": 42,
    }

    # Prime the world
    for i in range(100):
        world.step(render=True)

    ALCFG_1 = AutoLabelingConf(**AL_1_Cfg_D)
    ALCFG_2 = AutoLabelingConf(**AL_2_Cfg_D)
    ALCFG_3 = AutoLabelingConf(**AL_3_Cfg_D)
    ALCFG_4 = AutoLabelingConf(**AL_4_Cfg_D)
    AL_1 = AutonomousLabeling(ALCFG_1)
    AL_2 = AutonomousLabeling(ALCFG_2)
    AL_3 = AutonomousLabeling(ALCFG_3)
    AL_4 = AutonomousLabeling(ALCFG_4)
    AL_1.load()
    AL_2.load()
    AL_3.load()
    AL_4.load()

    mm_settings = MapManagerCfg(**MMCfg_D)
    ngcmm_settings = NestedGeometricClipMapManagerCfg(**NGCMMCfg_D)
    rm_settings = RockManagerCfg(**RMCfg_D)
    lstm_settings = LargeScaleTerrainManagerCfg(**LSTMCfg_D)

    LSTM = LargeScaleTerrainManager(lstm_settings, ngcmm_settings, mm_settings, rm_settings)

    LSTM.build()
    height = LSTM.get_height_global(initial_position)

    bindMaterial(stage, "/Looks/LunarRegolith8k", "/World/Terrain")

    # Q_camera2 = quat_mul(heading_to_quat(0), Q_camera)

    # setDefaultOps(camera.GetPrim(), (0, 0, height + 0.5), Q_camera2, (1, 1, 1))
    # LSTM.update_visual_mesh((0, 0))
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    i3 = 0
    target = R * 2 * math.pi / max_displacement / acquisition_rate
    target += 10
    Q_camera = None
    update_done = False

    for i in range(100):
        world.step(render=True)

    print("Starting simulation")
    while target > i3:
        x_new = C + i2 * R * math.cos(theta[i])
        y_new = C + i2 * R * math.sin(theta[i])
        x_delta = x_new - x
        y_delta = y_new - y
        coords = (x_delta, y_delta)

        normal_vector = LSTM.get_normal_local(coords)
        heading_vector = np.array([np.cos(theta[i]), np.sin(theta[i]), 0])
        heading_vector = heading_vector / np.linalg.norm(heading_vector)
        heading_vector = np.cross(normal_vector, heading_vector)
        heading_vector = heading_vector / np.linalg.norm(heading_vector)
        heading_vector_2 = np.cross(normal_vector, heading_vector)
        RNorm = np.array([heading_vector, heading_vector_2, normal_vector]).T
        # mat = np.matmul(R_Z90, R_X90)
        # mat = np.matmul(RNorm, mat)

        RM = SSTR.from_matrix(RNorm)
        if Q_camera is None:
            Q_camera = RM.as_quat()
        else:
            Q_camera = EMAquat(Q_camera, RM.as_quat(), 0.0333)

        setXformOps(
            camera.GetPrim(),
            Gf.Vec3d(x_delta, y_delta, LSTM.get_height_local(coords) + 0.5),
            Gf.Quatd(Q_camera[-1], (Q_camera[0], Q_camera[1], Q_camera[2])),
            Gf.Vec3d(1, 1, 1),
        )

        update, coords = LSTM.update_visual_mesh(coords)

        for _ in range(render_substeps):
            world.step(render=True)

        i = (i + 1) % rotation_rate
        if i % acquisition_rate == 0:
            try:
                AL_1.record()
                AL_2.record()
                AL_3.record()
                AL_4.record()
                i3 += 1
            except Exception as e:
                print(e)
    #    i2 += spiral_rate
    timeline.stop()
    LSTM.map_manager.hr_dem_gen.shutdown()
    simulation_app.close()
