import omni
import copy
import math
import os

import numpy as np
from omni.isaac.kit import SimulationApp
from scipy.spatial.transform import Rotation as SSTR


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
    "headless": True,
    "samples_per_pixel_per_frame": 32,
    "max_bounces": 6,
    "max_specular_transmission_bounces": 6,
    "max_volume_bounces": 4,
    "subdiv_refinement_level": 0,
}
# cfg = {
#    "headless": False,
# }
simulation_app = SimulationApp(cfg)


RSCfg_1_D = {
    "block_size": 50,
    "seed": 42,
    "rock_dist_cfg": {
        "position_distribution": {
            "name": "thomas_point_process",
            "parent_density": 0.04,
            "child_density": 100,
            "sigma": 3.0,
            "seed": 42,
        },
        "scale_distribution": {
            "name": "uniform",
            "min": 0.02,
            "max": 0.05,
            "seed": 43,
        },
    },
}
RSCfg_2_D = {
    "block_size": 50,
    "seed": 42,
    "rock_dist_cfg": {
        "position_distribution": {
            "name": "thomas_point_process",
            "parent_density": 0.01,
            "child_density": 25,
            "sigma": 3.0,
            "seed": 44,
        },
        "scale_distribution": {
            "name": "uniform",
            "min": 0.05,
            "max": 0.2,
            "seed": 45,
        },
    },
}
RGCfg_1_D = {
    "rock_sampler_cfg": RSCfg_1_D,
    "rock_assets_folder": "assets/USD_Assets/rocks/small_rocks_v4",
    "instancer_name": "very_small_rock_instancer",
    "seed": 46,
    "block_span": 1,
    "add_colliders": False,
    "texture_name": "seaside_rock_2k",
    "texture_path": "assets/Textures/seaside_rock_2k.mdl",
}
RGCfg_2_D = {
    "rock_sampler_cfg": RSCfg_2_D,
    "rock_assets_folder": "assets/USD_Assets/rocks/small_rocks_v4",
    "instancer_name": "small_rock_instancer",
    "seed": 47,
    "block_span": 2,
    "add_colliders": False,
    "collider_mode": "none",
    "semantic_label": "small_rock",
    "texture_name": "seaside_rock_2k",
    "texture_path": "assets/Textures/seaside_rock_2k.mdl",
}
LSTCfg_D = {
    "seed": 42,
    "crater_gen_seed": None,
    "crater_gen_distribution_seed": None,
    "crater_gen_metadata_seed": None,
    "rock_gen_main_seed": None,
    "profiling": True,
    "update_every_n_meters": 2.0,
    "z_scale": 1.0,
    "block_size": 50,
    "dbs_max_elements": 10000000,
    "dbs_save_to_disk": False,
    "dbs_write_interval": 1000,
    "hr_dem_resolution": 0.025,
    "hr_dem_generate_craters": True,
    "hr_dem_num_blocks": 4,
    "crater_gen_densities": [0.025, 0.05, 0.5],
    "crater_gen_radius": [[1.5, 2.5], [0.75, 1.5], [0.25, 0.5]],
    "crater_gen_profiles_path": "assets/Terrains/crater_spline_profiles.pkl",
    "crater_gen_padding": 10.0,
    "crater_gen_min_xy_ratio": 0.85,
    "crater_gen_max_xy_ratio": 1.0,
    "crater_gen_random_rotation": True,
    "crater_gen_num_unique_profiles": 10000,
    "num_workers_craters": 8,
    "num_workers_interpolation": 1,
    "input_queue_size": 400,
    "output_queue_size": 30,
    "hrdem_interpolation_method": "bicubic",
    "hrdem_interpolator_name": "PIL",
    "hrdem_interpolator_padding": 2,
    "lr_dem_folder_path": "assets/Terrains/SouthPole",
    "lr_dem_name": "Site20_final_adj_5mpp_surf",
    "starting_position": (0, 0),
    "geo_cm_num_texels_per_level": 384,
    "geo_cm_target_res": 0.02,
    "geo_cm_fine_interpolation_method": "bilinear",
    "geo_cm_coarse_interpolation_method": "bicubic",
    "geo_cm_fine_acceleration_mode": "hybrid",
    "geo_cm_coarse_acceleration_mode": "gpu",
    "geo_cm_semantic_label": "terrain",
    "geo_cm_texture_name": "LunarRegolith8k",
    "geo_cm_texture_path": "assets/Textures/LunarRegolith8k.mdl",
    "geo_cm_apply_smooth_shading": False,
    "terrain_collider_enabled": True,
    "terrain_collider_resolution": 0.05,
    "terrain_collider_cache_size": 10,
    "terrain_collider_building_threshold": 4.0,
    "rock_gen_cfgs": [
        RGCfg_1_D,
        RGCfg_2_D,
    ],
}


if __name__ == "__main__":
    from omni.isaac.core import World
    from omni.usd import get_context
    from pxr import UsdLux, UsdGeom, Gf, UsdShade, Vt, Sdf, Usd

    from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops
    from src.terrain_management.large_scale_terrain.utils import ScopedTimer
    from src.configurations.large_scale_terrain_confs import LargeScaleTerrainConf
    from src.terrain_management.large_scale_terrain_manager import (
        LargeScaleTerrainManager,
    )
    from src.labeling.auto_label import AutonomousLabeling
    from src.configurations.auto_labeling_confs import AutoLabelingConf
    from omni.isaac.sensor import Camera

    def buildRealSenseRGB(stage, camera_path):
        rs_d455 = stage.DefinePrim(camera_path, "Xform")
        set_xform_ops(rs_d455, Gf.Vec3d(0, 0, 0), Gf.Quatd(1, (0, 0, 0)), Gf.Vec3d(1, 1, 1))
        # Mono
        camera_mono = UsdGeom.Camera.Define(stage, camera_path + "/mono")
        camera_mono.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100000))
        camera_mono.GetFocalLengthAttr().Set(1.93)
        camera_mono.GetFocusDistanceAttr().Set(0.0)
        camera_mono.GetFStopAttr().Set(0.0)
        camera_mono.GetHorizontalApertureAttr().Set(3.896)
        camera_mono.GetVerticalApertureAttr().Set(2.453)
        camera_mono_prim = camera_mono.GetPrim()
        set_xform_ops(camera_mono_prim, Gf.Vec3d(0, -0.0115, 0), Gf.Quatd(0.5, (0.5, -0.5, -0.5)), Gf.Vec3d(1, 1, 1))
        # Left
        camera_left = UsdGeom.Camera.Define(stage, camera_path + "/left")
        camera_left.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100000))
        camera_left.GetFocalLengthAttr().Set(1.93)
        camera_left.GetFocusDistanceAttr().Set(0.0)
        camera_left.GetFStopAttr().Set(0.0)
        camera_left.GetHorizontalApertureAttr().Set(3.896)
        camera_left.GetVerticalApertureAttr().Set(2.453)
        camera_left_prim = camera_left.GetPrim()
        set_xform_ops(camera_left_prim, Gf.Vec3d(0, 0.0475, 0), Gf.Quatd(0.5, (0.5, -0.5, -0.5)), Gf.Vec3d(1, 1, 1))
        # Right
        camera_right = UsdGeom.Camera.Define(stage, camera_path + "/right")
        camera_right.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100000))
        camera_right.GetFocalLengthAttr().Set(1.93)
        camera_right.GetFocusDistanceAttr().Set(0.0)
        camera_right.GetFStopAttr().Set(0.0)
        camera_right.GetHorizontalApertureAttr().Set(3.896)
        camera_right.GetVerticalApertureAttr().Set(2.453)
        camera_right_prim = camera_right.GetPrim()
        set_xform_ops(camera_right_prim, Gf.Vec3d(0, -0.0475, 0), Gf.Quatd(0.5, (0.5, -0.5, -0.5)), Gf.Vec3d(1, 1, 1))
        # Depth
        camera_depth = UsdGeom.Camera.Define(stage, camera_path + "/depth")
        camera_depth.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100000))
        camera_depth.GetFocalLengthAttr().Set(1.93)
        camera_depth.GetFocusDistanceAttr().Set(0.0)
        camera_depth.GetFStopAttr().Set(0.0)
        camera_depth.GetHorizontalApertureAttr().Set(3.896)
        camera_depth.GetVerticalApertureAttr().Set(2.453)
        camera_depth_prim = camera_depth.GetPrim()
        set_xform_ops(camera_depth_prim, Gf.Vec3d(0, 0, 0), Gf.Quatd(0.5, (0.5, -0.5, -0.5)), Gf.Vec3d(1, 1, 1))
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

    world = World(stage_units_in_meters=1.0)
    stage = get_context().get_stage()
    asset_path = os.path.join(os.getcwd(), "assets")

    # Let there be light
    light = UsdLux.DistantLight.Define(stage, "/World/sun")
    light.CreateIntensityAttr(1000.0)
    set_xform_ops(light.GetPrim(), Gf.Vec3d(0, 0, 0), Gf.Quatd(0.76, (0.65, 0, 0)), Gf.Vec3d(1, 1, 1))

    camera = buildRealSenseRGB(stage, "/World/camera")

    left_light = UsdLux.DiskLight.Define(stage, "/World/camera/left_light")
    left_light.CreateRadiusAttr(0.05)
    left_light.CreateIntensityAttr(10000000.0)
    left_light.CreateColorAttr(Gf.Vec3f(0.87, 0.97, 0.97))
    left_shaping_api = UsdLux.ShapingAPI(left_light.GetPrim())
    left_shaping_api.CreateShapingIesFileAttr().Set(asset_path + "/Textures/RobotProjector.ies")
    left_shaping_api.CreateShapingIesNormalizeAttr().Set(True)
    set_xform_ops(
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
    set_xform_ops(
        right_light.GetPrim(),
        Gf.Vec3d(0.0, -0.5, 0),
        Gf.Quatd(0.5, (0.5, -0.5, -0.5)),
        Gf.Vec3d(1, 1, 1),
    )

    C = 0
    R = 800
    render_substeps = 2
    max_displacement = 2.0 / 30
    acquisition_rate = 15
    perimeter = 2 * np.pi * R
    rotation_rate = int(perimeter / max_displacement)
    spiral_rate = 0.0 / rotation_rate
    theta = np.linspace(0, 2 * np.pi, rotation_rate)
    i = 0
    i2 = 1.0

    x = C + i2 * R * math.cos(theta[i])
    y = C + i2 * R * math.sin(theta[i])

    initial_position = (x, y)

    LSTCfg_D["starting_position"] = initial_position

    AL_Cfg_D = {
        "num_images": 10000000,
        "prim_path": "/World",
        "camera_names": ["mono", "left", "right", "depth"],
        "camera_resolutions": [(1280, 720), (1280, 720), (1280, 720), (1280, 720)],
        "data_dir": "data",
        "annotators_list": [["rgb", "pose"], ["ir", "pose"], ["ir", "pose"], ["depth", "pose"]],
        "image_formats": ["png", "png", "png", "png"],
        "annot_formats": ["json", "json", "json", "json"],
        "element_per_folder": 10000000,
        "save_intrinsics": True,
    }
    # Prime the world
    for i in range(100):
        world.step(render=True)

    ALCFG = AutoLabelingConf(**AL_Cfg_D)
    AL = AutonomousLabeling(ALCFG)
    AL.load()

    lstm_settings = LargeScaleTerrainConf(**LSTCfg_D)
    LSTM = LargeScaleTerrainManager(lstm_settings)

    LSTM.build()
    height = LSTM.get_height_global(initial_position)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    i3 = 0
    target = R * 2 * math.pi / max_displacement / acquisition_rate
    target += 10
    Q_camera = None

    for _ in range(100):
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

        RM = SSTR.from_matrix(RNorm)
        if Q_camera is None:
            Q_camera = RM.as_quat()
        else:
            Q_camera = EMAquat(Q_camera, RM.as_quat(), 0.0333)

        set_xform_ops(
            camera.GetPrim(),
            Gf.Vec3d(x_delta, y_delta, LSTM.get_height_local(coords) + 0.5),
            Gf.Quatd(Q_camera[-1], (Q_camera[0], Q_camera[1], Q_camera[2])),
            Gf.Vec3d(1, 1, 1),
        )

        with ScopedTimer("update_visual_mesh", active=False):
            update, coords = LSTM.update_visual_mesh(coords)

        i = (i + 1) % rotation_rate
        if i % acquisition_rate == 0:
            for _ in range(render_substeps):
                with ScopedTimer("env_step", active=False):
                    world.step(render=True)
            try:
                AL.record()
                i3 += 1
            except Exception as e:
                print(e)
    #    i2 += spiral_rate
    timeline.stop()
    LSTM.map_manager.hr_dem_gen.shutdown()
    simulation_app.close()
