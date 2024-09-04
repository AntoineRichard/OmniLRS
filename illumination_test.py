import omni
import copy
import math
import os

import numpy as np
from omni.isaac.kit import SimulationApp
from scipy.spatial.transform import Rotation as SSTR


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
#    "headless": False,
# }
simulation_app = SimulationApp(cfg)

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
    "hr_dem_generate_craters": False,
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
    "lr_dem_name": "ldem_87s_5mpp",
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
    "terrain_collider_enabled": False,
    "terrain_collider_resolution": 0.05,
    "terrain_collider_cache_size": 10,
    "terrain_collider_building_threshold": 4.0,
    "rock_gen_cfgs": [],
}

SEConf_D = {
    "start_date": {
        "year": 2024,
        "month": 9,
        "day": 4,
        "hour": 23,
        "minute": 17,
    },
    "time_scale": 3600.0,
    "update_interval": 600.0,
}

if __name__ == "__main__":
    from omni.isaac.core import World
    from omni.usd import get_context
    from pxr import UsdLux, UsdGeom, Gf

    from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops
    from src.terrain_management.large_scale_terrain.utils import ScopedTimer
    from src.configurations.large_scale_terrain_confs import LargeScaleTerrainConf
    from src.terrain_management.large_scale_terrain_manager import (
        LargeScaleTerrainManager,
    )
    from src.labeling.auto_label import AutonomousLabeling
    from src.configurations.auto_labeling_confs import AutoLabelingConf
    from src.stellar.stellar_engine import StellarEngine
    from src.configurations.stellar_engine_confs import StellarEngineConf

    def buildCamera(stage, camera_path):
        cam = stage.DefinePrim(camera_path, "Xform")
        # Mono
        camera_mono = UsdGeom.Camera.Define(stage, camera_path + "/ortho")
        camera_mono.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100000))
        camera_mono.GetFocalLengthAttr().Set(1.93)
        camera_mono.GetFocusDistanceAttr().Set(0.0)
        camera_mono.GetFStopAttr().Set(0.0)
        camera_mono.GetHorizontalApertureAttr().Set(3.896)
        camera_mono.GetVerticalApertureAttr().Set(2.453)
        return cam

    world = World(stage_units_in_meters=1.0)
    stage = get_context().get_stage()
    asset_path = os.path.join(os.getcwd(), "assets")

    # Let there be light
    light = UsdLux.DistantLight.Define(stage, "/World/sun")
    light.CreateIntensityAttr(1000.0)
    set_xform_ops(light.GetPrim(), Gf.Vec3d(0, 0, 0), Gf.Quatd(0.76, (0.65, 0, 0)), Gf.Vec3d(1, 1, 1))

    camera = buildCamera(stage, "/World/camera")
    set_xform_ops(camera, Gf.Vec3d(0, 0, 60000), Gf.Quatd(1, (0, 0, 0)), Gf.Vec3d(1, 1, 1))

    LSTCfg_D["starting_position"] = (0, 0)

    AL_Cfg_D = {
        "num_images": 10000000,
        "prim_path": "/World",
        "camera_names": ["ortho"],
        "camera_resolutions": [(2048, 2048)],
        "data_dir": "data",
        "annotators_list": [["rgb", "pose"]],
        "image_formats": ["png"],
        "annot_formats": ["json"],
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

    SEConf = StellarEngineConf(**SEConf_D)
    SE = StellarEngine(SEConf)
    SE.setLatLon(*LSTM.map_manager.get_lat_lon())

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    render_substeps = 2

    for _ in range(100):
        world.step(render=True)

    img = np.zeros((2048, 2048), dtype=np.float64)

    print("Starting simulation")
    for i in range(20000):
        with ScopedTimer("env_step", active=False):
            SE.update(1)  # 1 second --> 1 hour
            alt, az, dist = SE.getAltAz("sun")
            print(alt, az)
            quat = SE.convertAltAzToQuat(alt, az)
            set_xform_ops(
                light.GetPrim(), Gf.Vec3d(0, 0, 0), Gf.Quatd(quat[0], (quat[1], quat[2], quat[3])), Gf.Vec3d(1, 1, 1)
            )
            for _ in range(2):
                world.step(render=True)
            try:
                for camera_name, name, annotator in AL.annotators.values():
                    if name == "rgb":
                        img += np.mean(annotator.get_data().astype(float), axis=2)
                pass
            except Exception as e:
                print(e)
    np.save("illumination_test.npy", img)
    timeline.stop()
    LSTM.map_manager.hr_dem_gen.shutdown()
    simulation_app.close()
