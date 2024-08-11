import omni
import copy
import math
import os

from omni.isaac.kit import SimulationApp

def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return x, y, z, w

def heading_to_quat(heading):
    return 0, 0, math.sin(heading / 2), math.cos(heading / 2)

def loadSandMaterial(asset_path: str):
    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        #mtl_url="http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Natural/Sand.mdl",
        mtl_url=asset_path+"/Textures/Sand.mdl",
        mtl_name="Sand",
        mtl_path="/Looks/Sand",
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


#def loadCarpetMaterial():
#    omni.kit.commands.execute(
#        "CreateMdlMaterialPrimCommand",
#        mtl_url="http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Carpet/Carpet_Pattern_Squares_Multi.mdl",
#        mtl_name="Carpet",
#        mtl_path="/Looks/Carpet",
#    )


simulation_app = SimulationApp({"headless": True})
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
    "output_queue_size": 16,
    "worker_queue_size": 2,
}
IWMCfg_D = {
    "num_workers": 1,
    "input_queue_size": 400,
    "output_queue_size": 30,
    "worker_queue_size": 200,
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
}

NGCMMCfg_D = {
    "num_texels_per_level": 384,
    "target_res": 0.01,
    "fine_interpolation_method": "bicubic",
    "coarse_interpolation_method": "bicubic",
    "fine_acceleration_mode": "hybrid",
    "coarse_acceleration_mode": "hybrid",
    "profiling": False,
}

if __name__ == "__main__":
    from omni.isaac.core import World
    from omni.usd import get_context
    from pxr import UsdLux, UsdGeom, Gf, UsdShade

    from WorldBuilders.pxr_utils import setDefaultOps, addDefaultOps
    import numpy as np

    from src.terrain_management.large_scale_terrain.map_manager import MapManagerCfg
    from src.terrain_management.large_scale_terrain.high_resolution_DEM_generator import (
        HighResDEMGenCfg,
    )
    from src.terrain_management.large_scale_terrain_manager import (
        NestedGeometricClipMapManagerCfg,
        LargeScaleTerrainManagerCfg,
        LargeScaleTerrainManager,
    )
    from src.labeling.auto_label import AutonomousLabeling
    from src.configurations.auto_labeling_confs import AutoLabelingConf

    def bindMaterial(stage, mtl_prim_path, prim_path):
        mtl_prim = stage.GetPrimAtPath(mtl_prim_path)
        prim = stage.GetPrimAtPath(prim_path)
        shade = UsdShade.Material(mtl_prim)
        UsdShade.MaterialBindingAPI(prim).Bind(
            shade, UsdShade.Tokens.strongerThanDescendants
        )

    world = World(stage_units_in_meters=1.0)
    stage = get_context().get_stage()
    asset_path = os.path.join(os.getcwd(), "assets")
    loadSandMaterial(asset_path)
    #loadCarpetMaterial()

    # Let there be light
    light = UsdLux.DistantLight.Define(stage, "/World/sun")
    light.CreateIntensityAttr(3000.0)
    addDefaultOps(light.GetPrim())
    setDefaultOps(light.GetPrim(), (0, 0, 0), (0.65, 0, 0, 0.76), (1, 1, 1))

    Q_camera = (0.707,0,0,0.707) # x,y,z,w

    # Camera
    camera = UsdGeom.Camera.Define(stage, "/World/camera")
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100000))
    camera.GetFocalLengthAttr().Set(18)
    addDefaultOps(camera.GetPrim())
    setDefaultOps(camera.GetPrim(), (0, 0, 0), Q_camera, (1, 1, 1))
    left_light = UsdLux.DiskLight.Define(stage, "/World/camera/left_light")
    left_light.CreateRadiusAttr(0.05)
    left_light.CreateIntensityAttr(10000000.0)
    left_light.CreateColorAttr(Gf.Vec3f(0.87, 0.97, 0.97))
    left_shaping_api = UsdLux.ShapingAPI(left_light.GetPrim())
    left_shaping_api.CreateShapingIesFileAttr().Set(asset_path + "/Textures/RobotProjector.ies")
    left_shaping_api.CreateShapingIesNormalizeAttr().Set(True)
    addDefaultOps(left_light.GetPrim())
    setDefaultOps(left_light.GetPrim(), (-0.5, 0, 0), (0,0,0,1), (1, 1, 1))
    right_light = UsdLux.DiskLight.Define(stage, "/World/camera/right_light")
    right_light.CreateRadiusAttr(0.05)
    right_light.CreateIntensityAttr(10000000.0)
    right_light.CreateColorAttr(Gf.Vec3f(0.87, 0.97, 0.97))
    right_shaping_api = UsdLux.ShapingAPI(right_light.GetPrim())
    right_shaping_api.CreateShapingIesFileAttr().Set(asset_path + "/Textures/RobotProjector.ies")
    right_shaping_api.CreateShapingIesNormalizeAttr().Set(True)
    addDefaultOps(right_light.GetPrim())
    setDefaultOps(right_light.GetPrim(), (0.5, 0, 0), (0,0,0,1), (1, 1, 1))

    C = 0
    R = 500 * 5
    max_displacement = 2.0 / 30
    acquisition_rate = 1
    perimeter = 2 * np.pi * R
    rotation_rate = int(perimeter / max_displacement)
    spiral_rate = 0.0 / rotation_rate
    theta = np.linspace(0, 2 * np.pi, rotation_rate)
    i = 0
    i2 = 1.0
    print(rotation_rate)

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
    ALCfg_D = {
        "num_images": 10000000,
        "prim_path": '/World',
        "camera_name": 'camera',
        "camera_resolution": (1280,720),
        "data_dir": 'data',
        "annotator_list": ["rgb"],
        "image_format": 'png',
        "annot_format": 'json',
        "element_per_folder": 10000000,
        "add_noise_to_rgb": False,
        "sigma": 0.0,
        "seed": 42,
    }
    ALCFG = AutoLabelingConf(**ALCfg_D)
    AL = AutonomousLabeling(ALCFG)
    AL.load()

    hrdem_settings = HighResDEMGenCfg(**HRDEMGenCfg_D)
    mm_settings = MapManagerCfg(**MMCfg_D)
    ngcmm_settings = NestedGeometricClipMapManagerCfg(**NGCMMCfg_D)
    lstm_settings = LargeScaleTerrainManagerCfg(**LSTMCfg_D)


    LSTM = LargeScaleTerrainManager(
        lstm_settings, ngcmm_settings, hrdem_settings, mm_settings
    )

    LSTM.build()
    height = LSTM.get_height_global(initial_position)

    bindMaterial(stage, "/Looks/Sand", "/World/Terrain")

    Q_camera2 = quat_mul(heading_to_quat(0), Q_camera)
    setDefaultOps(
        camera.GetPrim(), (0, 0, height + 0.5), Q_camera2, (1, 1, 1)
    )
    # LSTM.update_visual_mesh((0, 0))
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    i3 = 0
    target = R * 2 * math.pi / max_displacement / acquisition_rate
    target = target / 8
    target += 10
    Q_camera4 = None
    while target > i3:
        x_new = C + i2 * R * math.cos(theta[i])
        y_new = C + i2 * R * math.sin(theta[i])
        x_delta = x_new - x
        y_delta = y_new - y
        coords = (x_delta, y_delta)
        Q_camera2 = quat_mul(heading_to_quat(theta[i]), Q_camera)
        Q_terrain = LSTM.get_normal_local(coords)
        Q_camera3 = quat_mul(Q_terrain, Q_camera2)
        if Q_camera4 is None:
            Q_camera4 = copy.copy(Q_camera3)
        else:
            Q_camera4 = EMAquat(Q_camera4, Q_camera3, 0.0333)
        LSTM.update_visual_mesh(coords)
        setDefaultOps(
           camera.GetPrim(),
           (x_delta, y_delta, LSTM.get_height_local(coords) + 0.5),
           Q_camera4,
           (1, 1, 1),
        )
        world.step(render=True)
        i = (i + 1) % rotation_rate
        if i%acquisition_rate == 0:
            try:
                AL.record()
                i3 += 1
            except Exception as e:
                print(e)
    #    i2 += spiral_rate
    timeline.stop()
    LSTM.map_manager.hr_dem_gen.shutdown()
    simulation_app.close()
