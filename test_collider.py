import omni
import copy
import math
import os

from omni.isaac.kit import SimulationApp


cfg = {
    "headless": False,
}
simulation_app = SimulationApp(cfg)

CBCfg_D = {
    "resolution": 0.05,
    "block_size": 50,
    "collider_path": "/World/colliders",
    "collider_mode": "meshSimplification",
    "visible": True,
    "profiling": True,
}

CMCfg_D = {
    "collider_resolution": 0.05,
    "block_size": 50,
    "cache_size": 10,
    "build_colliders_n_meters_ahead": 4,
    "collider_path": "/World/colliders",
    "collider_builder_conf": CBCfg_D,
    "profiling": True,
}

PSMCfg_D = {
    "gravity": (0, 0, -1.62),
    "dt": 0.1666,
    "enable_ccd": True,
    "enable_stabilization": False,
}


if __name__ == "__main__":
    from omni.isaac.core import World
    from omni.usd import get_context
    from pxr import UsdLux, UsdGeom, Gf, UsdShade

    import numpy as np

    from src.terrain_management.large_scale_terrain.collider_manager import ColliderManager, ColliderManagerConf
    from src.configurations.physics_confs import PhysicsSceneConf
    from src.physics.physics_scene import PhysicsSceneManager
    from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops, add_collider, make_rigid

    def is_map_done():
        return True

    def bindMaterial(stage, mtl_prim_path, prim_path):
        mtl_prim = stage.GetPrimAtPath(mtl_prim_path)
        prim = stage.GetPrimAtPath(prim_path)
        shade = UsdShade.Material(mtl_prim)
        UsdShade.MaterialBindingAPI(prim).Bind(shade, UsdShade.Tokens.strongerThanDescendants)

    world = World(stage_units_in_meters=1.0)
    PSMCfg = PhysicsSceneConf(**PSMCfg_D)
    print(PSMCfg.physics_scene_args)
    PSM = PhysicsSceneManager(PSMCfg)

    stage = get_context().get_stage()
    asset_path = os.path.join(os.getcwd(), "assets")

    # Let there be light
    light = UsdLux.DistantLight.Define(stage, "/World/sun")
    light.CreateIntensityAttr(3000.0)
    set_xform_ops(light.GetPrim(), orient=Gf.Quatd(0.76, (0.65, 0, 0)))

    # Create a sphere
    sphere = UsdGeom.Sphere.Define(stage, "/World/sphere")
    add_collider(stage, sphere.GetPath(), mode="boundingSphere")
    make_rigid(stage, sphere.GetPath())

    map = np.load("assets/Terrains/SouthPole/LM1_final_adj_5mpp_surf/dem.npy") * 0.1 / 5.0

    for i in range(100):
        world.step(render=True)

    CM_Cfg = ColliderManagerConf(**CMCfg_D)
    source_resolution = 0.1
    CM = ColliderManager(CM_Cfg, map, map.shape, (map.shape[0] * 0.1 / 2, map.shape[1] * 0.1 / 2), source_resolution)

    C = 0
    R = 100
    max_displacement = 1.0 / 120
    acquisition_rate = 1
    perimeter = 2 * np.pi * R
    rotation_rate = int(perimeter / max_displacement)
    spiral_rate = 0.0 / rotation_rate
    theta = np.linspace(0, 2 * np.pi, rotation_rate)
    i = 0
    i2 = 1.0

    x = C + i2 * R * math.cos(theta[i])
    y = C + i2 * R * math.sin(theta[i])

    initial_position = (0, 0)

    timeline = omni.timeline.get_timeline_interface()
    # timeline.play()

    last_position = initial_position
    CM.build()
    CM.update(initial_position)
    set_xform_ops(sphere.GetPrim(), translate=Gf.Vec3d(0, 0, 20))

    while True:
        x = sphere.GetPrim().GetAttribute("xformOp:translate").Get()[0]
        y = sphere.GetPrim().GetAttribute("xformOp:translate").Get()[1]

        dist = math.sqrt((last_position[0] - x) ** 2 + (last_position[1] - y) ** 2)
        if dist > 2.0:
            last_position = (x, y)
            xu = int(x)
            yu = int(y)
            CM.update((x, y))
        world.step(render=True)

        i += 1
        i = i % theta.shape[0]

    timeline.stop()
    simulation_app.close()
