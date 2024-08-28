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
    "block_size": 10,
    "collider_path": "/World/colliders",
    "base_name": "collider_",
    "collider_mode": "meshSimplification",
}

CMCfg_D = {
    "collider_resolution": 0.05,
    "source_resolution": 0.1,
    "block_size": 10,
    "collider_path": "/World/colliders",
    "collider_builder_cfg": CBCfg_D,
}


if __name__ == "__main__":
    from omni.isaac.core import World
    from omni.usd import get_context
    from pxr import UsdLux, UsdGeom, Gf, UsdShade

    from WorldBuilders.pxr_utils import setDefaultOps, addDefaultOps
    import numpy as np

    from src.terrain_management.large_scale_terrain.collider_manager import ColliderManager, ColliderManagerCfg

    def is_map_done():
        return True

    def bindMaterial(stage, mtl_prim_path, prim_path):
        mtl_prim = stage.GetPrimAtPath(mtl_prim_path)
        prim = stage.GetPrimAtPath(prim_path)
        shade = UsdShade.Material(mtl_prim)
        UsdShade.MaterialBindingAPI(prim).Bind(shade, UsdShade.Tokens.strongerThanDescendants)

    world = World(stage_units_in_meters=1.0)
    stage = get_context().get_stage()
    asset_path = os.path.join(os.getcwd(), "assets")

    # Let there be light
    light = UsdLux.DistantLight.Define(stage, "/World/sun")
    light.CreateIntensityAttr(3000.0)
    addDefaultOps(light.GetPrim())
    setDefaultOps(light.GetPrim(), (0, 0, 0), (0.65, 0, 0, 0.76), (1, 1, 1))

    # Create a sphere
    sphere = UsdGeom.Sphere.Define(stage, "/World/sphere")
    addDefaultOps(sphere.GetPrim())
    setDefaultOps(sphere.GetPrim(), (0, 0, 0), (0, 0, 0, 1), (1, 1, 1))

    map = np.load("assets/Terrains/SouthPole/LM1_final_adj_5mpp_surf/dem.npy") * 0.1 / 5.0

    for i in range(100):
        world.step(render=True)

    CM_Cfg = ColliderManagerCfg(**CMCfg_D)
    CM = ColliderManager(CM_Cfg, map, map.shape, (map.shape[0] * 0.1 / 2, map.shape[1] * 0.1 / 2))

    CM.build()
    C = 0
    R = 50
    max_displacement = 1.0 / 30
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

    initial_position = (0, 0)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    last_position = initial_position

    while True:
        x = C + i2 * R * math.cos(theta[i])
        y = C + i2 * R * math.sin(theta[i])

        dist = math.sqrt((last_position[0] - x) ** 2 + (last_position[1] - y) ** 2)
        if dist > 2.0:
            last_position = (x, y)
            xu = int(x)
            yu = int(y)
            CM.update_collider((x, y))
        world.step(render=True)

        i += 1
        i = i % theta.shape[0]

    timeline.stop()
    simulation_app.close()
