from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni
import os
import numpy as np
from pxr import UsdGeom, Gf, Sdf, UsdPhysics, UsdShade, Usd, Vt
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension



enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.kit.viewport.actions")
simulation_app.update()


def createXform(stage, path):
    prim_path = omni.usd.get_stage_next_free_path(stage, path, False)
    obj_prim = stage.DefinePrim(prim_path, "Xform")
    return obj_prim, prim_path

def setProperty(xform: UsdGeom.Xformable, value, property,) -> None:
    op = None
    for xformOp in xform.GetOrderedXformOps():
        if xformOp.GetOpType() == property:
            op = xformOp
    if op:
        xform_op = op
    else:
        xform_op = xform.AddXformOp(property, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op.Set(value)

def setScale(xform: UsdGeom.Xformable, value) -> None:
    setProperty(xform, value, UsdGeom.XformOp.TypeScale)

def setTranslate(xform: UsdGeom.Xformable, value) -> None:
    setProperty(xform, value, UsdGeom.XformOp.TypeTranslate)

def setRotateXYZ(xform: UsdGeom.Xformable, value) -> None:
    setProperty(xform, value, UsdGeom.XformOp.TypeRotateXYZ)

def setTransform(xform: UsdGeom.Xformable, value: Gf.Matrix4d) -> None:
    setProperty(xform, value, UsdGeom.XformOp.TypeTransform)

def getTransform(rotation: Gf.Rotation, position: Gf.Vec3d) -> Gf.Matrix4d:
    matrix_4d = Gf.Matrix4d().SetTranslate(position)
    matrix_4d.SetRotateOnly(rotation)
    return matrix_4d

def createObject(prefix, stage, path, position=Gf.Vec3d(0, 0, 0), rotation=Gf.Rotation(Gf.Vec3d(0,0,1), 0), scale=Gf.Vec3d(1,1,1), is_instance=True) -> tuple:
    obj_prim, prim_path = createXform(stage, prefix)
    obj_prim.GetReferences().AddReference(path)
    if is_instance:
        obj_prim.SetInstanceable(True)
    xform = UsdGeom.Xformable(obj_prim)
    setScale(xform, scale)
    setTransform(xform, getTransform(rotation, position))
    return obj_prim, prim_path

def updateExtent(stage, instancer_path):
    instancer = UsdGeom.PointInstancer.Get(stage, instancer_path)
    extent = instancer.ComputeExtentAtTime(Usd.TimeCode(0), Usd.TimeCode(0))
    instancer.CreateExtentAttr(Vt.Vec3fArray([
        Gf.Vec3f(extent[0]),
        Gf.Vec3f(extent[1]),
    ]))

def createStandaloneInstance(stage, path):
    instancer = UsdGeom.PointInstancer.Define(stage, path)
    return instancer

def createInstancerAndCache(stage, path, asset_list):
    instancer = createStandaloneInstance(stage, path)
    createXform(stage, os.path.join(path,'cache'))
    for asset in asset_list:
        prim, prim_path = createObject(os.path.join(path,'cache','instance'), stage, asset)
        instancer.GetPrototypesRel().AddTarget(prim_path)
    # Set some dummy parameters
    setInstancerParameters(stage, path, pos=np.zeros((1,3)))

def setInstancerParameters(stage, path, pos: np.ndarray([],dtype=np.float), ids: np.ndarray([],dtype=int) = None, scale: np.ndarray([],dtype=np.float) = None, quat: np.ndarray([],dtype=np.float) = None):
    num = pos.shape[0]
    instancer_prim = stage.GetPrimAtPath(path)
    num_prototypes = len(instancer_prim.GetRelationship("prototypes").GetTargets())
    instancer_prim.GetAttribute("positions").Set(pos)
    if scale is None:
        scale = np.ones_like(pos)
    instancer_prim.GetAttribute("scales").Set(scale)
    if quat is None:
        quat = np.zeros((pos.shape[0],4))
        quat[:,-1] = 1
    instancer_prim.GetAttribute("orientations").Set(quat)
    if ids is None:
        ids=  (np.random.rand(num) * num_prototypes).astype(int)
    instancer_prim.GetAttribute("protoIndices").Set(ids)
    updateExtent(stage, path)


from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage
from src.robots.robot import RobotManager, Robot
from pxr import PhysicsSchemaTools


stage = omni.usd.get_context().get_stage()
root = "/home/antoine/Documents/Lunalab/Assets/Rocks"

world = World(stage_units_in_meters=1.0)

world.step()
PhysicsSchemaTools.addGroundPlane(stage, "/groundPlane", "Z", 100, Gf.Vec3f(0, 0, 0), Gf.Vec3f(1.0))
timeline = omni.timeline.get_timeline_interface()
timeline.play()
spawning_pose_list = [{"position":Gf.Vec3d(1.5,0.5,2.0),"orientation":Gf.Rotation(Gf.Vec3d(0,0,0),1)},
                      {"position":Gf.Vec3d(2.5,0.5,2.0),"orientation":Gf.Rotation(Gf.Vec3d(0,0,0),1)},
                      {"position":Gf.Vec3d(3.5,0.5,2.0),"orientation":Gf.Rotation(Gf.Vec3d(0,0,0),1)},
                      {"position":Gf.Vec3d(4.5,0.5,2.0),"orientation":Gf.Rotation(Gf.Vec3d(0,0,0),1)},
                      {"position":Gf.Vec3d(5.5,0.5,2.0),"orientation":Gf.Rotation(Gf.Vec3d(0,0,0),1)}]

Robs = RobotManager(spawning_poses=spawning_pose_list, is_ROS2=True)
Robs.addRobot("/home/antoine/Documents/Lunalab/Robots/Loe_revor.usd", "/Leo_01", 42)
Robs.addRobot("/home/antoine/Documents/Lunalab/Robots/Loe_revor.usd", "/Leo_02", 42)
Robs.addRobot("/home/antoine/Documents/Lunalab/Robots/Loe_revor.usd", "/Leo_03", 42)
#Rob = Robot("/home/antoine/Documents/Lunalab/Robots/Loe_revor.usd", "leo_01", robots_root="/")
#Rob.load(Gf.Vec3d(1,1,2), Gf.Rotation(Gf.Vec3d(0,0,0), 1))
world.step(render=True)

i = 0
while simulation_app.is_running():
    world.step(render=True)
    i += 1
    if i % 120 == 0:
        Robs.resetRobots()
