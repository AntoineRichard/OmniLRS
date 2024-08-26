from pxr import UsdGeom, Gf, Usd

from omni.physx.scripts import utils as physx_utils


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


def addCollision(
    stage: Usd.Stage,
    path: str,
    mode: str = "none",
) -> None:
    """
    Adds a collision to a prim.

    Args:
        stage (Usd.Stage): The stage.
        path (str): The path to the prim.
        mode (str, optional): The mode of the collision. Defaults to "none".
    """

    # Checks that the mode selected by the user is correct.
    accepted_modes = [
        "none",
        "convexHull",
        "convexDecomposition",
        "meshSimplification",
        "boundingSphere",
        "boundingCube",
    ]
    assert mode in accepted_modes, (
        "Decimation mode: " + mode + " for colliders unknown."
    )
    # Get the prim and add collisions.
    prim = stage.GetPrimAtPath(path)
    physx_utils.setCollider(prim, approximationShape=mode)


def removeCollision(
    stage: Usd.Stage,
    path: str,
) -> None:
    """
    Removes a collision from a prim.

    Args:
        stage (Usd.Stage): The stage.
        path (str): The path to the prim.
    """

    # Get the prim and remove collisions.
    prim = stage.GetPrimAtPath(path)
    physx_utils.removeCollider(prim)
