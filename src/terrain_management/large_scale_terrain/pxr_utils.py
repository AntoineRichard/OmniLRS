from pxr import UsdGeom, Gf, Usd, Vt

from omni.physx.scripts import utils as physx_utils


def set_xform_op(prim: Usd.Prim, value, property: UsdGeom.XformOp.Type) -> None:
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


def set_scale(prim: Usd.Prim, value: Gf.Vec3d) -> None:
    """
    Sets the scale of a prim.

    Args:
        prim (Usd.Prim): The prim to set the scale.
        value (Gf.Vec3d): The value of the scale.
    """

    set_xform_op(prim, value, UsdGeom.XformOp.TypeScale)


def set_translate(prim: Usd.Prim, value: Gf.Vec3d) -> None:
    """
    Sets the translation of a prim.

    Args:
        prim (Usd.Prim): The prim to set the translation.
        value (Gf.Vec3d): The value of the translation.
    """

    set_xform_op(prim, value, UsdGeom.XformOp.TypeTranslate)


def set_rotate_xyz(prim: Usd.Prim, value: Gf.Vec3d) -> None:
    """
    Sets the rotation of a prim.

    Args:
        prim (Usd.Prim): The prim to set the rotation.
        value (Gf.Vec3d): The value of the rotation.
    """

    set_xform_op(prim, value, UsdGeom.XformOp.TypeRotateXYZ)


def set_orient(prim: Usd.Prim, value: Gf.Quatd) -> None:
    """
    Sets the rotation of a prim.

    Args:
        prim (Usd.Prim): The prim to set the rotation.
        value (Gf.Quatd): The value of the rotation.
    """

    set_xform_op(prim, value, UsdGeom.XformOp.TypeOrient)


def set_transform(prim, value: Gf.Matrix4d) -> None:
    """
    Sets the transform of a prim.

    Args:
        prim (Usd.Prim): The prim to set the transform.
        value (Gf.Matrix4d): The value of the transform.
    """

    set_xform_op(prim, value, UsdGeom.XformOp.TypeTransform)


def set_xform_ops(
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

    set_translate(prim, translate)
    set_orient(prim, orient)
    set_scale(prim, scale)


def get_transform(prim: Usd.Prim, parent: Usd.Prim) -> Gf.Matrix4d:
    """
    Gets the transform of a prim relative to its parent.

    Args:
        prim (Usd.Prim): The prim to get the transform.
        parent (Usd.Prim): The parent of the prim.
    """

    return UsdGeom.XformCache(0).ComputeRelativeTransform(prim, parent)[0]


def add_collider(
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
    assert mode in accepted_modes, "Decimation mode: " + mode + " for colliders unknown."
    # Get the prim and add collisions.
    prim = stage.GetPrimAtPath(path)
    physx_utils.setCollider(prim, approximationShape=mode)


def remove_collider(
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


def delete_prim(
    stage: Usd.Stage,
    path: str,
) -> None:
    """
    Deletes a prim.

    Args:
        stage (Usd.Stage): The stage.
        path (str): The path to the prim.
    """

    # Get the prim and delete it.
    prim = stage.GetPrimAtPath(path)
    stage.RemovePrim(prim)


def enable_smooth_shade(
    prim: Usd.Prim,
    extra_smooth: bool = False,
) -> None:
    """
    Enables smooth shading on a prim.

    Args:
        prim (Usd.Prim): The prim on which to enable smooth shading.
        extra_smooth (bool, optional): Toggles the use of the smooth
        method instead of catmullClark. Defaults to False.
    """

    # Sets the subdivision scheme to smooth.
    prim.GetAttribute("subdivisionScheme").Set(UsdGeom.Tokens.catmullClark)
    # Sets the triangle subdivision rule.
    if extra_smooth:
        prim.GetAttribute("triangleSubdivisionRule").Set(UsdGeom.Tokens.smooth)
    else:
        prim.GetAttribute("triangleSubdivisionRule").Set(UsdGeom.Tokens.catmullClark)
