__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import Tuple
import carb
import omni

from src.configurations.rendering_confs import FlaresConf, ChromaticAberrationsConf, MotionBlurConf


# ==============================================================================
# Renderer Control
# ==============================================================================


def enable_RTX_real_time(**kwargs) -> None:
    """
    Enables the RTX real time renderer. The ray-traced render.

    Args:
        data (int, optional): Not used. Defaults to 0.
    """

    action_registry = omni.kit.actions.core.get_action_registry()
    action = action_registry.get_action("omni.kit.viewport.actions", "set_renderer_rtx_realtime")
    action.execute()


def enable_RTX_interactive(*kwargs) -> None:
    """
    Enables the RTX interactive renderer. The path-traced render.

    Args:
        data (int, optional): Not used. Defaults to 0.
    """

    action_registry = omni.kit.actions.core.get_action_registry()
    action = action_registry.get_action("omni.kit.viewport.actions", "set_renderer_rtx_pathtracing")
    action.execute()


# ==============================================================================
# Lens Flares Controls
# ==============================================================================


def apply_lens_flare(flare_settings: FlaresConf = None, enable: bool = False) -> None:
    """
    Enables the lens flare effect.

    Args:
        enable (bool): True to enable the lens flare, False to disable it.
    """

    if enable:
        enable_lens_flare(True)
        set_flare_scale(flare_settings.scale)
        set_flare_num_blades(flare_settings.blades)
        set_flare_aperture_rotation(flare_settings.aperture_rotation)
        set_flare_sensor_aspect_ratio(flare_settings.sensor_aspect_ratio)
        set_flare_sensor_diagonal(flare_settings.sensor_diagonal)
        set_flare_fstop(flare_settings.fstop)
        set_flare_focal_length(flare_settings.focal_length)
    else:
        enable_lens_flare(False)


def enable_lens_flare(enable: bool = True) -> None:
    """
    Enables the lens flare effect.

    Args:
        enable (bool): True to enable the lens flare, False to disable it.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/enabled", enable)


def set_flare_scale(value: float = 0.0) -> None:
    """
    Sets the scale of the lens flare.

    Args:
        value (float): The scale of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/flareScale", value)


def set_flare_num_blades(value: int = 9) -> None:
    """
    Sets the number of blades of the lens flare.
    A small number will create sharp spikes, a large number will create a smooth circle.

    Args:
        value (int): The number of blades of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/blades", int(value))


def set_flare_aperture_rotation(value: float = 0.0) -> None:
    """
    Sets the rotation of the lens flare.

    Args:
        value (float): The rotation of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/apertureRotation", value)


def set_flare_sensor_diagonal(value: float = 28.0) -> None:
    """
    Sets the sensor diagonal of the lens flare.

    Args:
        value (float): The sensor diagonal of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/sensorDiagonal", value)


def set_flare_sensor_aspect_ratio(value: float = 1.5) -> None:
    """
    Sets the sensor aspect ratio of the lens flare.

    Args:
        value (float): The sensor aspect ratio of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/sensorAspectRatio", value)


def set_flare_fstop(value: float = 2.8) -> None:
    """
    Sets the f-stop of the lens flare.

    Args:
        value (float): The f-stop of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/fNumber", value)


def set_flare_focal_length(value: float = 12.0):
    """
    Sets the focal length of the lens flare.

    Args:
        value (float): The focal length of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/focalLength", value)


# ==============================================================================
# Chromatic Aberration Controls
# ==============================================================================


def apply_chromatic_aberrations(settings: ChromaticAberrationsConf = None, enable: bool = False) -> None:
    """
    Applies the chromatic aberration effect.

    Args:
        settings (ChromaticAberrationsConf): The settings of the chromatic aberration.
    """
    if enable:
        enable_chromatic_aberrations(True)
        set_chromatic_aberration_strength(settings.strength)
        set_chromatic_aberration_model(settings.model)
        set_chromatic_aberration_lanczos(settings.enableLanczos)
    else:
        enable_chromatic_aberrations(False)


def enable_chromatic_aberrations(enable: bool = True) -> None:
    """
    Enables the chromatic aberration effect.

    Args:
        enable (bool): True to enable the chromatic aberration, False to disable it.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/chromaticAberration/enabled", enable)


def set_chromatic_aberration_strength(value: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
    """
    Sets the strength of the chromatic aberration.

    Args:
        value (Tuple[float,float,float]): The strength of the chromatic aberration.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/chromaticAberration/strengthR", value[0])
    settings.set("/rtx/post/chromaticAberration/strengthG", value[1])
    settings.set("/rtx/post/chromaticAberration/strengthB", value[2])


def set_chromatic_aberration_model(value: Tuple[str, str, str] = ("Radial", "Radial", "Radial")) -> None:
    """
    Sets the model of the chromatic aberration.

    Args:
        value (Tuple[str,str,str]): The model of the chromatic aberration.
    """

    for model in value:
        if model not in ["Radial", "Barrel"]:
            raise ValueError(f"Invalid chromatic aberration model: {model}")

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/chromaticAberration/modelR", value[0])
    settings.set("/rtx/post/chromaticAberration/modelG", value[1])
    settings.set("/rtx/post/chromaticAberration/modelB", value[2])


def set_chromatic_aberration_lanczos(value: bool = False) -> None:
    """
    Sets the lanczos of the chromatic aberration.

    Args:
        value (bool): Set to True to enable lanczos, False to disable it.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/chromaticAberration/enableLanczos", value)


# ==============================================================================
# Motion Blur Controls
# ==============================================================================


def apply_motion_blur(settings: MotionBlurConf = None, enable: bool = False) -> None:
    """
    Applies the motion blur effect.

    Args:
        settings (MotionBlurConf): The settings of the motion blur.
    """

    if enable:
        enable_motion_blur(True)
        set_motion_blur_diameter_fraction(settings.max_blur_diameter_fraction)
        set_motion_blur_exposure_fraction(settings.exposure_fraction)
        set_motion_blur_num_samples(settings.num_samples)
    else:
        enable_motion_blur(False)


def enable_motion_blur(settings: bool = True) -> None:
    """
    Enables the motion blur effect.

    Args:
        enable (bool): True to enable the motion blur, False to disable it.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/motionblur/enabled", settings)


def set_motion_blur_diameter_fraction(value: float = 0.0) -> None:
    """
    Sets the diameter fraction of the motion blur.

    Args:
        value (float): The diameter fraction of the motion blur.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/motionblur/maxBlurDiameterFraction", value)


def set_motion_blur_exposure_fraction(value: float = 0.0) -> None:
    """
    Sets the exposure fraction of the motion blur.

    Args:
        value (float): The exposure fraction of the motion blur.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/motionblur/exposureFraction", value)


def set_motion_blur_num_samples(value: int = 8) -> None:
    """
    Sets the number of samples of the motion blur.

    Args:
        value (int): The number of samples of the motion blur.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/motionblur/numSamples", value)
