from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni
import time
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.objects import VisualCuboid
import numpy as np

from lunalab import LabController

# Enables this ROS1 extension
enable_extension("omni.isaac.ros_bridge")
enable_extension("omni.kit.viewport.actions")
simulation_app.update()


# Checks that the master is running
import rosgraph
if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

# Loads ROS1 dependent libraries
import rospy
from std_msgs.msg import Bool, Float32, ColorRGBA, Int8, Int32, String, Empty
from geometry_msgs.msg import Pose


class ROS_LabManager:
    def __init__(self):
        # setting up the world
        self.LC = LabController()
        self.LC.load()

        self.projector_subs = []
        self.projector_subs.append(rospy.Subscriber("/Lunalab/Projector/TurnOn", Bool, self.setProjectorOn, queue_size=1))
        self.projector_subs.append(rospy.Subscriber("/Lunalab/Projector/Intensity", Float32, self.setProjectorIntensity, queue_size=1))
        self.projector_subs.append(rospy.Subscriber("/Lunalab/Projector/Radius", Float32, self.setProjectorRadius, queue_size=1))
        self.projector_subs.append(rospy.Subscriber("/Lunalab/Projector/Pose", Pose, self.setProjectorPose, queue_size=1))
        self.projector_subs.append(rospy.Subscriber("/Lunalab/Projector/Color", ColorRGBA, self.setProjectorColor, queue_size=1))
        self.ceiling_subs = []
        self.ceiling_subs.append(rospy.Subscriber("/Lunalab/CeilingLights/TurnOn", Bool, self.setCeilingOn, queue_size=1))
        self.ceiling_subs.append(rospy.Subscriber("/Lunalab/CeilingLights/Intensity", Float32, self.setCeilingIntensity, queue_size=1))
        self.ceiling_subs.append(rospy.Subscriber("/Lunalab/CeilingLights/Radius", Float32, self.setCeilingRadius, queue_size=1))
        self.ceiling_subs.append(rospy.Subscriber("/Lunalab/CeilingLights/FOV", Float32, self.setCeilingFOV, queue_size=1))
        self.ceiling_subs.append(rospy.Subscriber("/Lunalab/CeilingLights/Color", ColorRGBA, self.setCeilingColor, queue_size=1))
        self.curtains_subs = []
        self.curtains_subs.append(rospy.Subscriber("/Lunalab/Curtains/Extend", Bool, self.setCurtainsMode, queue_size=1))
        self.terrains_subs = []
        self.terrains_subs.append(rospy.Subscriber("/Lunalab/Terrain/Switch", Int8, self.switchTerrain, queue_size=1))
        self.terrains_subs.append(rospy.Subscriber("/Lunalab/Terrain/EnableRocks", Bool, self.enableRocks, queue_size=1))
        self.terrains_subs.append(rospy.Subscriber("/Lunalab/Terrain/RandomizeRocks", Int32, self.randomizeRocks, queue_size=1))
        self.terrains_subs.append(rospy.Subscriber("/Lunalab/Terrain/PlaceRocks", String, self.placeRocks, queue_size=1))
        self.render_subs = []
        self.render_subs.append(rospy.Subscriber("/Lunalab/Render/EnableRTXRealTime", Empty, self.useRTXRealTimeRender, queue_size=1))
        self.render_subs.append(rospy.Subscriber("/Lunalab/Render/EnableRTXInteractive", Empty, self.useRTXInteractiveRender, queue_size=1))
        self.render_subs.append(rospy.Subscriber("/Lunalab/LensFlare/EnableLensFlares", Bool, self.setLensFlareOn, queue_size=1))
        self.render_subs.append(rospy.Subscriber("/Lunalab/LensFlare/NumBlades", Int8, self.setLensFlareNumBlade, queue_size=1))
        self.render_subs.append(rospy.Subscriber("/Lunalab/LensFlare/Scale", Float32, self.setLensFlareScale, queue_size=1))
        self.render_subs.append(rospy.Subscriber("/Lunalab/LensFlare/ApertureRotation", Float32, self.setLensFlareApertureRotation, queue_size=1))
        self.render_subs.append(rospy.Subscriber("/Lunalab/LensFlare/FocalLength", Float32, self.setLensFlareFocalLength, queue_size=1))
        self.render_subs.append(rospy.Subscriber("/Lunalab/LensFlare/Fstop", Float32, self.setLensFlareFstop, queue_size=1))
        self.render_subs.append(rospy.Subscriber("/Lunalab/LensFlare/SensorAspectRatio", Float32, self.setLensFlareSensorAspectRatio, queue_size=1))
        self.render_subs.append(rospy.Subscriber("/Lunalab/LensFlare/SensorDiagonal", Float32, self.setLensFlareSensorDiagonal, queue_size=1))

        self.modifications = []

    def clearModifications(self):
        self.modifications = []
    def applyModifications(self):
        for mod in self.modifications:
            mod[0](mod[1])
        self.clearModifications()
    def reset(self):
        pass

    def setProjectorOn(self, data):
        self.modifications.append([self.LC.turnProjectorOnOff, data.data])
    def setProjectorIntensity(self, data):
        default_intensity = 120000000.0
        data = default_intensity*float(data.data)/100.0
        self.modifications.append([self.LC.setProjectorIntensity, data])
    def setProjectorRadius(self, data):
        self.modifications.append([self.LC.setProjectorRadius, data.data])
    def setProjectorColor(self, data):
        color = [data.r, data.g, data.b]
        self.modifications.append([self.LC.setProjectorColor, color])
    def setProjectorPose(self, data):
        position = [data.position.x, data.position.y, data.position.z]
        quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.modifications.append([self.LC.setProjectorPose, (position, quaternion)])

    def setCeilingOn(self, data):
        self.modifications.append([self.LC.turnRoomLightsOnOff, data.data])
    def setCeilingIntensity(self, data):
        self.modifications.append([self.LC.setRoomLightsIntensity, data.data])
    def setCeilingRadius(self, data):
        self.modifications.append([self.LC.setRoomLightsRadius, data.data])
    def setCeilingFOV(self, data):
        self.modifications.append([self.LC.setRoomLightsFOV, data.data])
    def setCeilingColor(self, data):
        color = [data.r, data.g, data.b]
        self.modifications.append([self.LC.setRoomLightsColor, color])

    def setCurtainsMode(self, data):
        self.modifications.append([self.LC.curtainsExtend, data.data])

    def switchTerrain(self, data):
        self.modifications.append([self.LC.switchTerrain, data.data])
    def enableRocks(self, data):
        self.modifications.append([self.LC.enableRocks, data.data])
    def randomizeRocks(self, data):
        data = int(data.data)
        self.modifications.append([self.LC.randomizeRocks, data])
    def placeRocks(self, data):
        self.modifications.append([self.LC.placeRocks, data.data])
    
    def useRTXRealTimeRender(self, data):
        self.modifications.append([self.LC.enableRTXRealTime, 0])
    def useRTXInteractiveRender(self, data):
        self.modifications.append([self.LC.enableRTXInteractive, 0])

    def setLensFlareOn(self, data):
        self.modifications.append([self.LC.enableLensFlare, data.data])
    def setLensFlareNumBlade(self, data):
        data = int(data.data)
        self.modifications.append([self.LC.setFlareNumBlades, data])
    def setLensFlareScale(self, data):
        data = float(data.data)
        self.modifications.append([self.LC.setFlareScale, data])
    def setLensFlareFstop(self, data):
        data = float(data.data)
        self.modifications.append([self.LC.setFlareFstop, data])
    def setLensFlareFocalLength(self, data):
        data = float(data.data)
        self.modifications.append([self.LC.setFlareFocalLength, data])
    def setLensFlareSensorAspectRatio(self, data):
        data = float(data.data)
        self.modifications.append([self.LC.setFlareSensorAspectRatio, data])
    def setLensFlareSensorDiagonal(self, data):
        data = float(data.data)
        self.modifications.append([self.LC.setFlareSensorDiagonal, data])
    def setLensFlareApertureRotation(self, data):
        data = float(data.data)
        self.modifications.append([self.LC.setFlareApertureRotation, data])

    def cleanScene(self):
        # Cleanup
        for sub in self.projector_subs:
            sub.unregister()
        for sub in self.ceiling_subs:
            sub.unregister()
        for sub in self.curtains_subs:
            sub.unregister()
        for sub in self.terrains_subs:
            sub.unregister()
        for sub in self.render_subs:
            sub.unregister()
        rospy.signal_shutdown("Shutting down")

class SimulationManager:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        self.ROSLabManager = ROS_LabManager()
        
    def run_simulation(self):
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)
            if self.world.is_playing():
                if self.world.current_time_step_index == 0:
                    self.world.reset()
                    self.ROSLabManager.reset()
                self.ROSLabManager.applyModifications()

        self.timeline.stop()
        simulation_app.close()


if __name__ == "__main__":
    rospy.init_node("lab_controller_node", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
    SM = SimulationManager()
    SM.run_simulation()
