# System imports
from threading import Thread
import numpy as np
import time
import sys
import os

# Isaac sim SDK imports
from omni.isaac.kit import SimulationApp
# Start the simulation
renderer = "PathTracing"
renderer = "RayTracedLighting"

CONFIG = {
    "samples_per_pixel_per_frame":32, 
    "max_bounces": 4, 
    "max_specular_transmission_bounces":6, 
    "max_volume_bounces": 4, 
    "subdiv_refinement_level": 0, 
    "renderer": renderer,
    "headless":True,
}

simulation_app = SimulationApp(CONFIG)
# Once the sim is started load isaac libs (including ROS)
import omni
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core import World
from pxr import Gf
# Custom libs
from src.environments.lunaryard_capture import LabController
from src.labeling.auto_label import AutonomousLabeling
from assets import get_assets_path
WORKINGDIR = get_assets_path()

simulation_app.update()

LAB_X_MIN = 0.5
LAB_X_MAX = 39.5
LAB_Y_MIN = 0.5
LAB_Y_MAX = 39.5
MPP = 0.04

terrain_settings = {"crater_spline_profiles": WORKINGDIR+"/Terrains/crater_spline_profiles.pkl",
                    "dems_path": WORKINGDIR+"/Terrains/Lunaryard",
                    "sim_length": LAB_Y_MAX - LAB_Y_MIN + 1,
                    "sim_width": LAB_X_MAX - LAB_X_MIN + 1,
                    "lab_x_min": LAB_X_MIN,
                    "lab_x_max": LAB_X_MAX,
                    "lab_y_min": LAB_Y_MIN,
                    "lab_y_max": LAB_Y_MAX,
                    "resolution": MPP,
                    "mesh_pos": (0.0, 0.0, 0.0),
                    "mesh_rot": (0.0, 0.0, 0.0, 1.0),
                    "mesh_scale": (1.0, 1.0, 1.0),
                    "max_elevation": 1.0,
                    "min_elevation": -1.0,
                    "z_scale": 1,
                    "pad":500,
                    "num_repeat":0,
                    "densities": [0.025, 0.05, 0.5],
                    "radius": [(1.5,2.5), (0.75,1.5), (0.25,0.5)],
                    "root_path": "/Lunaryard",
                    "texture_path": "/Lunaryard/Looks/Basalt",
                    "seed": 42,
                    "is_yard": True,
                    "is_lab": False}

class SimulationManager:
    """"
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the simulation
    - Cleaning the simulation"""

    def __init__(self) -> None:
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        self.physics_ctx = self.world.get_physics_context()
        self.physics_ctx.set_solver_type("PGS")
        # Lab manager thread
        print("Before ROS")
        self.world.reset()
        self.LC = LabController(terrain_settings=terrain_settings)
        self.LC.load()
        print("After lab is loaded")
        for i in range(100):
            self.world.step(render=True)
        print("After world reset")
        self.AL = AutonomousLabeling("/Lunaryard/Camera")
        self.AL.load()
        
    def run_simulation(self) -> None:
        """
        Runs the simulation."""
        
        self.timeline.play()
        i = 0
        while simulation_app.is_running():
            self.world.step(render=True)
            if self.world.is_playing():
                try:
                    self.AL.record()
                except:
                    pass
                self.LC.randomizeSun()
                self.LC.randomizeEarth()
                # Apply modifications to the lab only once the simulation step is finished
                self.LC.randomizeCamera()
                if i%100 == 0:
                    self.LC.randomizeRocks()
                if i%1000 == 0:
                    self.LC.switchTerrain(-1)
            i+=1
        self.timeline.stop()
        simulation_app.close()


if __name__ == "__main__":
    SM = SimulationManager()
    SM.run_simulation()
