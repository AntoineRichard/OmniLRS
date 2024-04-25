import dataclasses

@dataclasses.dataclass
class DEMProceduralSRSpecs:
    meters_per_pixel: float = 0.01
    heightmap_size: float = 20.0
    num_lods_levels: int = 4
    start_position: tuple = (20000, 20000)

class ProceduralTerrainDatabase:
    def __init__(self):
        self.chunks = {}
        self.craters_data = 

class DEMProceduralSR:
    def __init__(self, dem_specs, procedural_sr_specs):
        # Load DEM
        # Get the region to perform SR
        # Get that a crop of size 