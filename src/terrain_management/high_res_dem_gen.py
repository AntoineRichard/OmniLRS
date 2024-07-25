from typing import List, Tuple
import numpy as np
import dataclasses
import threading
import queue
import copy

from crater_gen import CraterBuilder, CraterDB, CraterSampler
from crater_gen import (
    CraterBuilderCfg,
    CraterDBCfg,
    CraterSamplerCfg,
    CraterGeneratorCfg,
    CraterDynamicDistributionCfg,
)
from crater_gen import CraterMetadata, BoundingBox


class CraterBuilderWorker(threading.Thread):
    def __init__(
        self, builder: CraterBuilder, output_queue: queue.Queue, queue_size: int = 10
    ):
        threading.Thread.__init__(self)
        self.builder = builder
        self.input_queue = queue.Queue(maxsize=queue_size)
        self.output_queue = output_queue

    def get_input_queue_length(self):
        return self.input_queue.qsize()

    def is_input_queue_empty(self):
        return self.input_queue.empty()

    def is_input_queue_full(self):
        return self.input_queue.full()

    def run(self):
        while True:
            coords, crater_meta_data = self.input_queue.get()
            if crater_meta_data is None:
                break
            self.output_queue.put(
                (coords, self.builder.generateCraters(crater_meta_data, coords))
            )
            self.input_queue.task_done()


class CraterBuilderManager:
    def __init__(
        self,
        builder: CraterBuilder,
        num_workers: int = 1,
        input_queue_size: int = 10,
        output_queue_size: int = 10,
        worker_queue_size: int = 10,
    ) -> None:
        # Create the builder (it allows for the generation of craters)
        self.builder = builder

        # Create the input queue (it holds the crater metadata)
        self.input_queue = queue.Queue(maxsize=input_queue_size)
        # Create the output queue, shared by all workers (it holds the generated craters)
        self.output_queue = queue.Queue(maxsize=output_queue_size)

        # Create the workers and start them
        self.workers = [
            CraterBuilderWorker(
                builder, self.output_queue, queue_size=worker_queue_size
            )
            for _ in range(num_workers)
        ]
        for worker in self.workers:
            worker.start()

        # Create the manager thread
        self.manager_thread = threading.Thread(target=self.dispatch_jobs)
        self.manager_thread.start()

    def are_workers_done(self):
        return all([worker.is_input_queue_empty() for worker in self.workers])

    def get_load_per_worker(self):
        return {
            "worker_{}".format(i): worker.get_input_queue_length()
            for i, worker in enumerate(self.workers)
        }

    def get_output_queue_length(self):
        return self.output_queue.qsize()

    def is_output_queue_empty(self):
        return self.output_queue.empty()

    def is_output_queue_full(self):
        return self.output_queue.full()

    def get_shortest_queue_index(self):
        return min(
            range(len(self.workers)),
            key=lambda i: self.workers[i].get_input_queue_length(),
        )

    def dispatch_jobs(self):
        # Assigns the jobs such that the workers have a balanced load
        while True:
            coords, crater_metadata = self.input_queue.get()
            if crater_metadata is None:  # Check for shutdown signal
                break
            self.workers[self.get_shortest_queue_index()].input_queue.put(
                (coords, crater_metadata)
            )
            self.input_queue.task_done()

    def process_crater_meta_data(
        self, coords: Tuple[float, float], crater_metadata: List[CraterMetadata]
    ):
        # Add crater metadata to the main input queue
        self.input_queue.put((coords, crater_metadata))

    def collect_results(self):
        # Collect results from the workers
        print("Collecting results from workers...")
        results = []
        while not self.is_output_queue_empty():
            results.append(self.output_queue.get())
            self.output_queue.task_done()
        print(f"Collected {len(results)} results from workers.")
        return results

    def shutdown(self):
        print("Shutting down thread manager...")
        # Signal the manager thread to stop
        self.input_queue.put(((0, 0), None))
        self.manager_thread.join()
        print("Shutting down workers...")
        # Signal the worker threads to stop
        for worker in self.workers:
            worker.input_queue.put(((0, 0), None))
        for worker in self.workers:
            worker.join()

    def __del__(self):
        print("Entering destructor...")
        self.shutdown()


@dataclasses.dataclass
class HighResDEMGenCfg:
    num_blocks: int = dataclasses.field(default_factory=int)
    block_size: float = dataclasses.field(default_factory=float)
    pad_size: float = dataclasses.field(default_factory=float)
    max_blocks: int = dataclasses.field(default_factory=int)
    seed: int = dataclasses.field(default_factory=int)
    resolution: float = dataclasses.field(default_factory=float)
    z_scale: float = dataclasses.field(default_factory=float)

    radius: List[Tuple[float, float]] = dataclasses.field(default_factory=list)
    densities: List[float] = dataclasses.field(default_factory=list)
    num_repeat: int = dataclasses.field(default_factory=int)

    save_to_disk: bool = dataclasses.field(default_factory=bool)
    write_to_disk_interval: int = dataclasses.field(default_factory=int)

    profiles_path: str = dataclasses.field(default_factory=str)
    min_xy_ratio: float = dataclasses.field(default_factory=float)
    max_xy_ratio: float = dataclasses.field(default_factory=float)
    random_rotation: bool = dataclasses.field(default_factory=bool)
    num_unique_profiles: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        self.crater_db_cfg = CraterDBCfg(
            block_size=self.block_size,
            max_blocks=self.max_blocks,
            save_to_disk=self.save_to_disk,
            write_to_disk_interval=self.write_to_disk_interval,
        )
        CGC = {
            "profiles_path": self.profiles_path,
            "min_xy_ratio": self.min_xy_ratio,
            "max_xy_ratio": self.max_xy_ratio,
            "random_rotation": self.random_rotation,
            "seed": self.seed,
            "num_unique_profiles": self.num_unique_profiles,
        }
        CDDC = {
            "densities": self.densities,
            "radius": self.radius,
            "num_repeat": self.num_repeat,
            "seed": self.seed,
        }
        self.crater_sampler_cfg = CraterSamplerCfg(
            block_size=self.block_size,
            crater_gen_cfg=CGC,
            crater_dist_cfg=CDDC,
        )
        self.crater_builder_cfg = CraterBuilderCfg(
            block_size=self.block_size,
            pad_size=self.pad_size,
            resolution=self.resolution,
            z_scale=self.z_scale,
        )


class HighResDEMGen:
    def __init__(self, low_res_dem: np.ndarray, settings: HighResDEMGenCfg):
        self.low_res_dem = low_res_dem
        self.settings = settings

        self.current_block_coord = (0, 0)

        self.sim_lock = False
        self.terrain_is_primed = False
        self.crater_db_is_primed = False
        self.build()

    def build(self):
        self.crater_db = CraterDB(self.settings.crater_db_cfg)
        self.crater_sampler = CraterSampler(
            self.settings.crater_sampler_cfg, db=self.crater_db
        )
        self.crater_builder = CraterBuilder(
            self.settings.crater_builder_cfg, db=self.crater_db
        )
        self.crater_builder_manager = CraterBuilderManager(
            self.crater_builder,
            num_workers=8,
            input_queue_size=100,
            output_queue_size=10,
            worker_queue_size=100,
        )
        self.build_block_grid()
        self.instantiate_high_res_dem()

    def instantiate_high_res_dem(self):
        self.high_res_dem = np.zeros(
            (
                int(
                    (self.settings.num_blocks * 2 + 3)
                    * self.settings.block_size
                    / self.settings.resolution
                ),
                int(
                    (self.settings.num_blocks * 2 + 3)
                    * self.settings.block_size
                    / self.settings.resolution
                ),
            ),
            dtype=np.float32,
        )

    def cast_coordinates_to_block_space(self, coordinates: Tuple[float, float]):
        x, y = coordinates
        x_block = int(x // self.settings.block_size) * self.settings.block_size
        y_block = int(y // self.settings.block_size) * self.settings.block_size
        return (x_block, y_block)

    def build_block_grid(self):
        self.block_grid_tracker = {}
        self.map_grid_block2coords = {}

        # Instantiate empty state for each block in the grid
        state = {
            "has_crater_data": False,
            "has_terrain_data": False,
            "is_padding": False,
        }
        # Generate a grid that spans num_blocks in each direction plus 1 block for caching
        for x in range(-self.settings.num_blocks - 1, self.settings.num_blocks + 2, 1):
            x_c = x * self.settings.block_size
            x_i = x_c + self.current_block_coord[0]
            for y in range(
                -self.settings.num_blocks - 1, self.settings.num_blocks + 2, 1
            ):
                y_c = y * self.settings.block_size
                y_i = y_c + self.current_block_coord[1]
                self.block_grid_tracker[(x_c, y_c)] = copy.copy(state)
                if (x == -self.settings.num_blocks - 1) or (
                    x == self.settings.num_blocks + 1
                ):
                    self.block_grid_tracker[(x_c, y_c)]["is_padding"] = True
                elif (y == -self.settings.num_blocks - 1) or (
                    y == self.settings.num_blocks + 1
                ):
                    self.block_grid_tracker[(x_c, y_c)]["is_padding"] = True
                else:
                    self.block_grid_tracker[(x_c, y_c)]["is_padding"] = False
                self.map_grid_block2coords[(x_i, y_i)] = (x_c, y_c)

    def shift_block_grid(self, coordinates: Tuple[float, float]):
        new_block_grid_tracker = {}
        new_map_grid_block2coords = {}

        for x in range(-self.settings.num_blocks - 1, self.settings.num_blocks + 2, 1):
            x_c = x * self.settings.block_size
            x_i = x_c + coordinates[0]
            for y in range(
                -self.settings.num_blocks - 1, self.settings.num_blocks + 2, 1
            ):
                y_c = y * self.settings.block_size
                y_i = y_c + coordinates[1]

                # Check if the block is new or already in the map
                if (x_i, y_i) not in self.map_grid_block2coords:
                    # This block is not in the map, so it is a new block
                    new_block_grid_tracker[(x_c, y_c)] = {
                        "has_crater_data": False,
                        "has_terrain_data": False,
                        "is_padding": False,
                    }
                else:
                    # This block is already in the map
                    x_c_2, y_c_2 = self.map_grid_block2coords[(x_i, y_i)]
                    new_block_grid_tracker[(x_c, y_c)] = self.block_grid_tracker[
                        (x_c_2, y_c_2)
                    ]

                new_map_grid_block2coords[(x_i, y_i)] = (x_c, y_c)
                if (x == -self.settings.num_blocks - 1) or (
                    x == self.settings.num_blocks + 1
                ):
                    self.block_grid_tracker[(x_c, y_c)]["is_padding"] = True
                elif (y == -self.settings.num_blocks - 1) or (
                    y == self.settings.num_blocks + 1
                ):
                    self.block_grid_tracker[(x_c, y_c)]["is_padding"] = True
                else:
                    self.block_grid_tracker[(x_c, y_c)]["is_padding"] = False
                # print(
                #    f"Block {x_c, y_c} is padding: {self.block_grid_tracker[(x_c, y_c)]['is_padding']}"
                # )

        # Overwrite the old state with the new state
        self.block_grid_tracker = new_block_grid_tracker
        self.map_grid_block2coords = new_map_grid_block2coords

    def shift_dem(self, pixel_shift: Tuple[int, int]):
        x_shift, y_shift = pixel_shift
        if x_shift > self.high_res_dem.shape[0] or y_shift > self.high_res_dem.shape[1]:
            # If the shift is larger than the DEM, reset the DEM
            self.high_res_dem[:, :] = 0
        else:
            # Shift the DEM
            if x_shift > 0:
                x_min_s = 0
                x_max_s = -x_shift
                x_min_t = x_shift
                x_max_t = self.high_res_dem.shape[0]
            elif x_shift == 0:
                x_min_s = 0
                x_max_s = self.high_res_dem.shape[0]
                x_min_t = 0
                x_max_t = self.high_res_dem.shape[0]
            else:
                x_min_s = -x_shift
                x_max_s = self.high_res_dem.shape[0]
                x_min_t = 0
                x_max_t = x_shift
            if y_shift > 0:
                y_min_s = 0
                y_max_s = -y_shift
                y_min_t = y_shift
                y_max_t = self.high_res_dem.shape[1]
            elif y_shift == 0:
                y_min_s = 0
                y_max_s = self.high_res_dem.shape[1]
                y_min_t = 0
                y_max_t = self.high_res_dem.shape[1]
            else:
                y_min_s = -y_shift
                y_max_s = self.high_res_dem.shape[1]
                y_min_t = 0
                y_max_t = y_shift
            print(f"Source: {x_min_s, x_max_s, y_min_s, y_max_s}")
            print(f"Target: {x_min_t, x_max_t, y_min_t, y_max_t}")
            # Copy the data
            self.high_res_dem[x_min_t:x_max_t, y_min_t:y_max_t] = self.high_res_dem[
                x_min_s:x_max_s, y_min_s:y_max_s
            ]
            if x_shift < 0:
                self.high_res_dem[x_max_t:, :] = 0
            elif x_shift > 0:
                self.high_res_dem[:x_min_t, :] = 0
            if y_shift < 0:
                self.high_res_dem[:, y_max_t:] = 0
            elif y_shift > 0:
                self.high_res_dem[:, :y_min_t] = 0

    def shift(self, coordinates):
        # Compute initial coordinates in block space
        new_block_coord = self.cast_coordinates_to_block_space(coordinates)
        # Compute pixel shift between the new and old block coordinates
        delta_coord = (
            new_block_coord[0] - self.current_block_coord[0],
            new_block_coord[1] - self.current_block_coord[1],
        )
        pixel_shift = (
            -int(delta_coord[0] / self.settings.resolution),
            -int(delta_coord[1] / self.settings.resolution),
        )
        # Shift the block grid
        print("Shifting block grid...")
        self.shift_block_grid(new_block_coord)
        # print(self.block_grid_tracker)
        # Shift the DEM
        print("Shifting DEM...")
        print(f"Pixel shift: {pixel_shift}")
        self.shift_dem(pixel_shift)
        # Sets the current block coordinates to the new one.
        self.current_block_coord = new_block_coord
        # Trigger terrain update
        # Generate crater metadata for the new blocks
        print("Generating crater metadata...")
        self.generate_craters_metadata()
        # Asynchronous terrain block generation
        print("Generating terrain blocks...")
        # print(self.block_grid_tracker)
        self.generate_terrain_blocks()
        print("Done feeding queues...")

    def trigger_terrain_update(self):
        # You want to trigger an update if the robot moved enough into the next block
        # I.e. we want to avoid cases where the robot is at the edge of a block and the
        # terrain data keeps getting updated.
        # Something like 2 meters into the next block should be enough, and to go back
        # to the previous block, the robot would have to move 4 meters back.
        pass

    def generate_craters_metadata(self):
        # Generate crater metadata at for + 2 blocks in each direction
        region = BoundingBox(
            x_min=int(
                self.current_block_coord[0]
                - (self.settings.num_blocks + 2) * self.settings.block_size
            ),
            x_max=int(
                self.current_block_coord[0]
                + (self.settings.num_blocks + 2) * self.settings.block_size
            ),
            y_min=int(
                self.current_block_coord[1]
                - (self.settings.num_blocks + 2) * self.settings.block_size
            ),
            y_max=int(
                self.current_block_coord[1]
                + (self.settings.num_blocks + 2) * self.settings.block_size
            ),
        )
        self.crater_sampler.sample_craters_by_region(region)
        # Check if the block has crater data (it should always be true)
        for coords in self.map_grid_block2coords.keys():
            exists = self.crater_db.check_block_exists(coords)
            local_coords = self.map_grid_block2coords[coords]
            if exists:
                self.block_grid_tracker[local_coords]["has_crater_data"] = True
            else:
                self.block_grid_tracker[local_coords]["has_crater_data"] = False
                print(f"Block {coords} does not have crater data")

    def generate_terrain_blocks(self):
        # Generate terrain data for + 1 block in each direction
        for grid_key in self.map_grid_block2coords.keys():
            x_i = grid_key[0]  # + self.current_block_coord[0]
            y_i = grid_key[1]  # + self.current_block_coord[1]
            coords = (x_i, y_i)
            grid_coords = self.map_grid_block2coords[grid_key]
            if self.block_grid_tracker[grid_coords]["has_terrain_data"]:
                continue
            # Generate terrain data
            self.crater_builder_manager.process_crater_meta_data(
                coords, self.crater_db.get_block_data(coords)
            )

    def collect_terrain_data(self):
        # print("Collecting terrain data...")
        results = self.crater_builder_manager.collect_results()
        # print("Updating DEM...")
        offset = int(
            (self.settings.num_blocks + 1)
            * self.settings.block_size
            / self.settings.resolution
        )
        # print(f"Offset: {offset}")
        # print(f"Shape: {self.high_res_dem.shape}")
        for coords, data in results:
            # print(coords)
            x_i, y_i = coords
            x_c, y_c = self.map_grid_block2coords[(x_i, y_i)]
            local_coords = (x_c, y_c)
            # print(f"x_coordinates: {x_c / self.settings.resolution + offset}")
            # print(f"y_coordinates: {y_c / self.settings.resolution + offset}")
            # print(
            #    f"load per worker: {self.crater_builder_manager.get_load_per_worker()}"
            # )
            # if self.block_grid_tracker[coords]["is_padding"]:
            #    print(self.block_grid_tracker[coords])
            #    print("Skipping padding block...")
            #    continue
            self.high_res_dem[
                int(x_c / self.settings.resolution)
                + offset : int(
                    (x_c + self.settings.block_size) / self.settings.resolution
                )
                + offset,
                int(y_c / self.settings.resolution)
                + offset : int(
                    (y_c + self.settings.block_size) / self.settings.resolution
                )
                + offset,
            ] = data[0]
            self.block_grid_tracker[local_coords]["has_terrain_data"] = True

    def __del__(self):
        self.crater_builder_manager.shutdown()


if __name__ == "__main__":

    HRDEMGCfg_D = {
        "num_blocks": 2,  # int = dataclasses.field(default_factory=int)
        "block_size": 50,  # float = dataclasses.field(default_factory=float)
        "pad_size": 10.0,  # float = dataclasses.field(default
        "max_blocks": int(1e7),  # int = dataclasses.field(default_factory=int)
        "seed": 42,  # int = dataclasses.field(default_factory=int)
        "resolution": 0.05,  # float = dataclasses.field(default_factory=float)
        "z_scale": 1.0,  # float = dataclasses.field(default_factory=float)
        "radius": [
            [1.5, 2.5],
            [0.75, 1.5],
            [0.25, 0.5],
        ],  # List[Tuple[float, float]] = dataclasses.field(default_factory=list)
        "densities": [
            0.025,
            0.05,
            0.5,
        ],  # List[float] = dataclasses.field(default_factory=list)
        "num_repeat": 1,  # int = dataclasses.field(default_factory=int)
        "save_to_disk": False,  # bool = dataclasses.field(default_factory=bool)
        "write_to_disk_interval": 100,  # int = dataclasses.field(default_factory=int)
        "profiles_path": "assets/Terrains/crater_spline_profiles.pkl",  # str = dataclasses.field(default_factory=str)
        "min_xy_ratio": 0.85,  # float = dataclasses.field(default_factory=float)
        "max_xy_ratio": 1.0,  # float = dataclasses.field(default_factory=float)
        "random_rotation": True,  # bool = dataclasses.field(default_factory=bool)
        "num_unique_profiles": 10000,  # int = dataclasses.field(default_factory=int)
    }

    settings = HighResDEMGenCfg(**HRDEMGCfg_D)
    low_res_dem = None

    HRDEMGen = HighResDEMGen(low_res_dem, settings)
    import time
    from matplotlib import pyplot as plt
    import matplotlib.colors as mcolors

    HRDEMGen.shift((0, 0))
    time.sleep(2.0)
    plt.figure()
    im_data = None
    while (not HRDEMGen.crater_builder_manager.is_output_queue_empty()) or (
        not HRDEMGen.crater_builder_manager.are_workers_done()
    ):
        HRDEMGen.collect_terrain_data()
        norm = mcolors.Normalize(
            vmin=HRDEMGen.high_res_dem.min(), vmax=HRDEMGen.high_res_dem.max()
        )
        if im_data is None:
            im_data = plt.imshow(HRDEMGen.high_res_dem, cmap="terrain", norm=norm)
        else:
            im_data.set_data(HRDEMGen.high_res_dem)
            im_data.set_norm(norm)
        plt.pause(0.25)
        plt.draw()
        print("Collecting terrain data...")
    HRDEMGen.shift((50, 0))
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(10.0)
    while (not HRDEMGen.crater_builder_manager.is_output_queue_empty()) or (
        not HRDEMGen.crater_builder_manager.are_workers_done()
    ):
        HRDEMGen.collect_terrain_data()
        norm = mcolors.Normalize(
            vmin=HRDEMGen.high_res_dem.min(), vmax=HRDEMGen.high_res_dem.max()
        )
        im_data.set_data(HRDEMGen.high_res_dem)
        im_data.set_norm(norm)
        plt.pause(0.25)
        plt.draw()
        print("Collecting terrain data...")
    time.sleep(10.0)
    HRDEMGen.collect_terrain_data()
    time.sleep(0.25)
    print("Done collecting terrain data...")
    HRDEMGen.shift((50, 50))
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(10.0)
    while (not HRDEMGen.crater_builder_manager.is_output_queue_empty()) or (
        not HRDEMGen.crater_builder_manager.are_workers_done()
    ):
        HRDEMGen.collect_terrain_data()
        norm = mcolors.Normalize(
            vmin=HRDEMGen.high_res_dem.min(), vmax=HRDEMGen.high_res_dem.max()
        )
        im_data.set_data(HRDEMGen.high_res_dem)
        im_data.set_norm(norm)
        plt.pause(0.25)
        plt.draw()
        print("Collecting terrain data...")
    time.sleep(10.0)
    HRDEMGen.collect_terrain_data()
    time.sleep(0.25)
    print("Done collecting terrain data...")
    HRDEMGen.shift((100, 100))
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(10.0)
    while (not HRDEMGen.crater_builder_manager.is_output_queue_empty()) or (
        not HRDEMGen.crater_builder_manager.are_workers_done()
    ):
        HRDEMGen.collect_terrain_data()
        norm = mcolors.Normalize(
            vmin=HRDEMGen.high_res_dem.min(), vmax=HRDEMGen.high_res_dem.max()
        )
        im_data.set_data(HRDEMGen.high_res_dem)
        im_data.set_norm(norm)
        plt.pause(0.25)
        plt.draw()
        print("Collecting terrain data...")
    time.sleep(10.0)
    HRDEMGen.collect_terrain_data()
    time.sleep(0.25)
    print("Done collecting terrain data...")
    HRDEMGen.shift((0, 0))
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(10.0)
    while (not HRDEMGen.crater_builder_manager.is_output_queue_empty()) or (
        not HRDEMGen.crater_builder_manager.are_workers_done()
    ):
        HRDEMGen.collect_terrain_data()
        norm = mcolors.Normalize(
            vmin=HRDEMGen.high_res_dem.min(), vmax=HRDEMGen.high_res_dem.max()
        )
        im_data.set_data(HRDEMGen.high_res_dem)
        im_data.set_norm(norm)
        plt.pause(0.25)
        plt.draw()
        print("Collecting terrain data...")
    time.sleep(10.0)
    HRDEMGen.collect_terrain_data()
    norm = mcolors.Normalize(
        vmin=HRDEMGen.high_res_dem.min(), vmax=HRDEMGen.high_res_dem.max()
    )
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    print("Done collecting terrain data...")
