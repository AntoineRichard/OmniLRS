from typing import List, Tuple
import multiprocessing
import numpy as np
import dataclasses
import threading
import time
import copy
import cv2

from src.terrain_management.crater_gen import CraterBuilder, CraterDB, CraterSampler
from src.terrain_management.crater_gen import (
    CraterBuilderCfg,
    CraterDBCfg,
    CraterSamplerCfg,
)
from src.terrain_management.crater_gen import CraterMetadata, BoundingBox


@dataclasses.dataclass
class InterpolatorCfg:
    source_resolution: float = dataclasses.field(default_factory=float)
    target_resolution: float = dataclasses.field(default_factory=float)
    source_padding: int = dataclasses.field(default_factory=int)
    method: str = dataclasses.field(default_factory=str)

    def __post_init__(self):
        self.method = self.method.lower()
        self.fx = self.source_resolution / self.target_resolution
        self.fy = self.source_resolution / self.target_resolution
        self.target_padding = int(self.source_padding * self.fx)
        if self.source_padding < 2:
            print("Warning: Padding may be too small for interpolation.")
            self.source_padding = 2

        if self.method == "bicubic":
            self.method = cv2.INTER_CUBIC
            if self.fx < 1.0:
                print(
                    "Warning: Bicubic interpolation with downscaling. Consider using a different method."
                )
        elif self.method == "nearest":
            self.method = cv2.INTER_NEAREST
        elif self.method == "linear":
            self.method = cv2.INTER_LINEAR
        elif self.method == "area":
            self.method = cv2.INTER_AREA
            if self.fx > 1.0:
                print(
                    "Warning: Area interpolation with upscaling. Consider using a different method."
                )
        else:
            raise ValueError(f"Invalid interpolation method: {self.method}")


class Interpolator:
    def __init__(self, settings: InterpolatorCfg):
        self.settings = settings

    def interpolate(self, data):
        raise NotImplementedError


class CPUInterpolator(Interpolator):
    def __init__(self, settings: InterpolatorCfg):
        super().__init__(settings)

    def interpolate(self, data):
        return cv2.resize(
            data,
            (0, 0),
            fx=self.settings.fx,
            fy=self.settings.fy,
            interpolation=self.settings.method,
        )[
            int(self.settings.source_padding * self.settings.fx) : -int(
                self.settings.source_padding * self.settings.fy
            ),
            int(self.settings.source_padding * self.settings.fx) : -int(
                self.settings.source_padding * self.settings.fy
            ),
        ]


class BaseWorker(multiprocessing.Process):
    def __init__(
        self, queue_size: int, output_queue: multiprocessing.JoinableQueue, **kwargs
    ):
        super().__init__()
        self.stop_event = multiprocessing.Event()
        self.input_queue = multiprocessing.JoinableQueue(maxsize=queue_size)
        self.output_queue = output_queue
        self.daemon = True

    def get_input_queue_length(self):
        return self.input_queue.qsize()

    def is_input_queue_empty(self):
        return self.input_queue.empty()

    def is_input_queue_full(self):
        return self.input_queue.full()

    def run(self):
        raise NotImplementedError


@dataclasses.dataclass
class WorkerManagerCfg:
    num_workers: int = dataclasses.field(default_factory=int)
    input_queue_size: int = dataclasses.field(default_factory=int)
    output_queue_size: int = dataclasses.field(default_factory=int)
    worker_queue_size: int = dataclasses.field(default_factory=int)


class BaseWorkerManager:
    def __init__(
        self,
        settings: WorkerManagerCfg = WorkerManagerCfg(),
        worker_class: BaseWorker = None,
        **kwargs,
    ):
        self.input_queue = multiprocessing.JoinableQueue(
            maxsize=settings.input_queue_size
        )
        self.output_queue = multiprocessing.JoinableQueue(
            maxsize=settings.output_queue_size
        )

        self.num_workers = settings.num_workers
        self.worker_class = worker_class
        self.worker_queue_size = settings.worker_queue_size
        self.kwargs = kwargs
        self.stop_event = multiprocessing.Event()

        self.instantiate_workers(
            num_workers=self.num_workers,
            worker_queue_size=self.worker_queue_size,
            worker_class=self.worker_class,
            **self.kwargs,
        )

        self.manager_thread = multiprocessing.Process(
            target=self.dispatch_jobs,
        )
        self.manager_thread.daemon = True
        self.manager_thread.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("Exception caught, shutting down workers and main thread.")
        self.stop_event.set()
        self.shutdown()

    def instantiate_workers(
        self,
        num_workers: int = 1,
        worker_queue_size: int = 10,
        worker_class: BaseWorker = None,
        **kwargs,
    ):
        self.workers = [
            worker_class(worker_queue_size, self.output_queue, **kwargs)
            for _ in range(num_workers)
        ]
        for worker in self.workers:
            worker.start()

    def get_load_per_worker(self):
        return {
            "worker_{}".format(i): worker.get_input_queue_length()
            for i, worker in enumerate(self.workers)
        }

    def get_input_queue_length(self):
        return self.input_queue.qsize()

    def get_output_queue_length(self):
        return self.output_queue.qsize()

    def is_input_queue_empty(self):
        return self.input_queue.empty()

    def is_output_queue_empty(self):
        return self.output_queue.empty()

    def is_input_queue_full(self):
        return self.input_queue.full()

    def is_output_queue_full(self):
        return self.output_queue.full()

    def get_shortest_queue_index(self):
        return min(
            range(len(self.workers)),
            key=lambda i: self.workers[i].get_input_queue_length(),
        )

    def are_workers_done(self):
        return all([worker.is_input_queue_empty() for worker in self.workers])

    def get_load_per_worker(self):
        return {
            "worker_{}".format(i): worker.get_input_queue_length()
            for i, worker in enumerate(self.workers)
        }

    def dispatch_jobs(self):
        raise NotImplementedError

    def process_data(self, coords, data):
        self.input_queue.put((coords, data))

    def collect_results(self):
        results = []
        num = self.get_output_queue_length()
        for _ in range(num):
            results.append(self.output_queue.get())
            self.output_queue.task_done()
        return results

    def shutdown(self):
        self.input_queue.put(((0, 0), None))
        self.manager_thread.join()
        for worker in self.workers:
            worker.input_queue.put(((0, 0), None))
        for worker in self.workers:
            worker.join()

    def __del__(self):
        self.shutdown()


class CraterBuilderWorker(BaseWorker):
    def __init__(
        self,
        queue_size: int = 10,
        output_queue: multiprocessing.Queue = multiprocessing.JoinableQueue(),
        builder: CraterBuilder = None,
    ):
        super().__init__(queue_size, output_queue)
        self.builder = copy.copy(builder)

    def run(self):
        while not self.stop_event.is_set():
            coords, crater_meta_data = self.input_queue.get()
            if crater_meta_data is None:
                self.input_queue.task_done()
                break
            self.output_queue.put(
                (coords, self.builder.generateCraters(crater_meta_data, coords))
            )
            self.input_queue.task_done()


class CraterBuilderManager(BaseWorkerManager):
    def __init__(
        self,
        settings: WorkerManagerCfg = WorkerManagerCfg(),
        builder: CraterBuilder = None,
    ) -> None:
        super().__init__(
            settings=settings,
            worker_class=CraterBuilderWorker,
            builder=builder,
        )

    def dispatch_jobs(self):
        # Assigns the jobs such that the workers have a balanced load
        while not self.stop_event.is_set():
            coords, crater_metadata = self.input_queue.get()
            if crater_metadata is None:  # Check for shutdown signal
                self.input_queue.task_done()
                break
            self.workers[self.get_shortest_queue_index()].input_queue.put(
                (coords, crater_metadata)
            )
            self.input_queue.task_done()


class BicubicInterpolatorWorker(BaseWorker):
    def __init__(
        self,
        queue_size: int = 10,
        output_queue: multiprocessing.JoinableQueue = multiprocessing.JoinableQueue(),
        interp: Interpolator = None,
    ):
        super().__init__(queue_size, output_queue)
        self.interpolator = copy.copy(interp)

    def run(self):
        while not self.stop_event.is_set():
            coords, data = self.input_queue.get()
            if data is None:
                break
            self.output_queue.put((coords, self.interpolator.interpolate(data)))
            self.input_queue.task_done()


class BicubicInterpolatorManager(BaseWorkerManager):
    def __init__(
        self,
        settings: WorkerManagerCfg = WorkerManagerCfg(),
        interp: Interpolator = None,
        num_cv2_threads: int = 4,
    ):
        super().__init__(
            settings=settings,
            worker_class=BicubicInterpolatorWorker,
            interp=interp,
        )
        cv2.setNumThreads(num_cv2_threads)

    def dispatch_jobs(self):
        while not self.stop_event.is_set():
            coords, data = self.input_queue.get()
            if data is None:
                break
            self.workers[self.get_shortest_queue_index()].input_queue.put(
                (coords, data)
            )
            self.input_queue.task_done()


@dataclasses.dataclass
class HighResDEMCfg:
    num_blocks: int = dataclasses.field(default_factory=int)
    block_size: float = dataclasses.field(default_factory=float)
    pad_size: float = dataclasses.field(default_factory=float)
    max_blocks: int = dataclasses.field(default_factory=int)
    seed: int = dataclasses.field(default_factory=int)
    resolution: float = dataclasses.field(default_factory=float)
    z_scale: float = dataclasses.field(default_factory=float)
    source_resolution: float = dataclasses.field(default_factory=float)
    resolution: float = dataclasses.field(default_factory=float)
    interpolation_padding: int = dataclasses.field(default_factory=int)
    generate_craters: bool = dataclasses.field(default_factory=bool)


@dataclasses.dataclass
class HighResDEMGenCfg:
    high_res_dem_cfg: HighResDEMCfg = dataclasses.field(default_factory=dict)
    crater_db_cfg: CraterDBCfg = dataclasses.field(default_factory=dict)
    crater_sampler_cfg: CraterSamplerCfg = dataclasses.field(default_factory=dict)
    crater_builder_cfg: CraterBuilderCfg = dataclasses.field(default_factory=dict)
    interpolator_cfg: InterpolatorCfg = dataclasses.field(default_factory=dict)
    crater_worker_manager_cfg: WorkerManagerCfg = dataclasses.field(
        default_factory=dict
    )
    interpolator_worker_manager_cfg: WorkerManagerCfg = dataclasses.field(
        default_factory=dict
    )

    def __post_init__(self):
        self.high_res_dem_cfg = HighResDEMCfg(**self.high_res_dem_cfg)
        self.crater_db_cfg = CraterDBCfg(**self.crater_db_cfg)
        self.crater_sampler_cfg = CraterSamplerCfg(**self.crater_sampler_cfg)
        self.crater_builder_cfg = CraterBuilderCfg(**self.crater_builder_cfg)
        self.interpolator_cfg = InterpolatorCfg(**self.interpolator_cfg)
        self.crater_worker_manager_cfg = WorkerManagerCfg(
            **self.crater_worker_manager_cfg
        )
        self.interpolator_worker_manager_cfg = WorkerManagerCfg(
            **self.interpolator_worker_manager_cfg
        )


class HighResDEMGen:
    def __init__(self, low_res_dem: np.ndarray, settings: HighResDEMGenCfg):
        self.low_res_dem = low_res_dem
        self.settings = settings

        self.current_block_coord = (0, 0)
        self.sim_is_warm = False

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
        self.interpolator = CPUInterpolator(self.settings.interpolator_cfg)
        self.crater_builder_manager = CraterBuilderManager(
            settings=self.settings.crater_worker_manager_cfg,
            builder=self.crater_builder,
        )
        self.interpolator_manager = BicubicInterpolatorManager(
            settings=self.settings.interpolator_worker_manager_cfg,
            interp=self.interpolator,
        )
        self.settings = self.settings.high_res_dem_cfg
        self.build_block_grid()
        self.instantiate_high_res_dem()
        self.get_low_res_dem_offset()

    def get_low_res_dem_offset(self):
        self.lr_dem_px_offset = (
            self.low_res_dem.shape[0] // 2,
            self.low_res_dem.shape[1] // 2,
        )
        self.lr_dem_ratio = self.settings.source_resolution / self.settings.resolution
        self.lr_dem_block_size = int(
            self.settings.block_size / self.settings.source_resolution
        )

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
            "has_crater_metadata": not self.settings.generate_craters,
            "has_crater_data": not self.settings.generate_craters,
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
                        "has_crater_metadata": not self.settings.generate_craters,
                        "has_crater_data": not self.settings.generate_craters,
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
        self.shift_block_grid(new_block_coord)
        # Shift the DEM
        self.shift_dem(pixel_shift)
        # Sets the current block coordinates to the new one.
        self.current_block_coord = new_block_coord
        # Trigger terrain update
        # Generate crater metadata for the new blocks
        if self.settings.generate_craters:
            self.generate_craters_metadata()
        # Asynchronous terrain block generation
        self.generate_terrain_blocks()

    def update_high_res_dem(self, coords):
        block_coordinates = self.cast_coordinates_to_block_space(coords)
        if not self.sim_is_warm:
            print("Warming up simulation")
            self.shift(block_coordinates)
            threading.Thread(target=self.threaded_high_res_dem_update).start()
            self.sim_is_warm = True
        if self.current_block_coord != block_coordinates:
            self.shift(coords)
            threading.Thread(target=self.threaded_high_res_dem_update).start()

    def is_map_done(self):
        return all(
            [
                self.block_grid_tracker[coords]["has_crater_metadata"]
                and self.block_grid_tracker[coords]["has_crater_data"]
                and self.block_grid_tracker[coords]["has_terrain_data"]
                for coords in self.map_grid_block2coords.values()
            ]
        )

    def threaded_high_res_dem_update(self):
        while not self.is_map_done():
            self.collect_terrain_data()
            time.sleep(0.1)

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
                self.block_grid_tracker[local_coords]["has_crater_metadata"] = True
            else:
                self.block_grid_tracker[local_coords]["has_crater_metadata"] = False
                print(f"Block {coords} does not have crater metadata")

    def querry_low_res_dem(self, coordinates: Tuple[float, float]):
        lr_dem_coordinates = (
            int(
                coordinates[0] / self.settings.source_resolution
                + self.lr_dem_px_offset[0]
            ),
            int(
                coordinates[1] / self.settings.source_resolution
                + self.lr_dem_px_offset[1]
            ),
        )
        return self.low_res_dem[
            lr_dem_coordinates[0]
            - self.settings.interpolation_padding : lr_dem_coordinates[0]
            + self.lr_dem_block_size
            + self.settings.interpolation_padding,
            lr_dem_coordinates[1]
            - self.settings.interpolation_padding : lr_dem_coordinates[1]
            + self.lr_dem_block_size
            + self.settings.interpolation_padding,
        ]

    def get_coordinates(self, coordinates):
        x = (
            (coordinates[0] + self.high_res_dem.shape[0] // 2)
            * self.settings.resolution
            - self.current_block_coord[0]
            - self.settings.block_size // 2
        )
        y = (
            (coordinates[1] + self.high_res_dem.shape[1] // 2)
            * self.settings.resolution
            - self.current_block_coord[1]
            - self.settings.block_size // 2
        )
        return (x, y)

    def generate_terrain_blocks(self):
        # Generate terrain data for + 1 block in each direction
        for grid_key in self.map_grid_block2coords.keys():
            x_i = grid_key[0]  # + self.current_block_coord[0]
            y_i = grid_key[1]  # + self.current_block_coord[1]
            coords = (x_i, y_i)
            grid_coords = self.map_grid_block2coords[grid_key]
            if not self.block_grid_tracker[grid_coords]["has_crater_data"]:
                self.crater_builder_manager.process_data(
                    coords, self.crater_db.get_block_data(coords)
                )
            if not self.block_grid_tracker[grid_coords]["has_terrain_data"]:
                self.interpolator_manager.process_data(
                    coords, self.querry_low_res_dem(coords)
                )

    def collect_terrain_data(self):
        offset = int(
            (self.settings.num_blocks + 1)
            * self.settings.block_size
            / self.settings.resolution
        )
        crater_results = self.crater_builder_manager.collect_results()
        for coords, data in crater_results:
            x_i, y_i = coords
            x_c, y_c = self.map_grid_block2coords[(x_i, y_i)]
            local_coords = (x_c, y_c)
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
            ] += data[0]
            self.block_grid_tracker[local_coords]["has_crater_data"] = True
        terrain_results = self.interpolator_manager.collect_results()
        for coords, data in terrain_results:
            x_i, y_i = coords
            x_c, y_c = self.map_grid_block2coords[(x_i, y_i)]
            local_coords = (x_c, y_c)
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
            ] += data
            self.block_grid_tracker[local_coords]["has_terrain_data"] = True

    def shutdown(self):
        self.crater_builder_manager.shutdown()
        self.interpolator_manager.shutdown()

    def __del__(self):
        self.shutdown()


if __name__ == "__main__":
    HRDEMCfg_D = {
        "num_blocks": 4,  # int = dataclasses.field(default_factory=int)
        "block_size": 50,  # float = dataclasses.field(default_factory=float)
        "pad_size": 10.0,  # float = dataclasses.field(default
        "max_blocks": int(1e7),  # int = dataclasses.field(default_factory=int)
        "seed": 42,  # int = dataclasses.field(default_factory=int)
        "resolution": 0.05,  # float = dataclasses.field(default_factory=float)
        "z_scale": 1.0,  # float = dataclasses.field(default_factory=float)
        "source_resolution": 5.0,  # float = dataclasses.field(default_factory=float)
        "resolution": 0.05,  # float = dataclasses.field(default_factory=float)
        "interpolation_padding": 2,  # int = dataclasses.field(default_factory=int)
        "generate_craters": True,
    }
    CWMCfg_D = {
        "num_workers": 8,  # int = dataclasses.field(default_factory=int)
        "input_queue_size": 400,  # int = dataclasses.field(default_factory=int)
        "output_queue_size": 16,  # int = dataclasses.field(default_factory=int)
        "worker_queue_size": 2,  # int = dataclasses.field(default_factory=int)
    }
    IWMCfg_D = {
        "num_workers": 1,  # int = dataclasses.field(default_factory=int)
        "input_queue_size": 400,  # int = dataclasses.field(default_factory=int)
        "output_queue_size": 30,  # int = dataclasses.field(default_factory=int)
        "worker_queue_size": 200,  # int = dataclasses.field(default_factory=int)
    }
    CraterDBCfg_D = {
        "block_size": 50,
        "max_blocks": 7,
        "save_to_disk": False,
        "write_to_disk_interval": 100,
    }
    CGCfg_D = {
        "profiles_path": "assets/Terrains/crater_spline_profiles.pkl",
        "min_xy_ratio": 0.85,
        "max_xy_ratio": 1.0,
        "random_rotation": True,
        "seed": 42,
        "num_unique_profiles": 10000,
    }
    CDDCfg_D = {
        "densities": [0.025, 0.05, 0.5],
        "radius": [[1.5, 2.5], [0.75, 1.5], [0.25, 0.5]],
        "num_repeat": 1,
        "seed": 42,
    }
    CraterSamplerCfg_D = {
        "block_size": 50,
        "crater_gen_cfg": CGCfg_D,
        "crater_dist_cfg": CDDCfg_D,
    }
    CraterBuilderCfg_D = {
        "block_size": 50,
        "pad_size": 10.0,
        "resolution": 0.05,
        "z_scale": 1.0,
    }
    ICfg = {
        "source_resolution": 5.0,
        "target_resolution": 0.05,
        "source_padding": 2,
        "method": "bicubic",
    }
    HRDEMGenCfg_D = {
        "high_res_dem_cfg": HRDEMCfg_D,
        "crater_db_cfg": CraterDBCfg_D,
        "crater_sampler_cfg": CraterSamplerCfg_D,
        "crater_builder_cfg": CraterBuilderCfg_D,
        "interpolator_cfg": ICfg,
        "crater_worker_manager_cfg": CWMCfg_D,
        "interpolator_worker_manager_cfg": IWMCfg_D,
    }

    settings = HighResDEMGenCfg(**HRDEMGenCfg_D)
    # low_res_dem = np.load("assets/Terrains/SouthPole/ldem_87s_5mpp/dem.npy")
    low_res_dem = np.load("assets/Terrains/SouthPole/NPD_final_adj_5mpp_surf/dem.npy")
    HRDEMGen = HighResDEMGen(low_res_dem, settings)

    from matplotlib import pyplot as plt
    import matplotlib.colors as mcolors
    import time

    # Initial Generation
    HRDEMGen.shift((0, 0))
    time.sleep(2.0)
    plt.figure()
    im_data = None
    while (not HRDEMGen.crater_builder_manager.is_output_queue_empty()) or (
        not HRDEMGen.crater_builder_manager.are_workers_done()
        or (not HRDEMGen.crater_builder_manager.is_input_queue_empty())
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
    time.sleep(5.0)
    HRDEMGen.collect_terrain_data()
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(5.0)

    # Rover has moved enough trigger new gen
    HRDEMGen.shift((50, 0))
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(5.0)
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
    time.sleep(5.0)
    HRDEMGen.collect_terrain_data()
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(5.0)

    # Rover has moved some more trigger new gen
    HRDEMGen.shift((50, 50))
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(5.0)
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
    time.sleep(5.0)
    HRDEMGen.collect_terrain_data()
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(5.0)

    # Rover has moved some more
    HRDEMGen.shift((100, 100))
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(5.0)
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
    time.sleep(5.0)
    HRDEMGen.collect_terrain_data()
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(5.0)

    # Rover has moved some more
    HRDEMGen.shift((0, 0))
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(5.0)
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
    time.sleep(5.0)
    HRDEMGen.collect_terrain_data()
    im_data.set_data(HRDEMGen.high_res_dem)
    im_data.set_norm(norm)
    plt.pause(0.25)
    plt.draw()
    time.sleep(5.0)

    plt.figure()
    s = time.time()
    HRDEMGen.update_high_res_dem((50, 50))
    while not HRDEMGen.is_map_done():
        time.sleep(0.1)
    e = time.time()
    print(f"Time to update entire map: {e-s}")
    plt.imshow(HRDEMGen.high_res_dem, cmap="terrain")
    plt.show()
    s = time.time()
    HRDEMGen.update_high_res_dem((50, 100))
    while not HRDEMGen.is_map_done():
        time.sleep(0.1)
    e = time.time()
    print(f"Time to update one band: {e-s}")
    plt.imshow(HRDEMGen.high_res_dem, cmap="terrain")
    plt.show()
    s = time.time()
    HRDEMGen.update_high_res_dem((100, 100))
    while not HRDEMGen.is_map_done():
        time.sleep(0.1)
    e = time.time()
    print(f"Time to update one band: {e-s}")
    plt.imshow(HRDEMGen.high_res_dem, cmap="terrain")
    plt.show()
    s = time.time()
    HRDEMGen.update_high_res_dem((150, 150))
    while not HRDEMGen.is_map_done():
        time.sleep(0.1)
    e = time.time()
    print(f"Time to update two bands: {e-s}")
    plt.imshow(HRDEMGen.high_res_dem, cmap="terrain")
    plt.show()

    plt.close()
    print("Done collecting terrain data...")
    HRDEMGen.crater_builder_manager.shutdown()
    HRDEMGen.interpolator_manager.shutdown()
