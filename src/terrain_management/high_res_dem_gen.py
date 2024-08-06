from typing import List, Tuple, Dict
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
    """
    Configuration for the interpolator.

    Args:
        source_resolution (float): The resolution of the source data (meters per pixel).
        target_resolution (float): The resolution of the target data (meters per pixel).
        source_padding (int): The padding of the source data (pixels).
        method (str): The interpolation method. Can be one of "nearest", "linear", "bicubic", "area".
    """

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
    """
    Interpolator base class.

    The Interpolator class is responsible for interpolating the terrain data.
    """

    def __init__(self, settings: InterpolatorCfg):
        """
        Args:
            settings (InterpolatorCfg): The settings for the interpolator.
        """

        self.settings = settings

    def interpolate(self, data: np.ndarray) -> np.ndarray:
        """
        Interpolates the given data using the settings of the interpolator.

        Args:
            data (np.ndarray): The terrain data to interpolate.

        Returns:
            np.ndarray: The interpolated terrain data.

        TO BE IMPLEMENTED BY SUBCLASSES.
        """

        raise NotImplementedError


class CPUInterpolator(Interpolator):
    """
    An interpolator that uses the CPU to interpolate the terrain data.
    """

    def __init__(self, settings: InterpolatorCfg):
        """
        Args:
            settings (InterpolatorCfg): The settings for the interpolator.
        """

        super().__init__(settings)

    def interpolate(self, data: np.ndarray) -> np.ndarray:
        """
        Interpolates the given data using the settings of the interpolator.
        The interpolation is done using cv2 resize.

        Args:
            data (np.ndarray): The terrain data to interpolate.

        Returns:
            np.ndarray: The interpolated terrain data.
        """

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
    """
    BaseWorker class. This class is used to create worker processes that can be used
    to distribute the work across multiple processes.
    """

    def __init__(
        self, queue_size: int, output_queue: multiprocessing.JoinableQueue, **kwargs
    ):
        """
        Args:
            queue_size (int): The size of the input queue.
            output_queue (multiprocessing.JoinableQueue): The output queue.
            **kwargs: Additional arguments.
        """

        super().__init__()
        self.stop_event = multiprocessing.Event()
        self.input_queue = multiprocessing.JoinableQueue(maxsize=queue_size)
        self.output_queue = output_queue
        self.daemon = True

    def get_input_queue_length(self) -> int:
        """
        Returns the length of the input queue.

        Returns:
            int: The length of the input queue.
        """

        return self.input_queue.qsize()

    def is_input_queue_empty(self) -> bool:
        """
        Checks if the input queue is empty.

        Returns:
            bool: True if the input queue is empty, False otherwise.
        """

        return self.input_queue.empty()

    def is_input_queue_full(self):
        """
        Checks if the input queue is full.

        Returns:
            bool: True if the input queue is full, False otherwise.
        """

        return self.input_queue.full()

    def run(self) -> None:
        """
        The main function of the worker process.
        This function is called when the worker process is started.

        TO BE IMPLEMENTED BY SUBCLASSES.
        """

        raise NotImplementedError


@dataclasses.dataclass
class WorkerManagerCfg:
    """
    Configuration for the worker manager.

    Args:
        num_workers (int): The number of workers.
        input_queue_size (int): The size of the input queue.
        output_queue_size (int): The size of the output queue.
        worker_queue_size (int): The size of the worker queue.
    """

    num_workers: int = dataclasses.field(default_factory=int)
    input_queue_size: int = dataclasses.field(default_factory=int)
    output_queue_size: int = dataclasses.field(default_factory=int)
    worker_queue_size: int = dataclasses.field(default_factory=int)


class BaseWorkerManager:
    """
    BaseWorkerManager class. This class is used to manage the worker processes.
    It is responsible for distributing the work across the workers and collecting the results.
    It ensures that the workers are running and that the work is distributed in a balanced way.
    I.e. the workers have a similar load.
    """

    def __init__(
        self,
        settings: WorkerManagerCfg = WorkerManagerCfg(),
        worker_class: BaseWorker = None,
        **kwargs,
    ):
        """
        Args:
            settings (WorkerManagerCfg): The settings for the worker manager.
            worker_class (BaseWorker
            **kwargs: Additional arguments.
        """

        # Create the input and output queues
        self.input_queue = multiprocessing.JoinableQueue(
            maxsize=settings.input_queue_size
        )
        self.output_queue = multiprocessing.JoinableQueue(
            maxsize=settings.output_queue_size
        )

        # Create the workers
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

        # Start the manager thread
        self.manager_thread = multiprocessing.Process(
            target=self.dispatch_jobs,
        )
        self.manager_thread.daemon = True
        self.manager_thread.start()

    def __enter__(self):
        """
        Context manager enter method.
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """
        Context manager exit method.
        """

        print("Exception caught, shutting down workers and main thread.")
        self.stop_event.set()
        self.shutdown()

    def instantiate_workers(
        self,
        num_workers: int = 1,
        worker_queue_size: int = 10,
        worker_class: BaseWorker = None,
        **kwargs,
    ) -> None:
        """
        Instantiates the worker processes.

        Args:
            num_workers (int): The number of workers to instantiate.
            worker_queue_size (int): The size of the worker queue.
            worker_class (BaseWorker): The worker class to instantiate.
            **kwargs: Additional arguments.
        """

        self.workers = [
            worker_class(worker_queue_size, self.output_queue, **kwargs)
            for _ in range(num_workers)
        ]
        for worker in self.workers:
            worker.start()

    def get_load_per_worker(self) -> Dict[str, int]:
        """
        Returns the load of each worker.
        The dictionary contains the load of each worker in the form of:
            {
                "worker_0": 10,
                "worker_1": 20,
                ...
            }

        Where the key is the worker name and the value is the number of jobs
        in the worker's input queue.

        Returns:
            dict: A dictionary with the load of each worker.
        """
        return {
            "worker_{}".format(i): worker.get_input_queue_length()
            for i, worker in enumerate(self.workers)
        }

    def get_input_queue_length(self) -> int:
        """
        Returns the length of the input queue.

        Returns:
            int: The length of the input queue.
        """

        return self.input_queue.qsize()

    def get_output_queue_length(self) -> int:
        """
        Returns the length of the output queue.

        Returns:
            int: The length of the output queue.
        """

        return self.output_queue.qsize()

    def is_input_queue_empty(self) -> bool:
        """
        Checks if the input queue is empty.

        Returns:
            bool: True if the input queue is empty, False otherwise.
        """

        return self.input_queue.empty()

    def is_output_queue_empty(self) -> bool:
        """
        Checks if the output queue is empty.

        Returns:
            bool: True if the output queue is empty, False otherwise.
        """

        return self.output_queue.empty()

    def is_input_queue_full(self) -> bool:
        """
        Checks if the input queue is full.

        Returns:
            bool: True if the input queue is full, False otherwise.
        """

        return self.input_queue.full()

    def is_output_queue_full(self) -> bool:
        """
        Checks if the output queue is full.

        Returns:
            bool: True if the output queue is full, False otherwise.
        """

        return self.output_queue.full()

    def get_shortest_queue_index(self) -> int:
        """
        Returns the index of the worker with the shortest input queue.
        This is used to distribute the work in a balanced way.

        Returns:
            int: The index of the worker with the shortest input queue.
        """
        return min(
            range(len(self.workers)),
            key=lambda i: self.workers[i].get_input_queue_length(),
        )

    def are_workers_done(self) -> bool:
        """
        Checks if all the workers are done.

        Returns:
            bool: True if all the workers are done, False otherwise.
        """

        return all([worker.is_input_queue_empty() for worker in self.workers])

    def dispatch_jobs(self) -> None:
        """
        Dispatches the jobs to the workers.

        TO BE IMPLEMENTED BY SUBCLASSES.
        """
        raise NotImplementedError

    def process_data(self, coords: Tuple[float, float], data) -> None:
        """
        Processes the data by adding it to the worker manager's input queue.

        Args:
            coords (Tuple[float, float]): The coordinates of the data.
            data: The data to process.
        """

        self.input_queue.put((coords, data))

    def collect_results(self) -> List[Tuple[Tuple[float, float], np.ndarray]]:
        """
        Collects the results from the workers.

        Returns:
            List[Tuple[Tuple[float, float], np.ndarray]]: A list of tuples with the
            coordinates and the data.
        """

        results = []
        num = self.get_output_queue_length()
        for _ in range(num):
            results.append(self.output_queue.get())
            self.output_queue.task_done()
        return results

    def shutdown(self) -> None:
        """
        Shuts down the worker manager and its workers.
        """

        self.input_queue.put(((0, 0), None))
        self.manager_thread.join()
        for worker in self.workers:
            worker.input_queue.put(((0, 0), None))
        for worker in self.workers:
            worker.join()

    def __del__(self) -> None:
        """
        Destructor.
        """

        self.shutdown()


class CraterBuilderWorker(BaseWorker):
    """
    CraterBuilderWorker class. This class is responsible for generating the craters.
    """

    def __init__(
        self,
        queue_size: int = 10,
        output_queue: multiprocessing.Queue = multiprocessing.JoinableQueue(),
        builder: CraterBuilder = None,
    ) -> None:
        """
        Args:
            queue_size (int): The size of the input queue.
            output_queue (multiprocessing.JoinableQueue): The output queue.
            builder (CraterBuilder): The crater builder.
        """

        super().__init__(queue_size, output_queue)
        self.builder = copy.copy(builder)

    def run(self) -> None:
        """
        The main function of the worker process.
        This function is called when the worker process is started.
        It takes craters metadata and coordinates from the input queue
        and generates images with inprinted craters.
        """

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
    """
    CraterBuilderManager class. This class is responsible for managing the crater
    builder workers. It is responsible for distributing the work across the workers
    and collecting the results.
    """

    def __init__(
        self,
        settings: WorkerManagerCfg = WorkerManagerCfg(),
        builder: CraterBuilder = None,
    ) -> None:
        """
        Args:
            settings (WorkerManagerCfg): The settings for the worker manager.
            builder (CraterBuilder): The crater builder.
        """

        super().__init__(
            settings=settings,
            worker_class=CraterBuilderWorker,
            builder=builder,
        )

    def dispatch_jobs(self) -> None:
        """
        Dispatches the jobs to the workers.
        The jobs are dispatched in a balanced way such that the workers have a similar load.
        """

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
    """
    BicubicInterpolatorWorker class. This class is responsible for interpolating the
    terrain data.
    """

    def __init__(
        self,
        queue_size: int = 10,
        output_queue: multiprocessing.JoinableQueue = multiprocessing.JoinableQueue(),
        interp: Interpolator = None,
    ):
        """
        Args:
            queue_size (int): The size of the input queue.
            output_queue (multiprocessing.JoinableQueue): The output queue.
            interp (Interpolator): The interpolator.
        """

        super().__init__(queue_size, output_queue)
        self.interpolator = copy.copy(interp)

    def run(self) -> None:
        """
        The main function of the worker process.
        This function is called when the worker process is started.
        It takes terrain data from the input queue and interpolates it.
        """

        while not self.stop_event.is_set():
            coords, data = self.input_queue.get()
            if data is None:
                break
            self.output_queue.put((coords, self.interpolator.interpolate(data)))
            self.input_queue.task_done()


class BicubicInterpolatorManager(BaseWorkerManager):
    """
    BicubicInterpolatorManager class. This class is responsible for managing the
    bicubic interpolator workers. It is responsible for distributing the work across
    the workers and collecting the results.
    """

    def __init__(
        self,
        settings: WorkerManagerCfg = WorkerManagerCfg(),
        interp: Interpolator = None,
        num_cv2_threads: int = 4,
    ):
        """
        Args:
            settings (WorkerManagerCfg): The settings for the worker manager.
            interp (Interpolator): The interpolator.
            num_cv2_threads (int): The number of threads to use for cv2.
        """

        super().__init__(
            settings=settings,
            worker_class=BicubicInterpolatorWorker,
            interp=interp,
        )
        cv2.setNumThreads(num_cv2_threads)

    def dispatch_jobs(self):
        """
        Dispatches the jobs to the workers.
        The jobs are dispatched in a balanced way such that the workers have a similar load.
        """

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
    """
    Configuration for the high resolution DEM.

    Args:
        num_blocks (int): The number of blocks in each direction. Total number of blocks is 2 * num_blocks + 1.
        block_size (float): The size of each block in meters. The blocks are square.
        pad_size (float): The size of the padding in meters. Used to avoid edge cases when adding craters.
        max_blocks (int): The maximum number of blocks. Used to avoid memory issues.
        seed (int): The seed for the random number generator.
        resolution (float): The resolution of the high resolution DEM in meters per pixel.
        z_scale (float): The scale of the z axis.
        source_resolution (float): The resolution of the source data in meters per pixel.
        resolution (float): The resolution of the high resolution DEM in meters per pixel.
        interpolation_padding (int): The padding for the interpolation. In pixels.
        generate_craters (bool): True if craters should be generated, False otherwise.
    """

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
    """
    Configuration for the high resolution DEM generation.
    Note that the class should be fed dictionaries and not objects.

    Args:
        high_res_dem_cfg (HighResDEMCfg): The configuration for the high resolution DEM.
        crater_db_cfg (CraterDBCfg): The configuration for the crater DB.
        crater_sampler_cfg (CraterSamplerCfg): The configuration for the crater sampler.
        crater_builder_cfg (CraterBuilderCfg): The configuration for the crater builder.
        interpolator_cfg (InterpolatorCfg): The configuration for the interpolator.
        crater_worker_manager_cfg (WorkerManagerCfg): The configuration for the crater worker manager.
        interpolator_worker_manager_cfg (WorkerManagerCfg): The configuration for the interpolator worker manager.
    """

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
    """
    The HighResDEMGen class is responsible for generating the high resolution DEM.
    """

    def __init__(self, low_res_dem: np.ndarray, settings: HighResDEMGenCfg) -> None:
        """
        Args:
            low_res_dem (np.ndarray): The low resolution DEM.
            settings (HighResDEMGenCfg): The settings for the high resolution DEM generation.
        """
        self.low_res_dem = low_res_dem
        self.settings = settings

        self.current_block_coord = (0, 0)
        self.sim_is_warm = False

        self.sim_lock = False
        self.terrain_is_primed = False
        self.crater_db_is_primed = False
        self.build()

    def build(self) -> None:
        """
        Builds the objects responsible to generate the high resolution DEM.
        """

        # Creates the crater DB and crater sampler
        # The DB is used to store the crater metadata and the sampler is used
        # to generate the craters metadata.
        self.crater_db = CraterDB(self.settings.crater_db_cfg)
        self.crater_sampler = CraterSampler(
            self.settings.crater_sampler_cfg, db=self.crater_db
        )
        # Creates the crater builder and the interpolator they are used by the
        # worker managers to distribute the generation of craters and the
        # interpolation of the terrain data accross multiple workers.
        self.crater_builder = CraterBuilder(
            self.settings.crater_builder_cfg, db=self.crater_db
        )
        self.interpolator = CPUInterpolator(self.settings.interpolator_cfg)
        # Creates the worker managers that will distribute the work to the workers.
        # This enables the generation of craters and the interpolation of the terrain
        # data to be done in parallel.
        self.crater_builder_manager = CraterBuilderManager(
            settings=self.settings.crater_worker_manager_cfg,
            builder=self.crater_builder,
        )
        self.interpolator_manager = BicubicInterpolatorManager(
            settings=self.settings.interpolator_worker_manager_cfg,
            interp=self.interpolator,
        )
        # Instantiates the high resolution DEM with the given settings.
        self.settings = self.settings.high_res_dem_cfg
        self.build_block_grid()
        self.instantiate_high_res_dem()
        self.get_low_res_dem_offset()

    def get_low_res_dem_offset(self) -> None:
        """
        Computes the offset of the low resolution DEM in the high resolution DEM.
        """
        self.lr_dem_px_offset = (
            self.low_res_dem.shape[0] // 2,
            self.low_res_dem.shape[1] // 2,
        )
        self.lr_dem_ratio = self.settings.source_resolution / self.settings.resolution
        self.lr_dem_block_size = int(
            self.settings.block_size / self.settings.source_resolution
        )

    def instantiate_high_res_dem(self) -> None:
        """
        Instantiates the high resolution DEM with the given settings.
        The high resolution DEM is a numpy array that is used to store the terrain data.
        The high resolution DEM is generated with a padding of 1 block in each direction.
        This is done to avoid edge cases when computing the terrain data and to have a
        buffer of blocks to reuse when the high resolution DEM is shifted.

        Please note that this function does not generate the actual DEM, but only instantiates
        an empty numpy array with the correct dimensions.
        """

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

    def cast_coordinates_to_block_space(
        self, coordinates: Tuple[float, float]
    ) -> Tuple[int, int]:
        """
        Casts the given coordinates to the block space. The block space is the space
        where the blocks are defined. The block space is defined by the block size and
        the resolution of the high resolution DEM.

        The coordinates are still expressed in meters, but they can only be an increment of
        the block size (in meters).
        """

        x, y = coordinates
        x_block = int(x // self.settings.block_size) * self.settings.block_size
        y_block = int(y // self.settings.block_size) * self.settings.block_size
        return (x_block, y_block)

    def build_block_grid(self) -> None:
        """
        The block grid is a dictionary that keeps track of the state of each block in the
        grid. The state of each block is represented by a dictionary with the following
        keys:
            - has_crater_metadata: True if the block has the crater metadata, False otherwise.
            - has_crater_data: True if the block has the crater data, False otherwise.
            - has_terrain_data: True if the block has the terrain data, False otherwise.
            - is_padding: True if the block is a padding block, False otherwise.

        The map_grid_block2coords is a dictionary that maps the block coordinates to the
        grid coordinates. This is useful to quickly find the block in the grid given the
        grid coordinates.

        The block grid is generated with a padding of 1 block in each direction. This is
        done to avoid edge cases when computing the terrain data and to have a buffer of
        blocks to reuse when the high resolution DEM is shifted.
        """

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

    def shift_block_grid(self, coordinates: Tuple[float, float]) -> None:
        """
        Shifts the block grid to the given coordinates while preserving the state of the
        blocks. The function will also update the map_grid_block2coords dictionary to
        reflect the new block coordinates.
        """

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

    def shift_dem(self, pixel_shift: Tuple[int, int]) -> None:
        """
        When an update is triggered, the high resolution DEM is shifted by N amount of
        blocks in the pixel coordinate space. This is done to keep the high resolution
        DEM centered around the current block coordinate. This function also allows to
        reuse previously computed data when the high resolution DEM is shifted. Hence,
        when the simulation is "warm" the map only needs to update the new blocks,
        reducing the time required to compute them.

        Args:
            pixel_shift (Tuple[int, int]): Number of pixels to shift the high resolution
                DEM in the pixel coordinate space.
        """

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

    def shift(self, coordinates: Tuple[float, float]) -> None:
        """
        Shifts the high resolution DEM and the block grid to the given coordinates.
        The function will also trigger the generation of crater metadata for the
        new blocks. Once this is done, the function will send to the workers the new
        blocks to generate the terrain data.

        Args:
            coordinates (Tuple[float, float]): Coordinates in meters in the low
                resolution DEM frame
        """

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

    def get_height(self, coordinates: Tuple[float, float]) -> float:
        """
        Returns the height of the high resolution DEM at the given coordinates.

        Args:
            coordinates (Tuple[float, float]): Coordinates in meters in the
                low resolution DEM frame.

        Returns:
            float: Height of the high resolution DEM at the given coordinates.
        """

        local_coordinates = self.get_coordinates(coordinates)
        x = local_coordinates[0] / self.settings.resolution
        y = local_coordinates[1] / self.settings.resolution
        return self.high_res_dem[int(x), int(y)]

    def update_high_res_dem(self, coords: Tuple[float, float]) -> bool:
        """
        Updates the high resolution DEM data.
        The function will return True if the DEM has been updated, False otherwise.
        The DEM only gets updated if the coordinate casted in the "block space" is
        different from the current block coordinate. A coordinate in the "block space"
        is the coordinate of the block that the coordinates are in.

        Args:
            coords (Tuple[float, float]): Coordinates in meters in the low resolution
                DEM frame.

        Returns:
            bool: True if the DEM has been updated, False otherwise.
        """

        block_coordinates = self.cast_coordinates_to_block_space(coords)
        updated = False
        if not self.sim_is_warm:
            print("Warming up simulation")
            self.shift(block_coordinates)
            threading.Thread(target=self.threaded_high_res_dem_update).start()
            self.sim_is_warm = True
            updated = True
        if self.current_block_coord != block_coordinates:
            print("Triggering high res DEM update")
            self.shift(coords)
            threading.Thread(target=self.threaded_high_res_dem_update).start()
            updated = True
        return updated

    def is_map_done(self) -> bool:
        """
        Checks if all the blocks in the grid have the necessary data to render the
        terrain. If the blocks are missing data, the function will return False.
        If all the blocks have the necessary data, the function will return True.

        Returns:
            bool: True if all the blocks have the necessary data, False otherwise.
        """

        return all(
            [
                self.block_grid_tracker[coords]["has_crater_metadata"]
                and self.block_grid_tracker[coords]["has_crater_data"]
                and self.block_grid_tracker[coords]["has_terrain_data"]
                for coords in self.map_grid_block2coords.values()
            ]
        )

    def threaded_high_res_dem_update(self) -> None:
        """
        Threaded function to update the high resolution DEM. It will wait untill the
        terrain data and crater data is available for the blocks in the grid.
        """

        while not self.is_map_done():
            self.collect_terrain_data()
            time.sleep(0.1)

    def generate_craters_metadata(self) -> None:
        """
        Generates crater metadata for the blocks in the grid. The data is generated for
        the blocks in the grid plus 2 blocks in each direction. This is done to ensure
        that the crater data is available for the blocks that are being rendered.

        The generated crater metadata is stored in the crater database. And the blocks
        state is udpated to reflect that the crater metadata is available.
        """

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

    def querry_low_res_dem(self, coordinates: Tuple[float, float]) -> np.ndarray:
        """
        Takes coordinates in meters, with (0,0) being the centers of the low resolution
        DEM. Returns a patch from the low resolution DEM data for the block matching the
        coordinates. Note that these coordinates are using increments of block_size meters.

        Args:
            coordinates (Tuple[float, float]): Coordinates in meters in the low resolution
                DEM frame.

        Returns:
            np.ndarray: Patch of the low resolution DEM data for the block matching the
                coordinates.
        """

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

    def get_coordinates(self, coordinates: Tuple[float, float]) -> Tuple[float, float]:
        """
        Takes coordinates in meters, with (0,0) being the centers of the low resolution
        DEM. Returns the coordinates in meters, relatively to the center of the high
        resolution DEM.

        Args:
            coordinates (Tuple[float, float]): Coordinates in meters in the low resolution
                DEM frame.

        Returns:
            Tuple[float, float]: Coordinates in meters in the high resolution DEM frame
        """
        x = (
            coordinates[0]
            + (self.high_res_dem.shape[0] // 2) * self.settings.resolution
            - self.current_block_coord[0]
            - self.settings.block_size // 2
        )
        y = (
            coordinates[1]
            + (self.high_res_dem.shape[1] // 2) * self.settings.resolution
            - self.current_block_coord[1]
            - self.settings.block_size // 2
        )
        return (x, y)

    def generate_terrain_blocks(self) -> None:
        """
        Generates the terrain data for the blocks in the grid. The function will trigger
        the workers to generate the terrain data for the blocks in the grid. The terrain
        data is generated for the blocks in the grid plus 1 block in each direction. This
        is done to ensure that the terrain data is available for the blocks that are being
        rendered.
        """

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

    def collect_terrain_data(self) -> None:
        """
        Collects the terrain data from the workers and updates the high resolution DEM
        with the new terrain data. This is required to collect the output of the workers
        and update the high resolution DEM with it.
        """

        # Offset to account for the padding blocks
        offset = int(
            (self.settings.num_blocks + 1)
            * self.settings.block_size
            / self.settings.resolution
        )

        # Collect the results from the workers responsible for adding craters
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

        # Collect the results from the workers responsible for interpolating the terrain
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

    def shutdown(self) -> None:
        """
        Shuts down the high resolution DEM generation. This will shutdown the workers
        and the main worker manager.
        """

        self.crater_builder_manager.shutdown()
        self.interpolator_manager.shutdown()

    def __del__(self) -> None:
        """
        Destructor for the HighResDEMGen class. It will call the shutdown function to
        ensure that all the workers are properly shutdown.
        """

        self.shutdown()


if __name__ == "__main__":
    HRDEMCfg_D = {
        "num_blocks": 4,
        "block_size": 50,
        "pad_size": 10.0,
        "max_blocks": int(1e7),
        "seed": 42,
        "resolution": 0.05,
        "z_scale": 1.0,
        "source_resolution": 5.0,
        "resolution": 0.05,
        "interpolation_padding": 2,
        "generate_craters": True,
    }
    CWMCfg_D = {
        "num_workers": 8,
        "input_queue_size": 400,
        "output_queue_size": 16,
        "worker_queue_size": 2,
    }
    IWMCfg_D = {
        "num_workers": 1,
        "input_queue_size": 400,
        "output_queue_size": 30,
        "worker_queue_size": 200,
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
    plt.title("High Resolution DEM. Close that window to continue.")
    plt.imshow(HRDEMGen.high_res_dem, cmap="terrain")
    plt.show()
    s = time.time()
    HRDEMGen.update_high_res_dem((50, 100))
    while not HRDEMGen.is_map_done():
        time.sleep(0.1)
    e = time.time()
    print(f"Time to update one band: {e-s}")
    plt.title("High Resolution DEM. Close that window to continue.")
    plt.imshow(HRDEMGen.high_res_dem, cmap="terrain")
    plt.show()
    s = time.time()
    HRDEMGen.update_high_res_dem((100, 100))
    while not HRDEMGen.is_map_done():
        time.sleep(0.1)
    e = time.time()
    print(f"Time to update one band: {e-s}")
    plt.title("High Resolution DEM. Close that window to continue.")
    plt.imshow(HRDEMGen.high_res_dem, cmap="terrain")
    plt.show()
    s = time.time()
    HRDEMGen.update_high_res_dem((150, 150))
    while not HRDEMGen.is_map_done():
        time.sleep(0.1)
    e = time.time()
    print(f"Time to update two bands: {e-s}")
    plt.title("High Resolution DEM. Close that window to continue.")
    plt.imshow(HRDEMGen.high_res_dem, cmap="terrain")
    plt.show()

    plt.close()
    print("Done collecting terrain data...")
    HRDEMGen.crater_builder_manager.shutdown()
    HRDEMGen.interpolator_manager.shutdown()
