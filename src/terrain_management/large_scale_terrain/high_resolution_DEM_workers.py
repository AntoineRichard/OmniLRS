__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"


from typing import List, Tuple, Dict
import multiprocessing
import numpy as np
import dataclasses
import threading
import time
import copy
import cv2

from src.terrain_management.large_scale_terrain.crater_generation import (
    CraterBuilder,
    CraterDB,
    CraterSampler,
)
from src.terrain_management.large_scale_terrain.crater_generation import (
    CraterBuilderCfg,
    CraterDBCfg,
    CraterSamplerCfg,
)
from src.terrain_management.large_scale_terrain.crater_generation import (
    CraterMetadata,
    BoundingBox,
)


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
        self,
        queue_size: int,
        output_queue: multiprocessing.JoinableQueue,
        parent_thread: threading.Thread,
        thread_timeout: float = 1.0,
        **kwargs,
    ):
        """
        Args:
            queue_size (int): The size of the input queue.
            output_queue (multiprocessing.JoinableQueue): The output queue.
            parent_thread (threading.Thread): The parent thread.
            thread_timeout (float): The timeout for the worker thread. (seconds)
            **kwargs: Additional arguments.
        """

        super().__init__()
        self.stop_event = multiprocessing.Event()
        self.input_queue = multiprocessing.JoinableQueue(maxsize=queue_size)
        self.output_queue = output_queue
        self.daemon = True
        self.parent_thread = parent_thread
        self.thread_timeout = thread_timeout

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
        parent_thread: threading.Thread = None,
        thread_timeout: float = 1.0,
        **kwargs,
    ):
        """
        Args:
            settings (WorkerManagerCfg): The settings for the worker manager.
            worker_class (BaseWorker): The worker class to use.
            parent_thread (threading.Thread): The parent thread.
            thread_timeout (float): The timeout for the worker thread. (seconds)
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
        self.parent_thread = parent_thread
        self.thread_timeout = thread_timeout
        self.kwargs = kwargs
        self.stop_event = multiprocessing.Event()

        self.instantiate_workers(
            num_workers=self.num_workers,
            worker_queue_size=self.worker_queue_size,
            worker_class=self.worker_class,
            **self.kwargs,
        )

        # Start the manager thread
        self.manager_thread = threading.Thread(target=self.dispatch_jobs, daemon=True)

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
            worker_class(
                worker_queue_size,
                self.output_queue,
                self.parent_thread,
                self.thread_timeout,
                **kwargs,
            )
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

    def dispatch_jobs(self, parent_thread: threading.Thread) -> None:
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

    def shutdown_workers(self) -> None:
        """
        Shuts down the workers.
        """

        print("Shutting down workers.")

        for worker in self.workers:
            worker.stop_event.set()
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
        parent_thread: threading.Thread = None,
        thread_timeout: float = 1.0,
        builder: CraterBuilder = None,
    ) -> None:
        """
        Args:
            queue_size (int): The size of the input queue.
            output_queue (multiprocessing.JoinableQueue): The output queue.
            builder (CraterBuilder): The crater builder.
        """

        super().__init__(queue_size, output_queue, parent_thread, thread_timeout)
        self.builder = copy.copy(builder)

    def run(self) -> None:
        """
        The main function of the worker process.
        This function is called when the worker process is started.
        It takes craters metadata and coordinates from the input queue
        and generates images with inprinted craters.
        """

        while not self.stop_event.is_set():
            try:
                coords, crater_meta_data = self.input_queue.get(
                    timeout=self.thread_timeout
                )
                if crater_meta_data is None:
                    self.input_queue.task_done()
                    break
                self.output_queue.put(
                    (coords, self.builder.generateCraters(crater_meta_data, coords))
                )
                self.input_queue.task_done()
            except:
                pass
        print("crater worker dead.")


class CraterBuilderManager(BaseWorkerManager):
    """
    CraterBuilderManager class. This class is responsible for managing the crater
    builder workers. It is responsible for distributing the work across the workers
    and collecting the results.
    """

    def __init__(
        self,
        settings: WorkerManagerCfg = WorkerManagerCfg(),
        parent_thread: threading.Thread = None,
        thread_timeout: float = 1.0,
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
            parent_thread=parent_thread,
            thread_timeout=thread_timeout,
        )

    def dispatch_jobs(self) -> None:
        """
        Dispatches the jobs to the workers.
        The jobs are dispatched in a balanced way such that the workers have a similar load.
        """

        # Assigns the jobs such that the workers have a balanced load
        while (not self.stop_event.is_set()) and (self.parent_thread.is_alive()):
            try:
                coords, crater_metadata = self.input_queue.get(
                    timeout=self.thread_timeout
                )
                if crater_metadata is None:  # Check for shutdown signal
                    self.input_queue.task_done()
                    break
                self.workers[self.get_shortest_queue_index()].input_queue.put(
                    (coords, crater_metadata)
                )
                self.input_queue.task_done()
            except:
                pass

        self.shutdown_workers()
        print("crater manager dead.")


class BicubicInterpolatorWorker(BaseWorker):
    """
    BicubicInterpolatorWorker class. This class is responsible for interpolating the
    terrain data.
    """

    def __init__(
        self,
        queue_size: int = 10,
        output_queue: multiprocessing.JoinableQueue = multiprocessing.JoinableQueue(),
        parent_thread: threading.Thread = None,
        thread_timeout: float = 1.0,
        interp: Interpolator = None,
    ):
        """
        Args:
            queue_size (int): The size of the input queue.
            output_queue (multiprocessing.JoinableQueue): The output queue.
            interp (Interpolator): The interpolator.
        """

        super().__init__(queue_size, output_queue, parent_thread, thread_timeout)
        self.interpolator = copy.copy(interp)

    def run(self) -> None:
        """
        The main function of the worker process.
        This function is called when the worker process is started.
        It takes terrain data from the input queue and interpolates it.
        """

        while not self.stop_event.is_set():
            try:
                coords, data = self.input_queue.get(timeout=self.thread_timeout)
                if data is None:
                    break
                self.output_queue.put((coords, self.interpolator.interpolate(data)))
                self.input_queue.task_done()
            except:
                pass
        print("bicubic worker dead.")


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
        parent_thread: threading.Thread = None,
        thread_timeout: float = 1.0,
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
            parent_thread=parent_thread,
            thread_timeout=thread_timeout,
        )
        cv2.setNumThreads(num_cv2_threads)

    def dispatch_jobs(self) -> None:
        """
        Dispatches the jobs to the workers.
        The jobs are dispatched in a balanced way such that the workers have a similar load.
        """

        while (not self.stop_event.is_set()) and (self.parent_thread.is_alive()):
            try:
                coords, data = self.input_queue.get(timeout=self.thread_timeout)
                if data is None:
                    break
                self.workers[self.get_shortest_queue_index()].input_queue.put(
                    (coords, data)
                )
                self.input_queue.task_done()
            except:
                pass
        self.shutdown_workers()
        print("bicubic manager dead.")


def monitor_main_thread():
    """
    Monitors the main thread.
    This function is used to monitor the main thread and check if it is still alive.
    """

    while True:
        time.sleep(1)
        if not threading.main_thread().is_alive():
            print("Main thread is dead, shutting down workers.")
            break
