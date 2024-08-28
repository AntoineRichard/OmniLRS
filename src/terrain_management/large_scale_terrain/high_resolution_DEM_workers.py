__author__ = "Antoine Richard"
__copyright__ = "Copyright 2024, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"


import multiprocessing.managers
from typing import List, Tuple, Dict
import multiprocessing
import numpy as np
import dataclasses
import threading
import time
import copy
import PIL
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
                print("Warning: Bicubic interpolation with downscaling. Consider using a different method.")
        elif self.method == "nearest":
            self.method = cv2.INTER_NEAREST
        elif self.method == "linear":
            self.method = cv2.INTER_LINEAR
        elif self.method == "area":
            self.method = cv2.INTER_AREA
            if self.fx > 1.0:
                print("Warning: Area interpolation with upscaling. Consider using a different method.")
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
    This interpolator uses the cv2 library to interpolate the data.
    In our testing, cv2 was generating nasty banding artifacts when
    upscaling the data. So unless the upsampling factor is small, and
    the data isn't smooth to start with, I'd recommend using the PIL
    interpolator instead.

    The cv2 interpolator is faster than the PIL interpolator by about 4 times.
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


class CPUInterpolator_PIL(Interpolator):
    """
    An interpolator that uses the CPU to interpolate the terrain data.
    This interpolator uses the PIL library to interpolate the data.
    I found PIL to be more accurate than cv2 for bicubic interpolation.

    That's because cv2 uses an alpha value of -0.75 for its bicubic interpolation,
    when it should be -0.5.

    PIL uses a value of -0.5 for bicubic interpolation, which provides more accurate results,
    but is about 4 times slower than cv2.
    """

    def __init__(self, settings: InterpolatorCfg):
        """
        Args:
            settings (InterpolatorCfg): The settings for the interpolator.
        """

        super().__init__(settings)

        if self.settings.method == cv2.INTER_CUBIC:
            self.settings.method = PIL.Image.BICUBIC
        else:
            raise ValueError(
                "PIL interpolation only supports bicubic interpolation. Use the CV2 interpolator instead. It's faster."
            )

    def interpolate(self, data: np.ndarray) -> np.ndarray:
        image = PIL.Image.fromarray(data)
        image = image.resize(
            (int(data.shape[1] * self.settings.fx), int(data.shape[0] * self.settings.fy)),
            resample=self.settings.method,
        )
        return np.array(image)[
            int(self.settings.source_padding * self.settings.fx) : -int(
                self.settings.source_padding * self.settings.fy
            ),
            int(self.settings.source_padding * self.settings.fx) : -int(
                self.settings.source_padding * self.settings.fy
            ),
        ]


@dataclasses.dataclass
class WorkerManagerCfg:
    """
    Configuration for the worker manager.

    Args:
        num_workers (int): The number of workers.
        input_queue_size (int): The size of the input queue.
        output_queue_size (int): The size of the output queue.
    """

    num_workers: int = dataclasses.field(default_factory=int)
    input_queue_size: int = dataclasses.field(default_factory=int)
    output_queue_size: int = dataclasses.field(default_factory=int)


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
        worker_class=None,
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

        self.manager = multiprocessing.Manager()
        # Create the input and output queues
        self.input_queue = self.manager.Queue(maxsize=settings.input_queue_size)
        self.output_queue = self.manager.Queue(maxsize=settings.output_queue_size)

        # Create the workers
        self.num_workers = settings.num_workers
        self.worker_class = worker_class
        self.parent_thread = parent_thread
        self.thread_timeout = thread_timeout
        self.kwargs = kwargs

        self.instantiate_workers(**kwargs)

    def instantiate_workers(
        self,
        **kwargs,
    ) -> None:
        """
        Instantiates the worker processes.

        Args:
            **kwargs: Additional arguments. Used to pass the objects the workers need.
        """

        worker_instance = self.worker_class(self.thread_timeout, **kwargs)

        self.workers = [
            multiprocessing.Process(target=worker_instance.run, args=(self.input_queue, self.output_queue))
            for _ in range(self.num_workers)
        ]
        for worker in self.workers:
            worker.start()

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

    def are_workers_done(self) -> bool:
        """
        Checks if all the workers are done.
        This is not a reliable way to check if the workers are done.

        Returns:
            bool: True if all the workers are done, False otherwise.
        """

        return self.is_input_queue_empty() and self.is_output_queue_empty()

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

        has_items = True
        while has_items:
            try:
                results.append(self.output_queue.get_nowait())
            except:
                has_items = False
        return results

    def shutdown(self) -> None:
        """
        Shuts down the worker manager and its workers.
        """

        for worker in self.workers:
            self.input_queue.put(((0, 0), None))
        for worker in self.workers:
            worker.join()

    def __del__(self) -> None:
        """
        Destructor.
        """

        # Try except to avoid multiple calls to shutdown
        try:
            self.shutdown()
        except Exception as e:
            pass


class CraterBuilderWorker:
    """
    CraterBuilderWorker class. This class is responsible for generating the craters.
    """

    def __init__(
        self,
        thread_timeout: float = 1.0,
        builder: CraterBuilder = None,
    ) -> None:
        """
        Args:
            thread_timeout (float): The timeout for the worker thread. (seconds)
            builder (CraterBuilder): The crater builder.
        """

        self.thread_timeout = thread_timeout
        self.builder = copy.copy(builder)

    def run(self, input_queue: multiprocessing.Queue, output_queue: multiprocessing.Queue) -> None:
        """
        The main function of the worker process.
        This function is called when the worker process is started.
        It takes craters metadata and coordinates from the input queue
        and generates images with inprinted craters.

        Args:
            input_queue (multiprocessing.JoinableQueue): The input queue.
            output_queue (multiprocessing.JoinableQueue): The output queue.
        """

        while True:
            try:
                coords, crater_meta_data = input_queue.get(timeout=self.thread_timeout)
                if crater_meta_data is None:
                    break
                data_not_in_queue = True
                out = (coords, self.builder.generateCraters(crater_meta_data, coords))
                while data_not_in_queue:
                    try:
                        output_queue.put(out, timeout=0.01)
                        data_not_in_queue = False
                    except:
                        pass
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
            parent_thread (threading.Thread): The parent thread.
            thread_timeout (float): The timeout for the worker thread. (seconds)
            builder (CraterBuilder): The crater builder.
        """

        super().__init__(
            settings=settings,
            worker_class=CraterBuilderWorker,
            builder=builder,
            parent_thread=parent_thread,
            thread_timeout=thread_timeout,
        )


class BicubicInterpolatorWorker:
    """
    BicubicInterpolatorWorker class. This class is responsible for interpolating the
    terrain data.
    """

    def __init__(
        self,
        thread_timeout: float = 1.0,
        interp: Interpolator = None,
    ):
        """
        Args:
            thead_timeout (float): The timeout for the worker thread. (seconds)
            interp (Interpolator): The interpolator.
        """

        self.interpolator = copy.copy(interp)
        self.thread_timeout = thread_timeout

    def run(
        self,
        input_queue: multiprocessing.Queue,
        output_queue: multiprocessing.Queue,
    ) -> None:
        """
        The main function of the worker process.
        This function is called when the worker process is started.
        It takes terrain data from the input queue and interpolates it.

        Args:
            input_queue (multiprocessing.JoinableQueue): The input queue.
            output_queue (multiprocessing.JoinableQueue): The output queue.
        """

        while True:
            try:
                coords, data = input_queue.get(timeout=self.thread_timeout)
                if data is None:
                    break

                out = (coords, self.interpolator.interpolate(data))
                data_not_in_queue = True
                while data_not_in_queue:
                    try:
                        output_queue.put(out, timeout=0.01)
                        data_not_in_queue = False
                    except:
                        pass
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
            parent_thread (threading.Thread): The parent thread.
            thread_timeout (float): The timeout for the worker thread. (seconds
        """

        super().__init__(
            settings=settings,
            worker_class=BicubicInterpolatorWorker,
            interp=interp,
            parent_thread=parent_thread,
            thread_timeout=thread_timeout,
        )
        cv2.setNumThreads(num_cv2_threads)


class ThreadMonitor:
    def __init__(self):
        self.event = threading.Event()
        self.thread = threading.Thread(target=self.monitor_main_thread)
        self.thread.start()

    def monitor_main_thread(self) -> None:
        """
        Monitors the main thread.
        This function is used to monitor the main thread and check if it is still alive.
        """

        while not self.event.is_set():
            time.sleep(1)
            if not threading.main_thread().is_alive():
                print("Main thread is dead, shutting down workers.")
                break
