from typing import Tuple
import numpy as np
import warp as wp


@wp.kernel
def gen_perlin_noise(
    in_: wp.array(dtype=wp.vec2f),
    out: wp.array(dtype=wp.float32),
    seed: wp.uint32,
    coord: wp.vec2f,
):
    tid = wp.tid()
    out[tid] = wp.noise(seed, in_[tid] + coord)


class GenerateTerrainNoise:
    def __init__(
        self,
        seed: int,
        perlin_scale: float,
        displacement_scale: float,
        height: int,
        width: int,
    ):
        self.seed = seed
        self.perlin_scale = perlin_scale
        self.displacement_scale = displacement_scale
        self.height = height
        self.width = width

        self.instantiate_numpy_buffers()
        self.instantiate_warp_buffers()

    def instantiate_numpy_buffers(self):
        x = np.linspace(-self.perlin_scale / 2, self.perlin_scale / 2, self.width)
        y = np.linspace(-self.perlin_scale / 2, self.perlin_scale / 2, self.height)
        xx, yy = np.meshgrid(x, y)
        x = xx.flatten()
        y = yy.flatten()
        self.xy = np.stack((x, y)).T

    def instantiate_warp_buffers(self):
        self.xy_warp = wp.array(self.xy, dtype=wp.vec2f, device="cuda")
        self.noise = wp.zeros(
            (self.height * self.width), dtype=wp.float32, device="cuda"
        )

    def generate_noise(self, coordinates: Tuple[float, float]) -> np.ndarray:
        shift = np.array(coordinates) * self.displacement_scale
        wp.launch(
            kernel=gen_perlin_noise,
            dim=(self.height * self.width,),
            inputs=[
                self.xy_warp,
                self.noise,
                self.seed,
                shift,
            ],
        )

        return self.noise.numpy().reshape((self.height, self.width))


if __name__ == "__main__":

    wp.init()
    from matplotlib import pyplot as plt
    import math
    import time

    seed_0 = 0
    seed_1 = 1
    seed_2 = 2
    perlin_scale_0 = 1
    perlin_scale_1 = 20
    perlin_scale_2 = 100
    displacement_scale_0 = 0.01
    displacement_scale_1 = 0.2
    displacement_scale_2 = 1
    height_0 = 100
    height_1 = 10
    height_2 = 1

    width = 1000
    height = 1000
    coordinates = (0, 0)

    im_data = None
    times = []
    for i in range(100):
        img = np.zeros((height, width))
        coordinates = (math.sin(i / 10) * 25, math.cos(i / 10) * 25)
        start = time.time()
        gen_0 = GenerateTerrainNoise(
            seed_0, perlin_scale_0, displacement_scale_0, height, width
        )
        gen_1 = GenerateTerrainNoise(
            seed_1, perlin_scale_1, displacement_scale_1, height, width
        )
        gen_2 = GenerateTerrainNoise(
            seed_2, perlin_scale_2, displacement_scale_2, height, width
        )
        img += gen_0.generate_noise(coordinates) * height_0
        img += gen_1.generate_noise(coordinates) * height_1
        img += gen_2.generate_noise(coordinates) * height_2
        end = time.time()
        times.append(end - start)

        if im_data is None:
            im_data = plt.imshow(img, cmap="terrain")
            plt.title("Perlin Noise generated terrain demo")
        else:
            im_data.set_data(img)
        plt.pause(0.01)
        plt.draw()

    print(f"Average time per generated terrain: {np.mean(times)}")
