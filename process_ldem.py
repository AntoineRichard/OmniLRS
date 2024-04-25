from osgeo import gdal
import numpy as np
import os
ds = gdal.Open("tmp/ldem_87s_5mpp.tif")
channel = np.array(ds.GetRasterBand(1).ReadAsArray())
os.makedirs("assets/Terrains/SouthPole/LDEM_87S",exist_ok=True)
np.save("assets/Terrains/SouthPole/LDEM_87S/dem.npy",channel)
