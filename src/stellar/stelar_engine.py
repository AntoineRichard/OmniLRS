from skyfield.api import PlanetaryConstants, load
from skyfield.vectorlib import VectorSum
from matplotlib import pyplot as plt
from typing import Tuple
import dataclasses
import numpy as np
import datetime
import math
import os
    
@dataclasses.dataclass
class Date:
    year: int
    month: int
    day: int
    hour: int
    minute: int

    def __post_init__(self):
        self.year = int(self.year)
        self.month = int(self.month)
        self.day = int(self.day)
        self.hour = int(self.hour)
        self.minute = int(self.minute)

        assert 1 <= self.month <= 12, "Month must be between 1 and 12."
        assert 1 <= self.day <= 31, "Day must be between 1 and 31."
        assert 0 <= self.hour <= 23, "Hour must be between 0 and 23."
        assert 0 <= self.minute <= 59, "Minute must be between 0 and 59."
        assert 1970 <= self.year <= 2050, "Year must be between 1970 and 2050."

@dataclasses.dataclass
class StellarEngineConfig:
    start_date: datetime.datetime
    time_scale: float
    update_interval: int
    distance_scale: float = 1e-3
    ephemeris_path: str = "assets/Ephemeris"
    ephemeris: str = "de421.bsp"
    moon_pa: str = "moon_pa_de421_1900-2050.bpc"
    moon_tf: str = "moon_080317.tf"
    pck: str = "pck00008.tpc"
    frame: str = "MOON_ME_DE421"

    def __post_init__(self):
        if isinstance(self.start_date, dict):
            d = Date(**self.start_date)
        else:
            d = self.start_date
        self.start_date = datetime.datetime(d.year, d.month, d.day, d.hour, d.minute, tzinfo=datetime.timezone.utc)

        assert 0 < self.time_scale, "Time scale must be greater than 0."
        assert 0 < self.update_interval, "Update interval must be greater than 0."
        assert 0 < self.distance_scale, "Distance scale must be greater than 0."

        assert self.ephemeris in ["de421.bsp"], "Ephemeris must be one of 'de421.bsp'."
        assert self.moon_pa in ["moon_pa_de421_1900-2050.bpc"], "Moon PA must be one of 'moon_pa_de421_1900-2050.bpc'."
        assert self.moon_tf in ["moon_080317.tf"], "Moon TF must be one of 'moon_080317.tf'."
        assert self.pck in ["pck00008.tpc", "pck00011.tpc"], "PCK must be one of 'pck00008.tpc', 'pck00011.tpc'."
        assert self.frame in ["MOON_ME_DE421"], "Frame must be one of 'MOON_ME_DE421'."

        self.ephemeris = os.path.join(self.ephemeris_path, self.ephemeris)
        self.moon_pa = os.path.join(self.ephemeris_path, self.moon_pa)
        self.moon_tf = os.path.join(self.ephemeris_path, self.moon_tf)
        self.pck = os.path.join(self.ephemeris_path, self.pck)

        assert os.path.exists(self.ephemeris), f"Ephemeris file {self.ephemeris} does not exist."
        assert os.path.exists(self.moon_pa), f"Moon PA file {self.moon_pa} does not exist."
        assert os.path.exists(self.moon_tf), f"Moon TF file {self.moon_tf} does not exist."
        assert os.path.exists(self.pck), f"PCK file {self.pck} does not exist."


class StellarEngine:
    """
    StellarEngine class to compute the positions of celestial bodies.
    After initializing the StellarEngine, set the latitude and longitude of the observer using set_latlon method.
    Then, call the update method to update the current time.
    Finally, use the get_altaz, get_radec, get_position, or get_local_position methods to get the position of a given body.
    """

    def __init__(self, cfg: StellarEngineConfig)-> None:
        """
        Initialize the StellarEngine.

        Args:
            cfg (StellarEngineConfig): the configuration for the StellarEngine.
        """

        self.cfg = cfg
        self.ts = load.timescale()
        self.current_time = cfg.start_date
        self.load_ephemeris()
        self.last_update = datetime.datetime.fromtimestamp(0, datetime.timezone.utc)
        self.t = self.ts.from_datetime(self.current_time)

    def load_ephemeris(self) -> None:
        """
        Load the ephemeris and planetary constants.
        """

        self.eph = load(self.cfg.ephemeris)
        self.earth, self.moon, self.sun, self.venus = self.eph['earth'], self.eph['moon'], self.eph['sun'], self.eph['venus']
        self.pc = PlanetaryConstants()
        self.pc.read_text(load(self.cfg.moon_tf))
        self.pc.read_text(load(self.cfg.pck))
        self.pc.read_binary(load(self.cfg.moon_pa))
        self.frame = self.pc.build_frame_named(self.cfg.frame)

    def set_latlon(self, lat: float, lon: float) -> None:
        """
        Set the latitude and longitude of the observer.

        Args:
            lat (float): the latitude of the observer.
            lon (float): the longitude of the observer.
        """

        self.observer = self.moon + self.pc.build_latlon_degrees(self.frame, lat, lon)

    def get_altaz(self, body: VectorSum) -> Tuple[float, float, float]:
        """
        Get the altitude, azimuth, and distance of the body in the observer's frame.

        Args:
            body (VectorSum): the body to get the altitude, azimuth, and distance of.
        
        Returns:
            Tuple[float, float, float]: the altitude, azimuth, and distance of the body in the observer's frame.
        """

        apparent = self.observer.at(self.t).observe(body).apparent()
        alt, az, distance = apparent.altaz()
        return alt.degrees, az.degrees, distance.m
    
    def get_radec(self, body: VectorSum) -> Tuple[float, float, float]:
        """
        Get the ra, dec, and distance of the body in the observer's frame.

        Args:
            body (VectorSum): the body to get the ra, dec, and distance of.

        Returns:
            Tuple[float, float, float]: the ra, dec, and distance of the body in the observer's frame.
        """

        apparent = self.observer.at(self.t).observe(body).apparent()
        ra, dec, distance = apparent.radec(epoch='date')
        return ra, dec, distance
    
    def get_position(self, body: VectorSum) -> Tuple[float, float, float]:
        """
        Get the position of the body in the observer's frame.

        Args:
            body (VectorSum): the body to get the position of.
        
        Returns:
            Tuple[float, float, float]: the x, y, z position of the body in the observer's frame.
        """

        apparent = self.observer.at(self.t).observe(body).apparent()
        return apparent.position.to('m').value
    
    def get_local_position(self, body: VectorSum) -> Tuple[float, float, float]:
        """
        Get the local position of the body in the observer's frame.

        Args:
            body (VectorSum): the body to get the position of.

        Returns:
            Tuple[float, float, float]: the x, y, z position of the body in the observer's frame.
        """

        alt, az, dist = self.get_altaz(body)
        xyz = (dist * math.cos(math.radians(alt)) * math.cos(math.radians(az)),
                dist * math.cos(math.radians(alt)) * math.sin(math.radians(az)),
                dist * math.sin(math.radians(alt)))
        return xyz
    
    def update(self, dt: float)  -> None:
        """
        Update the current time by dt seconds.
        If the time since the last update is greater than the update interval, update stellar time.
        Note that the delta of time is scaled by the time scale.
        
        Args:
            dt (float): time in seconds to update the current time.
        """
        self.current_time += datetime.timedelta(seconds=dt * self.cfg.time_scale)

        time_delta = self.current_time - self.last_update
        if time_delta.total_seconds() >= self.cfg.update_interval:
            self.last_update = self.current_time
            self.t = self.ts.from_datetime(self.current_time)


if __name__ == "__main__":
    date = Date(year=2024, month=5, day=1, hour=11, minute=50)
    SEC = StellarEngineConfig(start_date=date, time_scale=25, update_interval=1)
    SC = StellarEngine(cfg=SEC)
    SC.set_latlon(-26.3, 46.8)

    start_time = datetime.datetime.now()
    print("Computing positions...")
    earth_pos = []
    sun_pos = []
    venus_pos = []
    for i in range(10000):
        SC.update(600)
        earth_pos.append(SC.get_position(SC.earth))
        sun_pos.append(SC.get_position(SC.sun))
        venus_pos.append(SC.get_position(SC.venus))
    end_time = datetime.datetime.now()

    print("Time taken:", (end_time - start_time).total_seconds(), "seconds")
    print("iterations per second:", 10000 / (end_time - start_time).total_seconds())

    # 3D plot of the positions
    earth_pos = np.array(earth_pos)
    sun_pos = np.array(sun_pos)
    venus_pos = np.array(venus_pos)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(earth_pos[:, 0], earth_pos[:, 1], earth_pos[:, 2], label="Earth")
    ax.plot(sun_pos[:, 0], sun_pos[:, 1], sun_pos[:, 2], label="Sun")
    ax.plot(venus_pos[:, 0], venus_pos[:, 1], venus_pos[:, 2], label="Venus")
    plt.legend()

    date = Date(year=2019, month=12, day=20, hour=11, minute=5)
    SEC = StellarEngineConfig(start_date=date, time_scale=25, update_interval=1)
    SC = StellarEngine(cfg=SEC)
    SC.set_latlon(-26.3, 46.8)

    start_time = datetime.datetime.now()
    print("Computing altaz...")
    earth_altaz = []
    sun_altaz = []
    venus_altaz = []
    for i in range(10000):
        SC.update(600)
        earth_altaz.append(SC.get_altaz(SC.earth))
        sun_altaz.append(SC.get_altaz(SC.sun))
        venus_altaz.append(SC.get_altaz(SC.venus))
    end_time = datetime.datetime.now()

    print("Time taken:", (end_time - start_time).total_seconds(), "seconds")
    print("iterations per second:", 10000 / (end_time - start_time).total_seconds())

    # 3D plot of the positions
    earth_altaz = np.array(earth_altaz)
    sun_altaz = np.array(sun_altaz)
    venus_altaz = np.array(venus_altaz)
    fig = plt.figure()
    plt.plot(earth_altaz[:,0], label="Earth")
    plt.plot(sun_altaz[:,0], label="Sun")
    plt.plot(venus_altaz[:,0], label="Venus")
    plt.title("Altitude (degrees)")
    plt.legend()
    fig = plt.figure()
    plt.plot(earth_altaz[:,1], label="Earth")
    plt.plot(sun_altaz[:,1], label="Sun")
    plt.plot(venus_altaz[:,1], label="Venus")
    plt.title("Azimuth (degrees)")
    plt.legend()

    date = Date(year=2019, month=12, day=20, hour=11, minute=5)
    SEC = StellarEngineConfig(start_date=date, time_scale=25, update_interval=1)
    SC = StellarEngine(cfg=SEC)
    SC.set_latlon(-26.3, 46.8)

    start_time = datetime.datetime.now()
    print("Computing positions...")
    earth_pos = []
    sun_pos = []
    venus_pos = []
    for i in range(10000):
        SC.update(600)
        earth_pos.append(SC.get_local_position(SC.earth))
        sun_pos.append(SC.get_local_position(SC.sun))
        venus_pos.append(SC.get_local_position(SC.venus))
    end_time = datetime.datetime.now()

    print("Time taken:", (end_time - start_time).total_seconds(), "seconds")
    print("iterations per second:", 10000 / (end_time - start_time).total_seconds())

    # 3D plot of the positions
    earth_pos = np.array(earth_pos)
    sun_pos = np.array(sun_pos)
    venus_pos = np.array(venus_pos)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(earth_pos[:, 0], earth_pos[:, 1], earth_pos[:, 2], label="Earth")
    ax.plot(sun_pos[:, 0], sun_pos[:, 1], sun_pos[:, 2], label="Sun")
    ax.plot(venus_pos[:, 0], venus_pos[:, 1], venus_pos[:, 2], label="Venus")
    plt.legend()
    plt.show()


