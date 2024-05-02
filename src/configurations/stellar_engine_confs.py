import dataclasses
import datetime
import os

@dataclasses.dataclass
class Date:
    year: int = 2024
    month: int = 5
    day: int = 11
    hour: int = 16
    minute: int = 30

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
class StellarEngineConf:
    start_date: datetime.datetime = dataclasses.field(default_factory=Date)
    time_scale: float = 36000.0
    update_interval: int = 600.0
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

