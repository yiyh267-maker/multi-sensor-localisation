import math

class CoordConverter:
    def __init__(self, ref_lat, ref_lon):
        self.ref_lat = math.radians(ref_lat)
        self.ref_lon = math.radians(ref_lon)
        self.R = 6378137.0

    def wgs84_to_enu(self, lat, lon):
        phi = math.radians(lat)
        lam = math.radians(lon)
        y = (phi - self.ref_lat) * self.R
        x = (lam - self.ref_lon) * self.R * math.cos(self.ref_lat)
        return x, y
