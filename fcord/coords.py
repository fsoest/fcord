from typing import Self
import math


class Coordinate:
    def __init__(self):
        self.r = []

    def __len__(self) -> float:
        """Returns the l2 norm of the vector."""
        return sum([x ** 2 for x in self.r]) ** 0.5

    def l2_norm(self) -> float:
        return len(self)


class CartesianCoord(Coordinate):
    def __init__(self, x: float, y: float, z: float):
        super().__init__()
        self.x = x
        self.y = y
        self.z = z
        self.r = [x, y, z]

    def __repr__(self) -> str:
        return f"CartesianCoord({self.x}, {self.y}, {self.z})"

    def __add__(self, other):
        if isinstance(other, CartesianCoord):
            return CartesianCoord(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            raise ValueError(f"Cannot add CartesianCoord to {type(other)}")

    def __sub__(self, other):
        if isinstance(other, CartesianCoord):
            return CartesianCoord(self.x - other.x, self.y - other.y, self.z - other.z)
        else:
            raise ValueError(f"Cannot subtract CartesianCoord from {type(other)}")


class GPSCoord:
    def __init__(self, lat: float, lon: float, alt: float):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.r = [lat, lon, alt]

    def __repr__(self) -> str:
        return f"GPSCoord({self.lat}, {self.lon}, {self.alt})"

    def ecef(self):
        """Returns the ECEF coordinates of the GPS coordinates."""
        a = 6378137.0  # Semi-major axis
        f = 1 / 298.257223563  # Flattening
        b = a * (1 - f)
        e2 = (a ** 2 - b ** 2) / a ** 2  # Square of eccentricity
        lat_rad = math.radians(self.lat)
        lon_rad = math.radians(self.lon)

        N = a / (1 - e2 * math.sin(lat_rad) ** 2) ** 0.5
        x = (N + self.alt) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + self.alt) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - e2) + self.alt) * math.sin(lat_rad)
        return CartesianCoord(x, y, z)

    def distance(self, other: Self) -> float:
        return (self.ecef() - other.ecef()).l2_norm()


class ENUCoord(Coordinate):
    def __init__(self, e: float, n: float, u: float, global_origin: GPSCoord = None):
        super().__init__()
        self.global_origin = global_origin
        self.e = e
        self.n = n
        self.u = u
        self.r = [e, n, u]

    def __repr__(self) -> str:
        return f"ENUCoord({self.e}, {self.n}, {self.u})"

    def to_ned(self) -> "NEDCoord":
        return NEDCoord(self.n, self.e, -self.u, self.global_origin)

    def __add__(self, other) -> Self:
        if isinstance(other, ENUCoord):
            return ENUCoord(self.e + other.e, self.n + other.n, self.u + other.u)
        elif isinstance(other, NEDCoord):
            return self + other.to_enu()
        else:
            raise ValueError(f"Cannot add ENUCoord to {type(other)}")

    def __sub__(self, other) -> Self:
        if isinstance(other, ENUCoord):
            return ENUCoord(self.e - other.e, self.n - other.n, self.u - other.u)
        elif isinstance(other, NEDCoord):
            return self - other.to_enu()
        else:
            raise ValueError(f"Cannot subtract ENUCoord from {type(other)}")


class NEDCoord(Coordinate):
    def __init__(self, n: float, e: float, d: float, global_origin: GPSCoord = None):
        super().__init__()
        self.global_origin = global_origin
        self.n = n
        self.e = e
        self.d = d
        self.r = [n, e, d]

    def __repr__(self) -> str:
        return f"NEDCoord({self.n}, {self.e}, {self.d})"

    def to_enu(self) -> ENUCoord:
        return ENUCoord(self.e, self.n, -self.d, self.global_origin)

    def __add__(self, other) -> Self:
        if isinstance(other, NEDCoord):
            return NEDCoord(self.n + other.n, self.e + other.e, self.d + other.d)
        elif isinstance(other, ENUCoord):
            return self + other.to_ned()
        else:
            raise ValueError(f"Cannot add NEDCoord to {type(other)}")

    def __sub__(self, other) -> Self:
        if isinstance(other, NEDCoord):
            return NEDCoord(self.n - other.n, self.e - other.e, self.d - other.d)
        elif isinstance(other, ENUCoord):
            return self - other.to_ned()
        else:
            raise ValueError(f"Cannot subtract NEDCoord from {type(other)}")