import pyproj
import numpy as np

def gps_to_enu(latitude, longitude, altitude, ref_latitude, ref_longitude, ref_altitude):
    # Define WGS84 ellipsoid
    wgs84 = pyproj.Geod(ellps='WGS84')

    # Convert GPS coordinates to ECEF (cartesian) coordinates
    ref_ecef = pyproj.Geod(ellps='WGS84').fwd(ref_longitude, ref_latitude, ref_altitude, 0, 0, 0)
    point_ecef = pyproj.Geod(ellps='WGS84').fwd(longitude, latitude, altitude, 0, 0, 0)

    # Calculate the difference vector from reference point to GPS point
    delta_ecef = np.array(point_ecef) - np.array(ref_ecef)

    # Define rotation matrix from ECEF to ENU
    lon, lat, _ = ref_ecef
    t = np.array([
        [-np.sin(lon),             np.cos(lon),              0],
        [-np.sin(lat)*np.cos(lon),-np.sin(lat)*np.sin(lon), np.cos(lat)],
        [ np.cos(lat)*np.cos(lon), np.cos(lat)*np.sin(lon), np.sin(lat)]
    ])

    # Rotate the difference vector to get ENU coordinates
    enu = np.dot(t, delta_ecef)
    enu *= np.array([111000, 111000, 1])
    enu = np.around(enu,decimals=3)
    return enu

# Example usage:
latitude = 37.7749  # GPS latitude of the point to convert
longitude = -122.4194  # GPS longitude of the point to convert
altitude = 0  # GPS altitude of the point to convert
ref_latitude = 37.7759  # GPS latitude of the reference point (origin)
ref_longitude = -122.4194  # GPS longitude of the reference point (origin)
ref_altitude = 0  # GPS altitude of the reference point (origin)

enu_coordinates = gps_to_enu(latitude, longitude, altitude, ref_latitude, ref_longitude, ref_altitude)
print("ENU coordinates:", enu_coordinates)
