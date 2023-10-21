from math import asin, atan2, cos, degrees, radians, sin, degrees
from beacon import Beacon
import distance_calculation as distance

COMPASS_PHASE_SHIFT= 135

def car_orientation_angle_calculation(angle_radians):
    angle_degrees = degrees(angle_radians) % 360
    if angle_degrees < 0:
        angle_degrees += 360
    elif angle_degrees == 360:
        angle_degrees = 0
    return angle_degrees + COMPASS_PHASE_SHIFT

def calculateLocation(carLatitude, carLongitude, d, beacon_angle, z_rot, R=6371):
    """
    carLatitude: initial latitude, in degrees
    carLongitude: initial longitude, in degrees
    d: distance from car to beacon (in km)
    z_rot: rotation of the car w.r.t. to z axis (radians)
    R: optional radius of sphere, defaults to mean radius of earth

    Returns new lat/lon coordinate {d}km from initial, in degrees
    """
    lat = radians(carLatitude) 
    lon = radians(carLongitude)
    bearing = car_orientation_angle_calculation(z_rot)
    a = radians(bearing + beacon_angle)
    beaconLat = asin(sin(lat) * cos(d/R) + cos(lat) * sin(d/R) * cos(a))
    beaconLon = lon + atan2(
        sin(a) * sin(d/R) * cos(lat),
        cos(d/R) - sin(lat) * sin(beaconLat)
    )
    return (degrees(beaconLat), degrees(beaconLon))

"""
def calculatePosition(currentBeacon:Beacon):
    angle = Beacon.getBeaconAngle
    distanceFromCar = distance.calculate_distance(currentBeacon)

    lat = currentBeacon.carLatitude
    lon = currentBeacon.carLongitude
    
    beaconLatitud, beaconLongitud = calculateLocation(lat, lon, distanceFromCar, angle)
    currentBeacon.setGNSS(beaconLatitud, beaconLongitud)"""