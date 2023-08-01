from math import asin, atan2, cos, degrees, radians, sin
from beacon import Beacon
import distance_calculation as distance

def calculateLocation(carLatitude, carLongitude, d, bearing, R=6371):
    """
    carLatitude: initial latitude, in degrees
    carLongitude: initial longitude, in degrees
    d: distance from car to beacon (in km)
    bearing: (true) heading in degrees
    R: optional radius of sphere, defaults to mean radius of earth

    Returns new lat/lon coordinate {d}km from initial, in degrees
    """
    lat = radians(carLatitude)
    lon = radians(carLongitude)
    a = radians(bearing)
    beaconLat = asin(sin(lat) * cos(d/R) + cos(lat) * sin(d/R) * cos(a))
    beaconLon = lon + atan2(
        sin(a) * sin(d/R) * cos(lat),
        cos(d/R) - sin(lat) * sin(beaconLat)
    )
    return (degrees(beaconLat), degrees(beaconLon))

def calculatePosition(currentBeacon:Beacon):
    angle = 0 #TODO
    distanceFromCar = distance.calculateDistance(currentBeacon)

    lat = currentBeacon.carLatitude
    lon = currentBeacon.carLongitude
    
    beaconLatitud, beaconLongitud = calculateLocation(lat, lon, distanceFromCar, angle)
    currentBeacon.setGNSS(beaconLatitud, beaconLongitud)