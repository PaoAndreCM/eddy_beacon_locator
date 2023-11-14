import json

class Beacon:
    def __init__(self, type, boundingBoxPos, angle, distance, confidence, car_gps):
        self.type             = type
        self.boundindBoxPos   = boundingBoxPos
        self.car_gps          = car_gps
        self.beacon_gps       = []
        self.beacon_angle     = angle
        self.beacon_distance  = distance
        self.confidence       = confidence
        self.geojson_String   = None

    def setCarGPS(self, gps):
        self.car_gps = gps

    def getCarGps(self): 
        return self.car_gps

    def setBeaconGps(self, beacon_gps):
        self.beacon_gps = beacon_gps

    def getBoundingBox(self):
        return self.boundindBoxPos

    def getBeaconAngle(self):
        return self.beacon_angle
    
    def getBeaconDistance(self):
        return self.beacon_distance
        
    def SetBeaconAngle(self, angle):
        self._beacon_angle = angle 
    
    def SetConfidence(self, confidence):
        self.confidence = confidence

    def getBeaconData(self):
        return [self.type, self.beacon_gps[0], self.beacon_gps[1], self.confidence, self.beacon_angle]
        
