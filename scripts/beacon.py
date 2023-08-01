class Beacon:
    def __init__(self, type, boundingBoxPos):
        self.type            = type
        self.boundindBoxPos  = boundingBoxPos
        self.carLatitude     = None
        self.carLongitude    = None
        self.beaconLatitude  = None
        self.beaconLongitude = None

    def setCarGNSS(self, latitude, longitude):
        self.carLatitude  = latitude
        self.carLongitude = longitude

    def setGNSS(self, latitude, longitude):
        self.beaconLatitude = latitude
        self.beaconLongitude = longitude

    def getBoundingBox(self):
        return self.boundindBoxPos
    
    def getBeaconData(self):
        return [self.type, self.beaconLatitude, self.beaconLongitude]
        
