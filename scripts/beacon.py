class Beacon:
    def __init__(self, type, boundingBoxPos, angle, img_part_nr):
        self.type            = type
        self.boundindBoxPos  = boundingBoxPos
        self.car_gps         = []
        self.image_part_nr   = img_part_nr
        self.beacon_gps      = []
        self.beacon_angle = angle

    def setCarGPS(self, gps):
        self.car_gps = gps

        # debugging - remove later
        print("image part in which beacon was found:")
        print(self.image_part_nr)
        print("Type of found beacon:")
        print(self.type)
        print("gps of car when beacon was found: ")
        print(self.car_gps)
        print("bounding box pixel position of found beacon:")
        print(self.boundindBoxPos)
        print()
        print()

    def getCarGps(self): 
        return self.car_gps

    def setBeaconGps(self, beacon_gps):
        self.beacon_gps = beacon_gps

    def getBoundingBox(self):
        return self.boundindBoxPos
    
    def getBeaconData(self):
        return [self.type, self.beaconLatitude, self.beaconLongitude]

    def getBeaconAngle(self):
        return self._beacon_angle
        
    def SetBeaconAngle(self, angle):
        self._beacon_angle = angle
        
