from beacon import Beacon

listOfBeacons = []

def objDetection(image):
    #... add object detection

    listOfBeacons.append(beacon.Beacon(1, None)) #dummy beacon type 1, no bounding box
    listOfBeacons.append(beacon.Beacon(2, None)) #dummy beacon type 2, no bounding box
    return listOfBeacons