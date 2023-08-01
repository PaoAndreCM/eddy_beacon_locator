#!/usr/bin/env python3
"""
Construction site beacon detection and localization project
***********************************************************

*** Main module ***
This module controls the whole process. It subscribes to the topic of the ladybug camera.
When messages from that topic are received, they are converted to cv2 image data. A submodul for
for detection of possible beacons on that image is called. It returns a list of class beacons (detected ones).
Since rosbags are played synchronously, the current GNSS sensor data should belong to the car position
while taking the picture (-> verify with Prof if thats really the case). The car GNSS data is stored inside the
beacon class. As soon as the complete rosbag has been played, we iterate through the list of detected beacons
and calculate the absolute position of the beacon in a submodule (module is still missing (distance and angle calc),
the function returns dummy values at the moment). The absolute position is stored inside the beacon class.
Then json objects are created and published to a new topic (not tested)
"""

# Imports
import rospy
import cv2
import json
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import detect_beacon as detect
import beacon
import position_calculation as pos


rospy.init_node('beacon_detection', anonymous=True)  # Initialization of ROS node
bridge          = CvBridge()                         # CVBridge for converting msg data to opencv images
listOfBeacons   = []


def callback(msg):
    try:
        rospy.loginfo("message received.")
        image_data = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') # Conversion of msg to cv2 image
        newBeacons = detect.objDetection(image_data)                           # Run objection detetion on image
        
        for detectedBeacon in newBeacons:                                      # Processing of newly detected beacons
            latitude  = None
            longitude = None

            gps_lat_msg  = rospy.wait_for_message("/gnss_latitude", String)         # Get latitude of car
            gps_long_msg = rospy.wait_for_message("/gnss_longitude", String)        # Get longitude of car

            latitude  = float(gps_lat_msg.data)
            longitude = float(gps_long_msg.data)
            detectedBeacon.setCarGNSS(latitude, longitude)

            if latitude is not None and longitude is not None:
                listOfBeacons.append(detectedBeacon)

    except Exception as error:
        print("Error in beacon detection process:", error)
    
    # Debugging: Create a jpg file to check correct reading of rosbag
    #im_data = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #cv2.imwrite(str(msg.header.stamp.to_sec()) + ".png", im_data) 

def main():
    rospy.Subscriber("/camera/image_raw", Image, callback=callback)            # Subscribe to the camera image topic and run detection
    rospy.spin()                                                      # wait till all msg from topic have been played

    for currentBeacon in listOfBeacons:
        try:
            pos.calculatePosition(currentBeacon)
            type, lat, long = currentBeacon.getBeaconData()
            objData = {
                "type":      str(type),
                "latitude":  str(lat),
                "longitude": str(long)
            }

            jsonObj = json.dumps(objData)
            publisher = rospy.Publisher('beaconLocation', String, queue_size=10)
            rate = rospy.Rate(1)

            # Debugging: Create json files to check content of objects
            type, _, _ = currentBeacon.getBeaconData()
            with open(str(type) + ".json", "w") as json_file:
                json.dump(jsonObj, json_file)

            while not rospy.is_shutdown():
                publisher.publish(jsonObj)
                rate.sleep()
        except Exception as error:
            print("Error in position calculation:", error)

        

    




if __name__ == '__main__':
    main()