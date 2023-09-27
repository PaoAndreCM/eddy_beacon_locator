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
import os
import cv2
import json
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import detect_beacon as detect
import beacon
import position_calculation as pos
from sbg_driver.msg import SbgGpsPos as gps


rospy.init_node('beacon_detection', anonymous=True)  # Initialization of ROS node
bridge          = CvBridge()                         # CVBridge for converting msg data to opencv images
list_of_beacons = []
detected_beacons = []


def callback_cam(msg):
    global gps_data
    global list_of_beacons
    try:
        #rospy.loginfo("IMG received.")
        detected_beacons.clear()
        image_data = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') # Conversion of msg to cv2 image
        img_90 = cv2.rotate(image_data, cv2.ROTATE_90_CLOCKWISE)


        try:
            # *****************Split images*****************************
            # Get the dimensions of the combined image
            height, width = img_90.shape[:2]

            # Calculate the width of each individual picture
            picture_width = width // 6
            w_to_cut = 260
            # Crop the image into 6 separate regions
            pictures = [img_90[:, i * picture_width+w_to_cut: (i + 1) * picture_width-w_to_cut] for i in range(6)]
            pictures[3] = img_90[:, 3 * picture_width + w_to_cut + 20: (3 + 1) * picture_width - w_to_cut + 20]
            rearranged_pictures = [pictures[5], pictures[4], pictures[3], pictures[2], pictures[1]]
        except Exception as e:
            print("Error in image splitting process", e)

        # run beacon detection on each of the individual pics
        image_number = 1
        for img in rearranged_pictures:
            try:  
                detected_beacons.extend(detect.objDetection(img, cfg, weights, classes, image_number))
            except Exception as e:
                print("Error in detection module:", e)
            image_number += 1
        
        # add car gps info of car to beacon and append to list 
        for beacon in detected_beacons:
            beacon.setCarGPS(gps_data)
            list_of_beacons.append(beacon) 

        # for debugging
        if detected_beacons == []: 
            print("no beacon found")
 
    except Exception as error: 
        print("Error in detection function of main module:", error)
    
    # Debugging: Create a png file to check correct reading of rosbag
    #im_data = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    cv2.imwrite(str(msg.header.stamp.to_sec()) + ".png", img_90)#rearranged_pictures[4]) 

def callback_gps(msg): 
    #rospy.loginfo("car GPS received.")
    global gps_data 
    try: 
        gps_data = (msg.latitude, msg.longitude) 
    except Exception as error:
        print("Error getting gps: ", error)
 
def main():
  
    rospy.Subscriber("/camera/image_raw", Image, callback_cam)    # Subscribe to ladybug cam
    rospy.Subscriber("/sbg/gps_pos", gps , callback_gps)
    rospy.spin()                                                    # wait till all msg from topic have been played

'''
    for current_beacon in list_of_beacons:
        try:
            pos.calculatePosition(current_beacon)
            type, lat, long = current_beacon.getBeaconData()
            objData = {
                "type":      str(type),
                "latitude":  str(gnss_data[0]),#str(lat),
                "longitude": str(gnss_data[1])#str(long)
            }

            jsonObj = json.dumps(objData)
            publisher = rospy.Publisher('beaconLocation', String, queue_size=10)
            rate = rospy.Rate(1)

            # Debugging: Create json files to check content of objects
            type, _, _ = current_beacon.getBeaconData()
            with open(str(type) + ".json", "w") as json_file:
                json.dump(jsonObj, json_file)

            while not rospy.is_shutdown():
                publisher.publish(jsonObj)
                rate.sleep()
        except Exception as error:
            print("Error in position calculation:", error)'''


if __name__ == '__main__':

    classes = ['bake', 'bake2', 'intelliBake']
    gps_data        = []

    #************ADD YOUR PATH HERE**********************
    cfg = os.path.join('/home/jerome/Downloads/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/cfg/yolov4-tiny-custom.cfg')
    weights = os.path.join('/home/jerome/Downloads/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/yolov4-tiny-custom_best.weights')
    #*****************************************************
    
    #cfg = os.path.join('/home/sudi/catkin_ws_beacons/src/darknet_ros_pck/darknet_ros/yolo_network_config/cfg/yolov4-tiny-custom.cfg')
    #weights = os.path.join('/home/sudi/catkin_ws_beacons/src/darknet_ros_pck/darknet_ros/yolo_network_config/weights/yolov4-tiny-custom_best.weights')

    main()
