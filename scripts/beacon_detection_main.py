#!/usr/bin/env python3
"""
Construction site beacon detection and localization project
***********************************************************

*** Main module ***
This module controls the whole process. It subscribes to the topic of the ladybug camera and car gps.
When messages from that topic are received, they are converted to cv2 image data. A submodul for
for detection of possible beacons on that image is called. It returns a list of class beacons (detected ones).
GPS data is received with a frequency that is 5 times the frequency with what images are received (5Hz vs 1Hz). 
Therefore every detected beacon is associated with the last received car GPS data.
The image is split into 6 peaces, the one showing the sky is dumped and then the angle (relative to car orientation)
and distance of the beacon are determined. From this relative position and the car GPS the aboslute postion (GPS)
of the beacon is calculated and stored in the object. The data is then published.
"""

# Imports
import rospy
import os
import cv2 
import math
#import std_msgs.msg 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import detect_beacon
import beacon
import position_calculation as pos
from sbg_driver.msg import SbgGpsPos as gps
from sbg_driver.msg import SbgEkfEuler as ekf_euler
from eddy_beacon_locator.msg import beacon as beacon_msg


rospy.init_node('beacon_detection', anonymous=True)  # Initialization of ROS node
bridge           = CvBridge()                         # CVBridge for converting msg data to opencv images
list_of_beacons  = []
detected_beacons = []


def callback_cam(msg):
    global gps_data
    global ekf_y_angle
    global list_of_beacons
    global publisher
    global pub_test
    try:
        detected_beacons.clear()
        image_data = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') # Conversion of msg to cv2 image
        img_90 = cv2.rotate(image_data, cv2.ROTATE_90_CLOCKWISE)
        
        
        #********************
        #
        # This part of the function rotates splits the received ladybug image into 6 peaces and stores all of them
        # except the one showing only the sky.
        #
        #********************
        try:
            height, width = img_90.shape[:2]                   
            picture_width = width // 6                          
            w_to_cut = 260                                     

            pictures = [img_90[:, i * picture_width+w_to_cut: (i + 1) * picture_width-w_to_cut] for i in range(6)]
            pictures[3] = img_90[:, 3 * picture_width + w_to_cut + 20: (3 + 1) * picture_width - w_to_cut + 20]
            rearranged_pictures = [pictures[5], pictures[4], pictures[3], pictures[2], pictures[1]]
        except Exception as e:
            print("Error in image splitting process", e)

        #********************
        #
        # This part runs the beacon detection on the separate parts of the image. The image parts are numbered in order
        # to derive the correct angle from the front of the car
        #
        #********************
        image_number = 1
        for img in rearranged_pictures:
            try:  
                detected_beacons.extend(detect_beacon.objDetection(img, cfg, weights, classes, image_number, gps_data))
            except Exception as e:
                print("Error in detection module:", e)
            image_number += 1
        
        #********************
        #
        # The current GPS data of the car is stored for detected beacons.
        # The absolute position (GPS) of the beacon is calculated from its relative position (angle + distance to car)
        #
        #********************
        for beacon in detected_beacons:
            #beacon.setCarGPS(gps_data)                
            #print(pos.calculateLocation(gps_data[0], gps_data[1], (beacon.getBeaconDistance()*10**-(3)), ekf_y_angle, beacon.getBeaconAngle())) 
            carGPS = beacon.getCarGps() 
            beacon.setBeaconGps(pos.calculateLocation(carGPS[0], carGPS[1], (beacon.getBeaconDistance()*10**-(3)), ekf_y_angle, beacon.getBeaconAngle()))
            print(beacon.getBeaconData()) 
            #list_of_beacons.append(beacon) 

            type_id, latitude, longitude, confidence , angle = beacon.getBeaconData()
            beacon_message =  beacon_msg()
            beacon_message.beacon_type = str(type_id)
            beacon_message.latitude = latitude
            beacon_message.longitude = longitude 
            beacon_message.confidence = confidence

            publisher.publish(beacon_message)
        

        pub_test.publish(beacon_msg())

            
  
    except Exception as error: 
        print("Error in detection function of main module:", error)
    
    # Debugging: Create a png file to check correct reading of rosbag
    #im_data = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #cv2.imwrite(str(msg.header.stamp.to_sec()) + ".png", rearranged_pictures[0]) 


#********************
#
# Callback function for received car GPS data. New data overwrites old one
#
#********************
def callback_gps(msg): 
    global gps_data 
    try: 
        gps_data = (msg.latitude, msg.longitude) 
    except Exception as error:
        print("Error getting gps: ", error)

#********************
#
# Callback function for received car rotation w.r.t. to z axis. New data overwrites old 
#
#********************
def callback_ekf(msg): 
    global ekf_y_angle 
    try:  
        ekf_y_angle = msg.angle.z
    except Exception as error: 
        print("Error getting heading of car: ", error)

# Only for testing
 
 
#********************
#
# Main function. Subscription to three rostopics (ladybug_cam, car_gps, car_z_rotation). Detection and position calculation 
# beacons in callback_cam function. Data is pulished inside custom build message type "beacon_msg".
# Todo: Replace rospy.bin() in order not to get stuck here. This line (149) has to be commented out currently in order to publish everything successfully.
#
#********************
def main():
    global publisher
    global pub_test
    rospy.Subscriber("/camera/image_raw", Image, callback_cam, queue_size=1)                           # Subscribe to ladybug cam, object detection and pos calculation in callback
    rospy.Subscriber("/sbg/gps_pos", gps , callback_gps, queue_size=1)                                 # Subscribe to car gps
    rospy.Subscriber("/sbg/ekf_euler", ekf_euler , callback_ekf, queue_size=1)                         # Subscribe to ekf_euler to receive heading of car
    rate = rospy.Rate(10)  								    # Publshing frequency of 10Hz. Might need to be changed
    publisher = rospy.Publisher("trafficBeacon/beacon_pos", beacon_msg, queue_size=10)
    pub_test = rospy.Publisher("trafficBeacon/beacon_pssssos", beacon_msg, queue_size=10)
    rospy.spin()                                                                        # wait till all msg from topic have been played, remove later so won't be stuck here
    
       # create publisher publishing to trafficBeacon/beacon_pos topic
    while not rospy.is_shutdown():
        transmitted_beacons = [] 

        for current_beacon in list_of_beacons:                                               # Iterate through list of beacons, and publish
            try:
                
                                                                      

                type_id, latitude, longitude, confidence = current_beacon.getBeaconData()

                if not rospy.is_shutdown():                                                  # Check if ros is running. If so, publish
                    beacon_message =  beacon_msg()
                    beacon_message.beacon_type = str(type_id)
                    beacon_message.latitude = latitude
                    beacon_message.longitude = longitude
                    beacon_message.confidence = confidence

                    publisher.publish(beacon_message)
                    transmitted_beacons.append(current_beacon)
                    rate.sleep()
            except Exception as error:
                print("Error while publishing: ", error)

        for current in transmitted_beacons:
            list_of_beacons.remove(current)

if __name__ == '__main__':

    classes     = ['bake', 'bake2', 'intellibeacon']
    gps_data    = []
    ekf_y_angle = None


    #cfg = os.path.join('/home/tesla/catkin_ws/src/eddy_beacon_locator/scripts/yolo_network_config/cfg/yolov4-tiny-custom.cfg')
    #weights = os.path.join('/home/tesla/catkin_ws/src/eddy_beacon_locator/scripts/yolo_network_config/weights/yolov4-tiny-custom_best.weights')
    cfg = os.path.join('/home/jerome/Downloads/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/cfg/yolov4-tiny-custom.cfg')
    weights = os.path.join('/home/jerome/Downloads/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/yolov4-tiny-custom_best.weights')
    

    main()

 
