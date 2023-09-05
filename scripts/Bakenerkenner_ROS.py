#!/usr/bin/env python3

import rospy
import rosbag
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image

# import for bagfile-to-cv2-transport and other way round (Image format change)
from cv_bridge import CvBridge
bridge = CvBridge()

def detect(frame,cfg,weights,classes):
    # Load Model
    # Give the configuration and weight files for the model and load the network.
    net = cv2.dnn.readNet(weights,cfg)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i-1] for i in net.getUnconnectedOutLayers()]

    # image dimensions 
    height, width, channels = frame.shape

    # Create the pipeline
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

    net.setInput(blob)
    outs = net.forward(output_layers)

    cropped_plate = []
    crop_rect = []
    class_ids = []
    confidences = []
    boxes = []
 
    # Loop to predict plates
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                
                print("[x, y, w, h]:", x, y, w, h)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    #Detected plates put inside the image
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    for i in range(len(boxes)):
        if i in indexes.flatten():
            x, y, w, h = boxes[i]
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),3)

        # if ShowNumbersOfPlate:
        #     crop_rect.append([x,y,w,h])
        #     #Cropped detected rectangle
        #     crop = frame[y:y+h, x:x+w]
        #     cropped_plate.append(crop)
        #     no_of_detected_plates = len(crop_rect)
        #     print(no_of_detected_plates)
        #     confidences = confidences[-no_of_detected_plates:]
        #     print(confidence)
    
    return frame

def Rectified_Image_callback(data):

    # Stereokamera Image abonieren
    frame = data
    # Image von Rosformat zu OpenCV-format konvertieren
    frame = bridge.imgmsg_to_cv2(frame, desired_encoding="bgr8")

    img_90 = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    # *****************Split images*****************************
    # Get the dimensions of the combined image
    height, width = img_90.shape[:2]

    # Calculate the width of each individual picture
    picture_width = width // 6
    w_to_cut = 245

    # Crop the image into 6 separate regions
    pictures = [img_90[:, i * picture_width+w_to_cut: (i + 1) * picture_width-w_to_cut] for i in range(6)]
    pictures[3] = img_90[:, 3 * picture_width + w_to_cut + 20: (3 + 1) * picture_width - w_to_cut + 20]

    # Rearrange the cropped regions in the correct order
    # rearranged_pictures = [pictures[5], pictures[4], pictures[3], pictures[2], pictures[1]]
    #for i in range(5):
    #    frame_copy = detect(i,cfg,weights,classes)

    rearranged_pictures = [pictures[5], pictures[4], pictures[3], pictures[2], pictures[1]]
    for picture in rearranged_pictures:
        frame_copy = detect(picture, cfg, weights, classes)
    # **********************************************************


    #frame_copy = detect(img_90,cfg,weights,classes) #original

    # frame zurück zu ROS-Image konvertieren
    frame_copy = bridge.cv2_to_imgmsg(frame_copy, encoding="bgr8")

    # Frame in ROS publishen
    pub.publish(frame_copy)


if __name__ == '__main__':
    ## Load path to model config ##
    dirname = os.path.dirname(__file__)

    #************ADD YOUR PATH HERE**********************
    cfg = os.path.join('/home/sudi/catkin_ws_beacons/src/darknet_ros_pck/darknet_ros/yolo_network_config/cfg/yolov4-tiny-custom.cfg')
    weights = os.path.join('/home/sudi/catkin_ws_beacons/src/darknet_ros_pck/darknet_ros/yolo_network_config/weights/yolov4-tiny-custom_best.weights')
    #*****************************************************

    classes = ['bake', 'bake2']
    #Node initialisieren
    rospy.init_node("Bakenerkenner")
    # Publisher & Subscriber erstellen
    pub = rospy.Publisher('/Bakenerkenner', Image, queue_size=1)
    sub = rospy.Subscriber("/camera/image_raw", Image, callback=Rectified_Image_callback)
    #Console info
    rospy.loginfo("Publisher & Subscriber has been started.")
    rospy.loginfo("Node has been started")
    rospy.spin()
    # Der code befindet sich in der Callback-funktion des Subscribers.
    # Immer wenn etwas empfangen wird, wird es verarbeitet und gepublished.
