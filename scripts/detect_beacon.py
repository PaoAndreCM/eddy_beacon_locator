#!/usr/bin/env python3
import angle_calculation
import distance_calculation
import beacon
import cv2
import numpy as np

def objDetection(frame, cfg, weights, classes, image_part_number, car_gps):

    listOfBeacons = []
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
                
                #print("[x, y, w, h]:", x, y, w, h)
 
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

                # Adding bounding box to picture slice
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 4)
                
                #Writing picture of detected beacon to harddisk. File name placeholder is confidence value
                cv2.imwrite("./" + str(confidence) +".jpg", frame)
                
                # Calculate the angle of the beacon
                angle = angle_calculation.calculate_angle(image_part_number, center_x, width)
                #Calculate the distance from car to beacon
                distance = distance_calculation.calculate_distance(y, h, height)
                listOfBeacons.append(beacon.Beacon(class_id, [x,y], angle, distance, confidence, car_gps)) # insert beacon type/class later
                
    #Detected plates put inside the image
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    for i in range(len(boxes)):
        if i in indexes.flatten():
            x, y, w, h = boxes[i]
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),3)

    return listOfBeacons
