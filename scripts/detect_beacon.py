from angle_calculation import calculate_angle
from distance_calculation import calculate_distance
from beacon import Beacon
import cv2
import numpy as np

def objDetection(frame, cfg, weights, classes, image_part_number):

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
                # Calculate the angle of the beacon
                angle = calculate_angle(image_part_number, center_x, width)
                #Calculate the distance from car to beacon
                distance = calculate_distance(y, h, height)
                listOfBeacons.append(Beacon("Dummy", [x,y], angle, distance, image_part_number)) # insert beacon type/class later

                


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

    return listOfBeacons
