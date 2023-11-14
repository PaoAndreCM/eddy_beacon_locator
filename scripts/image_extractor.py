#!/usr/bin/env python3
import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# Initialize the CvBridge
bridge = CvBridge()

def save_image_from_rosbag(bag_file, topic, output_folder):
    rospy.init_node("image_extractor")

    with rosbag.Bag(bag_file, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[topic]):
            try:
                # Convert the ROS Image to an OpenCV image
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

                # Get the timestamp to use as the image filename
                timestamp = t.to_nsec()

                # Create the output folder if it doesn't exist
                os.makedirs(output_folder, exist_ok=True)

                # Save the image as .jpg in the output folder
                image_filename = os.path.join(output_folder, f"{timestamp}.jpg")
                cv2.imwrite(image_filename, cv_image)

                print(f"Saved image: {image_filename}")

            except Exception as e:
                print(f"Error while processing image: {e}")

def process_rosbag_folder(folder_path, topic, output_folder):
    for filename in os.listdir(folder_path):
        if filename.endswith(".bag"):
            bag_file = os.path.join(folder_path, filename)
            save_image_from_rosbag(bag_file, topic, output_folder)

if __name__ == '__main__':
    # Replace these variables with your desired values
    folder_path = "/home/sudi/RosBags"  # Folder containing the rosbag files
    topic = "/camera/image_raw"  # The topic containing the images
    output_folder = "/home/sudi/extracted_images"  # Folder to save the extracted .jpg images

    process_rosbag_folder(folder_path, topic, output_folder)

