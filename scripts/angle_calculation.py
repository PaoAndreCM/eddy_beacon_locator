#!/usr/bin/env python3

#############################################################################
# Author: Igor Kojanin
# Calculate the angle of a beacon in degrees based on the middle_x of the beacon's bounding box and the width of the picture.
# The full picture is 360 degrees, angle is calculated by:
# middle_x location of bounding box(in pixels) / by the length of a full 360 degree picture * 360 degrees
# August 2023
#############################################################################

COMPASS_PHASE_SHIFT = 0

# Calculates the orientation of the car in degrees where 0 is north, 90 is east and so on
# The input is the Z-axis angle of the car (in radians - 0 is north pi/2 is east and -pi/2 is west).
def car_orientation_angle_calculation(angle_radians):
    angle_degrees = math.degrees(angle_radians) % 360   # %360 to ensure that the resulting degrees fall within the range of 0 to 360 degrees
    if angle_degrees < 0:
        angle_degrees += 360
    if angle_degrees == 360:
        angle_degrees = 0
    return (angle_degrees + COMPASS_PHASE_SHIFT) % 360  # % 360 to ensure that when COMPASS_PHASE_SHIFT > 0 result falls withing the range of 0 to 360 degrees
    
def calculate_angle(picture_num, middle_x, width):
    #print(middle_x)
    #print(picture_num)
    width_of_full_image = width * 5
    num_of_pixels_left_to_center_of_camera = 510 / 1558 * width
    num_of_pixels_right_to_center_of_camera = width - num_of_pixels_left_to_center_of_camera
    if width == 0:
        return 0.0  # To avoid division by zero error, return 0 if width is 0.

    if picture_num == 1:  
        if middle_x > num_of_pixels_left_to_center_of_camera:
            return (middle_x-num_of_pixels_left_to_center_of_camera)/width_of_full_image * 360
        else:
            return (num_of_pixels_right_to_center_of_camera+width*4+middle_x)/width_of_full_image * 360
    elif picture_num == 2:
        return (num_of_pixels_right_to_center_of_camera+width*0+middle_x)/width_of_full_image * 360
    elif picture_num == 3:
        return (num_of_pixels_right_to_center_of_camera+width*1+middle_x)/width_of_full_image * 360
    elif picture_num == 4:
        return (num_of_pixels_right_to_center_of_camera+width*2+middle_x)/width_of_full_image * 360
    elif picture_num == 5:
        return (num_of_pixels_right_to_center_of_camera+width*3+middle_x)/width_of_full_image * 360
