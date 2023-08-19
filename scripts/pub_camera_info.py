#!/usr/bin/env python3

"""
This script reads a calibration file in yaml/yml format and publishes camera_info messages for a camera,  
the calibration file is passed as a command line argument 

Usage:
python3 pub_camera_info.py <calibration_file>

Arguments:
calibration_file: Path to yaml file containing camera calibration data.

Notes:
you can use it with another camera_name that has the same calibration file (yaml) structure and formatting 
you will just need to change the camera_name where the script Initializes publisher node below. 
"""

import rospy
import yaml
import argparse

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

def callback(data, camera_info_msg):
    camera_info_msg.header.stamp=data.header.stamp
    pub1.publish(camera_info_msg)
    
def yaml_to_CameraInfo(yaml_fname):
    # Load data from file
    with open(yaml_fname, 'r') as f:
       calib_data = yaml.safe_load(f)
    # Parse
    camera_info_msg = CameraInfo()
    
    camera_info_msg.width = calib_data['image_width']
    camera_info_msg.height = calib_data['image_height']
    
    camera_matrix = calib_data['camera_matrix']['data']
    camera_info_msg.K = [float(x) for x in camera_matrix]
    
    distortion_coeffs = calib_data['distortion_coefficients']['data']
    camera_info_msg.D = [float(x) for x in distortion_coeffs]
    
    rectification_matrix = calib_data['rectification_matrix']['data']
    camera_info_msg.R = [float(x) for x in rectification_matrix]

    projection_matrix = calib_data['projection_matrix']['data']
    camera_info_msg.P = [float(x) for x in projection_matrix]

    camera_info_msg.distortion_model = calib_data['distortion_model']
    
    print(camera_info_msg.K)
    print(camera_info_msg.D)
    return camera_info_msg

if __name__ == "__main__":
    
    # Get fname from command line
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("filename")
    args = arg_parser.parse_args()
    filename = args.filename

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(filename)

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    pub1 = rospy.Publisher("/iris/usb_cam/camera_info", CameraInfo, queue_size=10)
    sub1 = rospy.Subscriber("/iris/usb_cam/image_raw", Image, callback, camera_info_msg)
    
    # Spin
    while not rospy.is_shutdown():
       try:
         rospy.spin()
       except rospy.ROSException as e:
         rospy.logerr("Error in subscriber: %s", str(e))
         

