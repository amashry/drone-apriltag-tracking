#!/usr/bin/env python3

import rospy
import cv2
import numpy
import yaml

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

def callback(data, camera_info_msg):
    camera_info_msg.header.stamp=data.header.stamp
    #print(data.header.stamp)
    #print(camera_info_msg.header.stamp)
    pub1.publish(camera_info_msg)
    #print("Actually Published the info!!!")
    
def yaml_to_CameraInfo(yaml_fname):
    # Load data from file
    #fs = cv2.FileStorage(yaml_fname, cv2.FILE_STORAGE_READ)
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
    # Get fname from command line (cmd line input required)
    import argparse
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("filename", help="Path to yaml file containing " +\
                                             "camera calibration data")
    args = arg_parser.parse_args()
    filename = args.filename

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(filename)

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    pub1 = rospy.Publisher("/iris/usb_cam/camera_info", CameraInfo, queue_size=10)
    sub1 = rospy.Subscriber("/iris/usb_cam/image_raw", Image, callback, camera_info_msg)
    #rate = rospy.Rate(200)
    
    # Spin
    while not rospy.is_shutdown():
       try:
         rospy.spin()
       except rospy.ROSException as e:
         rospy.logerr("Error in subscriber: %s", str(e))
         

