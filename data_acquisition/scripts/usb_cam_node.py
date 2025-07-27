#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import yaml
import os

def load_camera_info(yaml_file, frame_id):
    with open(yaml_file, 'r') as f:
        calib_data = yaml.safe_load(f)

    cam_info = CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = calib_data['camera_matrix']['data']
    cam_info.D = calib_data['distortion_coefficients']['data']
    cam_info.R = calib_data['rectification_matrix']['data']
    cam_info.P = calib_data['projection_matrix']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    cam_info.header.frame_id = frame_id
    return cam_info

def main():
    rospy.init_node('usb_cam_node', anonymous=True)
    
    # ROS parameters
    cam_index = rospy.get_param("cam_idx", 0)
    calib_file = rospy.get_param("~camera_info_url", "")
    frame_id = rospy.get_param("~frame_id", "camera")

    if not os.path.isfile(calib_file):
        rospy.logerr(f"Calibration file not found: {calib_file}")
        return

    cam_topic = rospy.get_param("cam_topic")
    camInfo_topic = rospy.get_param("camInfo_topic")
    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
    info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=10)

    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        rospy.logerr("Cannot open USB camera")
        return

    bridge = CvBridge()
    cam_info_msg = load_camera_info(calib_file, frame_id)

    rospy.loginfo("USB camera node started with calibration.")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture image")
            continue

        stamp = rospy.Time.now()
        img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = frame_id

        cam_info_msg.header.stamp = stamp

        pub.publish(img_msg)
        info_pub.publish(cam_info_msg)

    cap.release()

if __name__ == "__main__":
    main()
