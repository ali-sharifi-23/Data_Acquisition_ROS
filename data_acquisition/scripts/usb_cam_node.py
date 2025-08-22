#!/usr/bin/env python3
import rospy
import cv2
import yaml
import re
import os
import subprocess
import threading
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class USBCameraPublisher:
    def __init__(self):
        rospy.init_node('usb_cam_node', anonymous=True)

        # Parameters
        self.usb_port = rospy.get_param("usb_port", "1-1")
        self.calib_file = rospy.get_param("~camera_info_url", "")
        self.frame_id = rospy.get_param("~frame_id", "camera")
        self.fps = rospy.get_param("~fps", 60)
        self.width = rospy.get_param("~width", 640)
        self.height = rospy.get_param("~height", 480)

        self.cam_index = self._find_camera_index(self.usb_port)
        if self.cam_index is None:
            rospy.signal_shutdown("Camera not found.")
            return

        if not os.path.isfile(self.calib_file):
            rospy.signal_shutdown(f"Calibration file not found: {self.calib_file}")
            return

        self.bridge = CvBridge()
        self.cam_info_msg = self._load_camera_info(self.calib_file, self.frame_id)

        # Publishers (queue_size=1 to drop old messages)
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
        self.info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=1)

        # OpenCV capture setup
        self.cap = cv2.VideoCapture(self.cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Drop buffered frames if supported

        if not self.cap.isOpened():
            rospy.signal_shutdown("Cannot open USB camera.")
            return

        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # Start camera thread
        threading.Thread(target=self._camera_reader, daemon=True).start()

    def _find_camera_index(self, usb_port):
        """Find /dev/video index for given USB port"""
        proc = subprocess.Popen(['v4l2-ctl', '--list-devices'],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = proc.communicate()
        if proc.returncode != 0:
            rospy.logerr(f"Error running v4l2-ctl: {stderr.decode()}")
            return None

        info = stdout.decode("utf-8")
        devices = [block.strip() for block in info.split('\n\n') if block.strip()]

        for dev_block in devices:
            if usb_port in dev_block:
                video_nodes = re.findall(r'/dev/video(\d+)', dev_block)
                if video_nodes:
                    return int(video_nodes[0])
        rospy.logerr(f"Camera with USB port '{usb_port}' not found.")
        return None

    def _load_camera_info(self, yaml_file, frame_id):
        """Load camera calibration file"""
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

    def _camera_reader(self):
        """Continuously read frames and store only the latest one"""
        while not rospy.is_shutdown():
            if not self.cap.grab():  # Grab newest frame (skip decode)
                continue
            ret, frame = self.cap.retrieve()
            if not ret:
                continue
            with self.frame_lock:
                self.latest_frame = frame

    def spin(self):
        """Main publishing loop"""
        rate = rospy.Rate(self.fps)
        rospy.loginfo("USB camera node started with real-time publishing.")

        while not rospy.is_shutdown():
            frame = None
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()

            if frame is not None:
                stamp = rospy.Time.now()
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = self.frame_id

                self.cam_info_msg.header.stamp = stamp

                self.image_pub.publish(img_msg)
                self.info_pub.publish(self.cam_info_msg)

            rate.sleep()

        self.cap.release()


if __name__ == "__main__":
    node = USBCameraPublisher()
    if node.cap.isOpened():
        node.spin()
