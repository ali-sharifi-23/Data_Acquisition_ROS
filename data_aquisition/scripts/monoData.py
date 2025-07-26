#!/usr/bin/env python3
import os
import csv
import rospy
import rospkg
import cv2
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from data_aquisition.msg import monoStream

# ROS and CSV setup
rospack = rospkg.RosPack()
package_path = rospack.get_path('data_aquisition')
record_dir = os.path.join(package_path, 'record')
os.makedirs(record_dir, exist_ok=True)

# Pose CSV
csv_pose_path = os.path.join(record_dir, 'mono_cam.csv')
csv_pose_file = open(csv_pose_path, mode='a', newline='')
csv_pose_writer = csv.writer(csv_pose_file)
if os.stat(csv_pose_path).st_size == 0:
    csv_pose_writer.writerow(['timestamp', 'x', 'y'])

# Image timestamp CSV
csv_img_path = os.path.join(record_dir, 'mono_image_timestamps.csv')
csv_img_file = open(csv_img_path, mode='a', newline='')
csv_img_writer = csv.writer(csv_img_file)
if os.stat(csv_img_path).st_size == 0:
    csv_img_writer.writerow(['image_timestamp_ns'])

# ROS Params
pub_topic = rospy.get_param("monoData_topic")
pose_topic = rospy.get_param("svoPose_topic")
image_topic = rospy.get_param("image_topic", "/camera/image_raw")

rospy.init_node("monoSensor", anonymous=False)
pub = rospy.Publisher(pub_topic, monoStream, queue_size=10)

# CV and video
bridge = CvBridge()
video_path = os.path.join(record_dir, 'mono_video.avi')
fourcc = cv2.VideoWriter_fourcc(*'MJPG')  # or 'avc1' or 'X264'
fps = 10.0
frame_size = (1280, 720)
video_writer = cv2.VideoWriter(video_path, fourcc, fps, frame_size)
if not video_writer.isOpened():
    rospy.logerr("Failed to open video writer. Check codec or path.")
else:
    rospy.loginfo(f"Video recording to {video_path}")

def pose_callback(msg: PoseStamped) -> None:
    p = msg.pose.position
    stamp_ns = msg.header.stamp.secs * 10**9 + msg.header.stamp.nsecs

    rospy.loginfo("t=%d pos=(%.3f, %.3f) -- publishing monoStream", stamp_ns, p.x, p.y)

    custom_msg = monoStream()
    custom_msg.mono_timestamp = int(stamp_ns)
    custom_msg.x_mono = p.x
    custom_msg.y_mono = p.y
    pub.publish(custom_msg)

    csv_pose_writer.writerow([custom_msg.mono_timestamp, custom_msg.x_mono, custom_msg.y_mono])
    csv_pose_file.flush()

def image_callback(msg: Image) -> None:
    try:
        image_stamp_ns = msg.header.stamp.secs * 10**9 + msg.header.stamp.nsecs
        csv_img_writer.writerow([int(image_stamp_ns)])
        csv_img_file.flush()

        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        resized = cv2.resize(cv_image, frame_size)
        video_writer.write(resized)
    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

def on_shutdown():
    rospy.loginfo("Shutting down monoSensor node. Closing files.")
    csv_pose_file.close()
    csv_img_file.close()
    video_writer.release()

if __name__ == "__main__":
    try:
        rospy.on_shutdown(on_shutdown)
        rospy.Subscriber(pose_topic, PoseStamped, pose_callback, queue_size=10)
        rospy.Subscriber(image_topic, Image, image_callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
