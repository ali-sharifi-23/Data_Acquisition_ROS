#!/usr/bin/env python3
import os
import csv
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from data_acquisition.msg import monoStream

class MonoSensorNode:
    def __init__(self):
        # ROS and CSV setup
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('data_acquisition')
        self.record_dir = os.path.join(package_path, 'record')
        os.makedirs(self.record_dir, exist_ok=True)

        # Pose CSV setup
        self.csv_pose_path = os.path.join(self.record_dir, 'mono_cam.csv')
        self.csv_pose_file = open(self.csv_pose_path, mode='a', newline='')
        self.csv_pose_writer = csv.writer(self.csv_pose_file)
        if os.stat(self.csv_pose_path).st_size == 0:
            self.csv_pose_writer.writerow(['timestamp', 'x', 'y'])

        # ROS Params
        self.pub_topic = rospy.get_param("monoData_topic")
        self.pose_topic = rospy.get_param("svoPose_topic")

        # Publisher setup
        self.pub = rospy.Publisher(self.pub_topic, monoStream, queue_size=10)

        # Initialize the node
        rospy.init_node("monoSensor", anonymous=False)

        # Set shutdown behavior
        rospy.on_shutdown(self.on_shutdown)

        # Subscriber setup
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=10)

    def pose_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        stamp_ns = msg.header.stamp.secs * 10**9 + msg.header.stamp.nsecs

        rospy.loginfo("t=%d pos=(%.3f, %.3f) -- publishing monoStream", stamp_ns, p.x, p.y)

        # Create and publish the custom message
        custom_msg = monoStream()
        custom_msg.mono_timestamp = int(stamp_ns)
        custom_msg.x_mono = p.x
        custom_msg.y_mono = p.y
        self.pub.publish(custom_msg)

        # Write data to CSV
        self.csv_pose_writer.writerow([custom_msg.mono_timestamp, custom_msg.x_mono, custom_msg.y_mono])
        self.csv_pose_file.flush()

    def on_shutdown(self):
        rospy.loginfo("Shutting down monoSensor node. Closing files.")
        self.csv_pose_file.close()

if __name__ == "__main__":
    try:
        # Instantiate and run the node
        node = MonoSensorNode()
        rospy.spin()  # Keeps the main thread running and responsive
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
