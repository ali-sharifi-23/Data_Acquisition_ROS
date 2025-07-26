#!/usr/bin/env python3
import rospy
import socket
import struct
import threading
import csv
import os
import rospkg
from data_aquisition.msg import stereoStream  # Import the custom message

# CSV setup
rospack = rospkg.RosPack()
package_path = rospack.get_path('data_aquisition')  # Use your actual package name
csv_file_path = os.path.join(package_path, 'record/stereo_cam.csv')
csv_file = open(csv_file_path, mode='a', newline='')
csv_writer = csv.writer(csv_file)
if os.stat(csv_file_path).st_size == 0:
    csv_writer.writerow(['timestamp', 'x', 'y', 'z'])

# Set up UDP listener
ip = rospy.get_param('stereoData_ip')
port = rospy.get_param('stereoData_port')
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind((ip, port))

# ROS Publisher
pub_topic = rospy.get_param('stereoData_topic')
rospy.init_node('stereoSensor', anonymous=True)
pub = rospy.Publisher(pub_topic, stereoStream, queue_size=10)

def stereo_publisher():
    while not rospy.is_shutdown():
        data, addr = udp_socket.recvfrom(4096)
        try:
            # Unpack the data (3 integers and 2 unsigned longs)
            unpacked_data = struct.unpack(f'd3d2d', data)

            timestamp = unpacked_data[0]
            x, y, z = unpacked_data[1:4]

            # Publish data as a string (could be a custom message instead)
            msg = stereoStream()
            msg.stereo_timestamp = int(timestamp*1e3)
            msg.x_stereo = x
            msg.y_stereo = y
            msg.z_stereo = z
            pub.publish(msg)

            # Log message internally (not shown unless log level allows)
            rospy.loginfo(f"Published message: {msg}")

            # Save to CSV
            csv_writer.writerow([msg.stereo_timestamp, msg.x_stereo, msg.y_stereo, msg.z_stereo])
            csv_file.flush()

        except struct.error:
            rospy.logwarn(f"Error unpacking data from {addr}")
            continue

def start_listening():
    listener_thread = threading.Thread(target=stereo_publisher)
    listener_thread.daemon = True  # Ensures thread will terminate when ROS shuts down
    listener_thread.start()

def on_shutdown():
    rospy.loginfo("Shutting down monoSensor node. Closing CSV file.")
    csv_file.close()

if __name__ == '__main__':
    try:
        rospy.on_shutdown(on_shutdown)
        start_listening()  # Start the UDP listener
        rospy.spin()  # Keep the ROS node running
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
