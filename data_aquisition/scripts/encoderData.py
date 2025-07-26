#!/usr/bin/env python3
import rospy
import rospkg
import csv
import os
import socket
import struct
import threading
from data_aquisition.msg import encoderStream  # Import the custom message

# Set up UDP listener
ip = rospy.get_param('encData_ip')
port = rospy.get_param('encData_port')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((ip, port))  # Port 5500 for first stream

# ROS Publisher
pub_topic = rospy.get_param("encData_topic")
rospy.init_node('encoderSensor', anonymous=True)
pub = rospy.Publisher(pub_topic, encoderStream, queue_size=10)

# CSV setup
rospack = rospkg.RosPack()
package_path = rospack.get_path('data_aquisition')  # Use your actual package name
csv_file_path = os.path.join(package_path, 'record/encoder.csv')
csv_file = open(csv_file_path, mode='a', newline='')
csv_writer = csv.writer(csv_file)
if os.stat(csv_file_path).st_size == 0:
    csv_writer.writerow(['timestamp', 'enc1', 'enc2', 'enc3'])

def encoder_publisher():
    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)
        try:
            # Unpack the data (3 integers and 2 unsigned longs)
            str1, str2, str3, str4, str5 = struct.unpack('3i2Q', data)

            rospy.loginfo(f"Received stream data: {str1, str2, str3, str4, str5} from {addr}")

            # Publish data as a string (could be a custom message instead)
            msg = encoderStream()
            msg.enc_timestamp = str4
            msg.enc1 = str1
            msg.enc2 = str2
            msg.enc3 = str3
            pub.publish(msg)

            # Log message internally (not shown unless log level allows)
            rospy.loginfo(f"Published message: {msg}")

            # Save to CSV
            csv_writer.writerow([msg.enc_timestamp, msg.enc1, msg.enc2, msg.enc3])
            csv_file.flush()

        except struct.error:
            rospy.logwarn(f"Error unpacking data from {addr}")
            continue

def on_shutdown():
    rospy.loginfo("Shutting down encoderSensor node. Closing CSV file.")
    csv_file.close()

def start_listening():
    listener_thread = threading.Thread(target=encoder_publisher)
    listener_thread.daemon = True  # Ensures thread will terminate when ROS shuts down
    listener_thread.start()

if __name__ == '__main__':
    try:
        rospy.on_shutdown(on_shutdown)
        start_listening()  # Start the UDP listener
        rospy.spin()  # Keep the ROS node running
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
