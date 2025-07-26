#!/usr/bin/env python3

import rospy
import os
import csv
import rospkg
from collections import deque
from threading import Lock
from data_aquisition.msg import encoderStream, monoStream, stereoStream, \
                                 SyncedMonoStream, SyncedStereoStream

class DualSynchronizer:
    def __init__(self):
        rospy.init_node("dual_sync_node")

        self.enc_buffer = deque()
        self.lock = Lock()
        self.slop_ns = rospy.get_param("sync_tolerance", 500000)

        # Subscribers
        enc_topic = rospy.get_param("encData_topic")
        mono_topic = rospy.get_param("monoData_topic")
        stereo_topic = rospy.get_param("stereoData_topic")
        rospy.Subscriber(enc_topic, encoderStream, self.encoder_callback)
        rospy.Subscriber(mono_topic, monoStream, self.mono_callback)
        rospy.Subscriber(stereo_topic, stereoStream, self.stereo_callback)

        # Publishers
        self.mono_pub = rospy.Publisher("/synced_mono_msg", SyncedMonoStream, queue_size=10)
        self.stereo_pub = rospy.Publisher("/synced_stereo_msg", SyncedStereoStream, queue_size=10)

        # CSV Logging setup
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('data_aquisition')  # Change if package name differs
        data_path = os.path.join(pkg_path, 'record')
        os.makedirs(data_path, exist_ok=True)

        # Mono CSV
        self.mono_csv_file = open(os.path.join(data_path, 'synced_mono.csv'), mode='a', newline='')
        self.mono_writer = csv.writer(self.mono_csv_file)
        if os.stat(self.mono_csv_file.name).st_size == 0:
            self.mono_writer.writerow(['timestamp_ns', 'enc1', 'enc2', 'enc3', 'x', 'y'])

        # Stereo CSV
        self.stereo_csv_file = open(os.path.join(data_path, 'synced_stereo.csv'), mode='a', newline='')
        self.stereo_writer = csv.writer(self.stereo_csv_file)
        if os.stat(self.stereo_csv_file.name).st_size == 0:
            self.stereo_writer.writerow(['timestamp_ns', 'enc1', 'enc2', 'enc3', 'x', 'y', 'z'])

        # Shutdown hook
        rospy.on_shutdown(self.close_files)

    def encoder_callback(self, msg):
        with self.lock:
            self.enc_buffer.append(msg)

    def mono_callback(self, mono_msg):
        with self.lock:
            mono_ts = int(mono_msg.mono_timestamp)
            matched_enc = self._find_best_match(self.enc_buffer, mono_ts, 'enc_timestamp')
            if not matched_enc:
                rospy.logwarn(f"[MONO] No encoder match for mono {mono_ts}")
                return

            msg = SyncedMonoStream()
            msg.timestamp = matched_enc.enc_timestamp
            msg.enc1 = matched_enc.enc1
            msg.enc2 = matched_enc.enc2
            msg.enc3 = matched_enc.enc3
            msg.x = mono_msg.x_mono
            msg.y = mono_msg.y_mono
            self.mono_pub.publish(msg)
            rospy.loginfo(f"[MONO] Published matched encoder+mono at {mono_ts}")

            # Write to CSV as string to preserve full timestamp
            self.mono_writer.writerow([
                f"'{int(msg.timestamp)}'",
                msg.enc1,
                msg.enc2,
                msg.enc3,
                msg.x,
                msg.y
            ])
            self.mono_csv_file.flush()

            self._cleanup_old(self.enc_buffer, mono_ts)

    def stereo_callback(self, stereo_msg):
        with self.lock:
            stereo_ts = int(stereo_msg.stereo_timestamp)
            matched_enc = self._find_best_match(self.enc_buffer, stereo_ts, 'enc_timestamp')
            if not matched_enc:
                rospy.logwarn(f"[STEREO] No encoder match for stereo {stereo_ts}")
                return

            msg = SyncedStereoStream()
            msg.timestamp = matched_enc.enc_timestamp
            msg.enc1 = matched_enc.enc1
            msg.enc2 = matched_enc.enc2
            msg.enc3 = matched_enc.enc3
            msg.x = stereo_msg.x_stereo
            msg.y = stereo_msg.y_stereo
            msg.z = stereo_msg.z_stereo
            self.stereo_pub.publish(msg)
            rospy.loginfo(f"[STEREO] Published matched encoder+stereo at {stereo_ts}")

            # Write to CSV as string to preserve full timestamp
            self.stereo_writer.writerow([
                f"'{int(msg.timestamp)}'",
                msg.enc1,
                msg.enc2,
                msg.enc3,
                msg.x,
                msg.y,
                msg.z
            ])
            self.stereo_csv_file.flush()

            self._cleanup_old(self.enc_buffer, stereo_ts)

    def _find_best_match(self, buffer, target_ts, field):
        for msg in buffer:
            ts = int(getattr(msg, field))
            if abs(target_ts - ts) <= self.slop_ns:
                return msg
        return None

    def _cleanup_old(self, buffer, cutoff_ts):
        while buffer and int(getattr(buffer[0], buffer[0]._slot_types[0])) <= cutoff_ts:
            buffer.popleft()

    def close_files(self):
        rospy.loginfo("Shutting down dual_sync_node. Closing CSV files.")
        self.mono_csv_file.close()
        self.stereo_csv_file.close()

if __name__ == "__main__":
    try:
        DualSynchronizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
