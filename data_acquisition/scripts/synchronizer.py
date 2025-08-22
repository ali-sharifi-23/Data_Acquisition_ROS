#!/usr/bin/env python3
import numpy as np
import rospy
import os
import csv
import rospkg
from collections import deque
from threading import Lock, Thread
from data_acquisition.msg import encoderStream, monoStream, stereoStream, \
                                 SyncedMonoStream, SyncedStereoStream

class DualSynchronizer:
    def __init__(self):
        rospy.init_node("dual_sync_node")

        self.enc_buffer = deque()
        self.mono_buffer = deque()
        self.stereo_buffer = deque()
        self.lock = Lock()
        self.slop_ns = rospy.get_param("sync_tolerance_ns", 8000000)  # 800us default
        self.max_wait_time_ns = 5 * 1e9 # 5s default

        # Subscribers
        # Using direct subscription is cleaner than managing threads
        rospy.Subscriber(rospy.get_param("encData_topic"), encoderStream, self.encoder_callback)
        rospy.Subscriber(rospy.get_param("monoData_topic"), monoStream, self.mono_callback)
        rospy.Subscriber(rospy.get_param("stereoData_topic"), stereoStream, self.stereo_callback)

        # Publishers
        self.mono_pub = rospy.Publisher("/synced_mono_msg", SyncedMonoStream, queue_size=30)
        self.stereo_pub = rospy.Publisher("/synced_stereo_msg", SyncedStereoStream, queue_size=30)

        # CSV Logging setup
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('data_acquisition')
        data_path = os.path.join(pkg_path, 'record')
        os.makedirs(data_path, exist_ok=True)

        self.mono_csv_file = open(os.path.join(data_path, 'synced_mono.csv'), mode='w', newline='')
        self.mono_writer = csv.writer(self.mono_csv_file)
        self.mono_writer.writerow(['timestamp_ns', 'enc1', 'enc2', 'enc3', 'x', 'y'])

        self.stereo_csv_file = open(os.path.join(data_path, 'synced_stereo.csv'), mode='w', newline='')
        self.stereo_writer = csv.writer(self.stereo_csv_file)
        self.stereo_writer.writerow(['timestamp_ns', 'enc1', 'enc2', 'enc3', 'x', 'y', 'z'])

        rospy.on_shutdown(self.close_files)

        # Start processing thread
        self.processing_thread = Thread(target=self.process_buffered_data)
        self.processing_thread.start()
        rospy.loginfo("Dual Synchronizer node started.")

    def encoder_callback(self, msg):
        with self.lock:
            self.enc_buffer.append(msg)

    def mono_callback(self, mono_msg):
        with self.lock:
            self.mono_buffer.append(mono_msg)

    def stereo_callback(self, stereo_msg):
        with self.lock:
            self.stereo_buffer.append(stereo_msg)

    def process_buffered_data(self):
        """
        This method continuously processes the buffered camera data, finds the best
        matching encoder data, and handles delays, timeouts, and buffer pruning.
        """
        rate = rospy.Rate(100)  # Process at 100Hz
        while not rospy.is_shutdown():
            with self.lock:
                now_ns = rospy.Time.now().to_nsec()

                # --- 1. Prune the Encoder Buffer ---
                # To prevent the encoder buffer from growing forever, we discard old
                # encoder messages that are no longer possibly needed.
                oldest_cam_ts = float('inf')
                if self.mono_buffer:
                    oldest_cam_ts = min(oldest_cam_ts, int(self.mono_buffer[0].mono_timestamp))
                if self.stereo_buffer:
                    oldest_cam_ts = min(oldest_cam_ts, int(self.stereo_buffer[0].stereo_timestamp))
                
                if oldest_cam_ts != float('inf'):
                    # Discard encoder messages that are much older than the oldest camera message
                    # We subtract the wait time to ensure we don't discard a message that a waiting camera message might need
                    prune_ts = oldest_cam_ts - self.max_wait_time_ns
                    while self.enc_buffer and int(self.enc_buffer[0].enc_timestamp) < prune_ts:
                        self.enc_buffer.popleft()

                # --- 2. Process Monocular Messages ---
                while self.mono_buffer:
                    # Peek at the message, don't remove it from the buffer yet.
                    mono_msg = self.mono_buffer[0]
                    mono_ts = int(mono_msg.mono_timestamp)

                    # Check for timeout: If the message has been waiting too long, discard it.
                    # This prevents the system from stalling if an encoder message is lost.
                    if now_ns - mono_ts > self.max_wait_time_ns:
                        rospy.logwarn(f"[MONO] Discarding old message at {mono_ts} (waited too long).")
                        self.mono_buffer.popleft()
                        continue # Move to the next message in the buffer

                    # Try to find a matching encoder message.
                    matched_enc = self._find_best_match(self.enc_buffer, mono_ts)
                    
                    if matched_enc:
                        # Match found! Publish it and remove the mono message from the buffer.
                        self._publish_synced_mono(mono_msg, matched_enc)
                        self.mono_buffer.popleft()
                    else:
                        # No suitable encoder message has arrived yet. Stop processing
                        # mono messages for this cycle and wait for more data.
                        break

                # --- 3. Process Stereo Messages (Identical logic to mono) ---
                while self.stereo_buffer:
                    stereo_msg = self.stereo_buffer[0]
                    stereo_ts = int(stereo_msg.stereo_timestamp)

                    if now_ns - stereo_ts > self.max_wait_time_ns:
                        rospy.logwarn(f"[STEREO] Discarding old message at {stereo_ts} (waited too long).")
                        self.stereo_buffer.popleft()
                        continue

                    matched_enc = self._find_best_match(self.enc_buffer, stereo_ts)

                    if matched_enc:
                        self._publish_synced_stereo(stereo_msg, matched_enc)
                        self.stereo_buffer.popleft()
                    else:
                        break
            
            rate.sleep()

    def _publish_synced_mono(self, mono_msg, enc_msg):
        msg = SyncedMonoStream()
        # Use the encoder's timestamp as the unifying timestamp for the synced message
        msg.timestamp = enc_msg.enc_timestamp
        msg.enc1 = enc_msg.enc1
        msg.enc2 = enc_msg.enc2
        msg.enc3 = enc_msg.enc3
        msg.x = mono_msg.x_mono
        msg.y = mono_msg.y_mono
        self.mono_pub.publish(msg)
        
        # FIX: Write timestamp as an integer, not a string with quotes
        self.mono_writer.writerow([
            int(msg.timestamp),
            msg.enc1, msg.enc2, msg.enc3,
            msg.x, msg.y
        ])
        self.mono_csv_file.flush()

    def _publish_synced_stereo(self, stereo_msg, enc_msg):
        msg = SyncedStereoStream()
        msg.timestamp = enc_msg.enc_timestamp
        msg.enc1 = enc_msg.enc1
        msg.enc2 = enc_msg.enc2
        msg.enc3 = enc_msg.enc3
        msg.x = stereo_msg.x_stereo
        msg.y = stereo_msg.y_stereo
        msg.z = stereo_msg.z_stereo
        self.stereo_pub.publish(msg)

        # FIX: Write timestamp as an integer, not a string with quotes
        self.stereo_writer.writerow([
            int(msg.timestamp),
            msg.enc1, msg.enc2, msg.enc3,
            msg.x, msg.y, msg.z
        ])
        self.stereo_csv_file.flush()

    def _find_best_match(self, buffer, target_ts):
        if not buffer:
            return None
        
        timestamps = np.array([int(msg.enc_timestamp) for msg in buffer])
        diffs = np.abs(timestamps - target_ts)
        closest_index = np.argmin(diffs)
        
        # Return the match only if it's within the acceptable tolerance
        if diffs[closest_index] <= self.slop_ns:
            return buffer[closest_index]
            
        return None

    def close_files(self):
        rospy.loginfo("Shutting down dual_sync_node. Closing CSV files.")
        if self.mono_csv_file:
            self.mono_csv_file.close()
        if self.stereo_csv_file:
            self.stereo_csv_file.close()

if __name__ == "__main__":
    try:
        DualSynchronizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
