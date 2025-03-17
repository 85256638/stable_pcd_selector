#!/usr/bin/env python
import rospy
import os
import json
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2, Imu
import ros_numpy
import open3d as o3d
import numpy as np
from datetime import datetime
import csv
import matplotlib.pyplot as plt
import signal
import time

# ===== Editable Parameters =====
FRAME_TIME = 1.0  # Used for data aggregation rate
BASE_OUTPUT_DIR = "/home/taixing/Cam-2-Trigger-Lidar_DATASAVES/pointclouds/"

# Load LIDAR_STATUS from the environment variable.
# If not present, use the default values.
try:
    LIDAR_STATUS = json.loads(os.environ.get("LIDAR_STATUS", '{"farleft_2": true, "left_57": true, "mid_65": true, "right_75": true}'))
except Exception as e:
    print("Error loading LIDAR_STATUS from environment, using defaults:", e)
    LIDAR_STATUS = {
        "farleft_2": True,
        "left_57": True,
        "mid_65": True,
        "right_75": True
    }
# ===============================


class LidarRecorder:
    def __init__(self):
        rospy.init_node('lidar_recorder', anonymous=True)
        self.point_buffer = {lidar_id: [] for lidar_id in LIDAR_STATUS}
        self.imu_data = {lidar_id: [] for lidar_id in LIDAR_STATUS}
        # Use a dictionary to store last save time per lidar channel
        self.last_save_time = {lidar_id: None for lidar_id in LIDAR_STATUS}

        # Session management variables
        self.session_active = False
        self.finalizing = False
        self.session_start = None
        self.session_dir = None

        rospy.Subscriber("/trigger_lidar", Bool, self.trigger_callback)
        for lidar_id in LIDAR_STATUS.keys():
            rospy.Subscriber(f"/livox/lidar_{lidar_id}", PointCloud2, self.pointcloud_callback, callback_args=lidar_id)
            rospy.Subscriber(f"/livox/imu_{lidar_id}", Imu, self.imu_callback, callback_args=lidar_id)

        rospy.Timer(rospy.Duration(FRAME_TIME), self.timer_callback)
        rospy.loginfo("✅ Lidar Recorder Initialized")
        rospy.on_shutdown(self.shutdown_cleanup)
        signal.signal(signal.SIGTERM, self.sigterm_handler)

    def create_session(self):
        self.session_start = rospy.Time.now().to_sec()
        start_str = datetime.fromtimestamp(self.session_start).strftime("%Y-%m-%d_%H-%M-%S")
        self.session_dir = os.path.join(BASE_OUTPUT_DIR, f"session_{start_str}_ongoing")
        os.makedirs(self.session_dir, exist_ok=True)
        for lidar_id in LIDAR_STATUS.keys():
            folder = os.path.join(self.session_dir, f"lidar_{lidar_id}")
            os.makedirs(folder, exist_ok=True)
            self.imu_data[lidar_id] = []
            self.point_buffer[lidar_id] = []
            self.last_save_time[lidar_id] = None  # Reset per-lidar last save time
            rospy.loginfo(f"📁 Created folder for {lidar_id}: {folder}")
        rospy.loginfo(f"📁 New session started: {self.session_dir}")

    def finalize_session(self):
        if self.session_dir:
            end_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            new_dir = self.session_dir.replace("_ongoing", f"_to_{end_str}")
            rospy.loginfo(f"DEBUG: Renaming {self.session_dir} → {new_dir}")
            try:
                os.rename(self.session_dir, new_dir)
                rospy.loginfo(f"🛑 Session finalized: {new_dir}")
            except Exception as e:
                rospy.logerr(f"⚠️ Failed to finalize session: {e}")
            self.session_active = False
            self.session_dir = None
            self.finalizing = False

    def trigger_callback(self, msg):
        if msg.data and not self.session_active:
            rospy.loginfo("🚀 Trigger ON – starting session")
            self.create_session()
            self.session_active = True
        elif not msg.data and self.session_active:
            rospy.loginfo("🛑 Trigger OFF – finalizing session")
            self.session_active = False
            self.finalizing = True
            # Clear buffered data so nothing new is saved.
            for lidar in self.point_buffer:
                self.point_buffer[lidar] = []
            for lidar in self.imu_data:
                self.imu_data[lidar] = []
            time.sleep(0.5)
            self.finalize_session()

    def pointcloud_callback(self, msg, lidar_id):
        if not self.session_active or self.finalizing or not LIDAR_STATUS[lidar_id]:
            return
        pc_np = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        points = np.zeros((pc_np.shape[0], 4), dtype=np.float32)
        points[:, 0] = pc_np['x']
        points[:, 1] = pc_np['y']
        points[:, 2] = pc_np['z']
        points[:, 3] = pc_np['intensity']
        self.point_buffer[lidar_id].append(points)

    def imu_callback(self, msg, lidar_id):
        if not self.session_active or self.finalizing or not LIDAR_STATUS[lidar_id]:
            return
        epoch = rospy.Time.now().to_sec()
        human_time = datetime.fromtimestamp(epoch).strftime("%Y-%m-%d %H:%M:%S.%f")
        # Use per-lidar last save time; if not available, write an empty string.
        timestamp_field = (datetime.fromtimestamp(self.last_save_time[lidar_id]).strftime("%Y-%m-%d %H:%M:%S")
                           if self.last_save_time[lidar_id] else "")
        entry = [
            timestamp_field,
            f"{epoch:.6f}",
            human_time,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ]
        self.imu_data[lidar_id].append(entry)

    def timer_callback(self, event):
        if not self.session_active or self.finalizing:
            return
        current_time = rospy.Time.now().to_sec()
        for lidar_id in LIDAR_STATUS.keys():
            if self.point_buffer[lidar_id]:
                self.save_pcd(current_time, lidar_id)
                self.point_buffer[lidar_id] = []

    def save_pcd(self, current_time, lidar_id):
        if not self.point_buffer[lidar_id]:
            return
        all_points = np.vstack(self.point_buffer[lidar_id])
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(all_points[:, :3])
        intensities = all_points[:, 3]
        max_intensity = intensities.max() if intensities.max() > 0 else 1.0
        normalized_intensities = intensities / max_intensity
        cmap = plt.get_cmap('viridis')
        colors = cmap(normalized_intensities)[:, :3]
        pc.colors = o3d.utility.Vector3dVector(colors)
        folder = os.path.join(self.session_dir, f"lidar_{lidar_id}")
        timestamp_str = datetime.fromtimestamp(current_time).strftime("%Y-%m-%d_%H:%M:%S-%f")
        filename = os.path.join(folder, f"{lidar_id}_{timestamp_str}.pcd")
        o3d.io.write_point_cloud(filename, pc)
        self.last_save_time[lidar_id] = current_time
        rospy.loginfo(f"✅ Saved PCD for {lidar_id}: {filename}")
        self.save_imu(lidar_id)
        
    def save_imu(self, lidar_id):
        if not self.imu_data[lidar_id]:
            return
        folder = os.path.join(self.session_dir, f"lidar_{lidar_id}")
        imu_path = os.path.join(folder, f"imu_data_{lidar_id}.csv")
        file_exists = os.path.exists(imu_path)
        with open(imu_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    "Timestamp of last PCD", "IMU Epoch Timestamp", "IMU Human Readable Time",
                    "Orientation X", "Orientation Y", "Orientation Z", "Orientation W",
                    "Angular Velocity X", "Angular Velocity Y", "Angular Velocity Z",
                    "Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"
                ])
            # Get the current last_save_time, which is the time of the current PCD save
            current_timestamp_field = datetime.fromtimestamp(self.last_save_time[lidar_id]).strftime("%Y-%m-%d %H:%M:%S") if self.last_save_time[lidar_id] else ""
            # Update "Timestamp of last PCD" for each entry to the current last_save_time
            for entry in self.imu_data[lidar_id]:
                entry[0] = current_timestamp_field
            writer.writerows(self.imu_data[lidar_id])
        self.imu_data[lidar_id] = []

    def sigterm_handler(self, signum, frame):
        rospy.loginfo("SIGTERM received – finalizing session if active.")
        if self.session_active:
            self.finalizing = True
            for lidar in self.point_buffer:
                self.point_buffer[lidar] = []
            self.finalize_session()
        exit(0)

    def shutdown_cleanup(self):
        rospy.loginfo("🛑 Shutting down Lidar Recorder – resetting trigger.")
        if self.session_active:
            self.finalizing = True
            for lidar in self.point_buffer:
                self.point_buffer[lidar] = []
            self.finalize_session()
        rospy.loginfo("✅ Shutdown complete.")

    def run(self):
        try:
            rospy.spin()
        finally:
            if self.session_active:
                self.finalize_session()

if __name__ == "__main__":
    recorder = LidarRecorder()
    recorder.run()
