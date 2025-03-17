#!/usr/bin/env python3
import rospy
import cv2
import os
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
from datetime import datetime
import shutil

# ==============================
# 🔧 CONFIGURATION PARAMETERS 🔧
# ==============================
YOLO_MODEL_PATH = os.environ.get("YOLO_MODEL_PATH", "/home/taixing/yolov8n.onnx")
SAVE_RAW_IMAGES = os.environ.get("SAVE_RAW_IMAGES", "True").lower() == "true"
SAVE_PROCESSED_IMAGES = os.environ.get("SAVE_PROCESSED_IMAGES", "False").lower() == "true"
CONFIDENCE_THRESHOLD = float(os.environ.get("CONFIDENCE_THRESHOLD", "0.25"))
PERSON_CLASS_ID = int(os.environ.get("PERSON_CLASS_ID", "0"))
SIMULATED_PERSON_DETECTION = os.environ.get("SIMULATED_PERSON_DETECTION", "True").lower() == "true"
SIMULATED_DETECTION_OPTION = os.environ.get("SIMULATED_DETECTION_OPTION", "always")
SIMULATED_DETECTION_ON_DURATION = float(os.environ.get("SIMULATED_DETECTION_ON_DURATION", "0.1"))  # minutes
SIMULATED_DETECTION_CYCLE = float(os.environ.get("SIMULATED_DETECTION_CYCLE", "1"))              # minutes
LATCH_TRIGGER_TOPIC = os.environ.get("LATCHING_BEHAVIOR", "True").lower() == "true"
IMAGE_SESSION_BASE_DIR = os.environ.get("IMAGE_SESSION_BASE_DIR", "/home/taixing/Cam-2-Trigger-Lidar_DATASAVES/images")
IMAGE_SAVE_INTERVAL = float(os.environ.get("IMAGE_SAVE_INTERVAL", "5"))  # Interval for saving images in seconds

class PersonDetector:
    def __init__(self):
        rospy.init_node("person_detector", anonymous=True)
        rospy.loginfo("✅ Person Detector Initialized")
        self.model = YOLO(YOLO_MODEL_PATH, task="detect")
        self.bridge = CvBridge()

        # Session management variables
        self.session_active = False
        self.session_start = None
        self.session_dir = None
        self.last_save_time = 0  # Initialize the last save time

        # ROS Topics
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.camera_callback)
        self.image_pub = rospy.Publisher("/person_detector/output", Image, queue_size=1)
        self.trigger_pub = rospy.Publisher("/trigger_lidar", Bool, queue_size=1, latch=LATCH_TRIGGER_TOPIC)
        self.lidar_active = False

        # Immediately publish False
        self.trigger_pub.publish(False)
        rospy.loginfo("🔴 Initial State: /trigger_lidar = False")
        rospy.on_shutdown(self.shutdown_cleanup)

    def create_session(self):
        self.session_start = time.time()
        start_str = datetime.fromtimestamp(self.session_start).strftime("%Y-%m-%d_%H-%M-%S")
        self.session_dir = os.path.join(IMAGE_SESSION_BASE_DIR, f"session_{start_str}_ongoing")
        os.makedirs(self.session_dir, exist_ok=True)
        self.raw_dir = os.path.join(self.session_dir, "raw")
        self.proc_dir = os.path.join(self.session_dir, "processed")
        os.makedirs(self.raw_dir, exist_ok=True)
        os.makedirs(self.proc_dir, exist_ok=True)
        self.last_save_time = time.time()  # Reset save time at session start
        rospy.loginfo(f"📁 Session started: {self.session_dir}")

    def finalize_session(self):
        if self.session_active and self.session_dir:
            rospy.loginfo(f"DEBUG: Entered finalize_session with {self.session_dir}")
            end_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            new_dir = self.session_dir.replace("_ongoing", f"_to_{end_str}")
            rospy.loginfo(f"DEBUG: Attempting rename {self.session_dir} → {new_dir}")
            try:
                os.rename(self.session_dir, new_dir)
                rospy.loginfo(f"🛑 Session finalized: {new_dir}")
            except Exception as e:
                rospy.logerr(f"⚠️ Failed to finalize session: {e}")
            self.session_active = False
            self.session_dir = None

    def camera_callback(self, msg):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Determine detection condition
            if SIMULATED_PERSON_DETECTION:
                if SIMULATED_DETECTION_OPTION.lower() == "always":
                    detected = True
                elif SIMULATED_DETECTION_OPTION.lower() == "interval":
                    base_time = self.session_start if self.session_start is not None else time.time()
                    elapsed = (time.time() - base_time) / 60.0
                    detected = (elapsed % SIMULATED_DETECTION_CYCLE) < SIMULATED_DETECTION_ON_DURATION
                else:
                    detected = True
            else:
                detected = False
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                results = self.model(frame_rgb, verbose=False)
                for detection in results:
                    for det in detection.boxes:
                        class_id = int(det.cls)
                        confidence = float(det.conf)
                        if class_id == PERSON_CLASS_ID and confidence > CONFIDENCE_THRESHOLD:
                            detected = True
                            x1, y1, x2, y2 = map(int, det.xyxy[0])
                            cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0,255,0), 2)
                            cv2.putText(frame_bgr, f"Person {confidence:.2f}", (x1, y1-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                if detected:
                    rospy.loginfo(f"🚀 Person Detected! Confidence: {confidence:.2f}")
                else:
                    rospy.loginfo("⏳ No person detected...")

            # Session management: start session on detection; finalize when detection goes False.
            if detected and not self.session_active:
                self.create_session()
                self.session_active = True
            elif not detected and self.session_active:
                self.finalize_session()

            # Save images if session is active and the defined interval has passed
            if self.session_active:
                current_time = time.time()
                if current_time - self.last_save_time >= IMAGE_SAVE_INTERVAL:
                    timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S-%f")
                    if SAVE_RAW_IMAGES:
                        raw_filename = os.path.join(self.raw_dir, f"raw_{timestamp}.jpg")
                        cv2.imwrite(raw_filename, frame_bgr)
                        rospy.loginfo(f"📸 Raw Image Saved: {raw_filename}")
                    if SAVE_PROCESSED_IMAGES:
                        proc_filename = os.path.join(self.proc_dir, f"proc_{timestamp}.jpg")
                        cv2.imwrite(proc_filename, frame_bgr)
                        rospy.loginfo(f"🖼️ Processed Image Saved: {proc_filename}")
                    self.last_save_time = current_time

            ros_frame = self.bridge.cv2_to_imgmsg(frame_bgr, "bgr8")
            self.image_pub.publish(ros_frame)

            # Publish trigger transitions
            if detected and not self.lidar_active:
                self.trigger_pub.publish(True)
                self.lidar_active = True
                rospy.loginfo("🚀 Trigger ON")
            elif not detected and self.lidar_active:
                self.trigger_pub.publish(False)
                self.lidar_active = False
                rospy.loginfo("🛑 Trigger OFF")
            elif not detected:
                self.trigger_pub.publish(False)

        except Exception as e:
            rospy.logerr(f"⚠️ Error in camera callback: {e}")

    def shutdown_cleanup(self):
        rospy.loginfo("🛑 Shutting down Person Detector – resetting trigger.")
        self.trigger_pub.publish(False)
        self.trigger_pub.unregister()
        self.image_pub.unregister()
        if self.session_active:
            self.finalize_session()
        rospy.loginfo("✅ Shutdown complete.")

if __name__ == "__main__":
    detector = PersonDetector()
    rospy.spin()
