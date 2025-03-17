#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time
from datetime import datetime
import shutil

# Configurable parameters
SAVE_INTERVAL = 5.0  # Seconds between saving images
DATASET_DIR = "/home/taixing/pretraindataset"  # Directory to save images

def log_remaining_disk_space():
    try:
        total, used, free = shutil.disk_usage("/")
        free_gb = free / (1024**3)
        rospy.loginfo("Remaining disk space: %.2f GB", free_gb)
    except Exception as e:
        rospy.logwarn("Could not determine disk space: %s", e)

class ImageDatasetCollector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_dataset_collector', anonymous=True)
        
        # Create a CvBridge instance to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()
        self.last_save_time = 0.0
        
        # Get parameters (allowing overrides via ROS parameters)
        self.dataset_dir = rospy.get_param("~dataset_dir", DATASET_DIR)
        self.save_interval = rospy.get_param("~save_interval", SAVE_INTERVAL)
        
        # Create the dataset directory if it does not exist
        if not os.path.exists(self.dataset_dir):
            os.makedirs(self.dataset_dir)
        
        # Subscribe to the camera image topic
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        rospy.loginfo("Started image dataset collector. Saving images every %.1f seconds to %s", 
                      self.save_interval, self.dataset_dir)
        log_remaining_disk_space()

    def image_callback(self, msg):
        current_time = time.time()
        # Save image only if the defined interval has passed
        if current_time - self.last_save_time >= self.save_interval:
            try:
                # Convert the ROS Image message to an OpenCV BGR image
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: %s", e)
                log_remaining_disk_space()
                return

            # Log the image resolution: shape returns (height, width, channels)
            rospy.loginfo("Image resolution: %d x %d", cv_image.shape[1], cv_image.shape[0])
            log_remaining_disk_space()
            
            # Use ROS system time for the filename
            ros_now = rospy.Time.now().to_sec()
            dt = datetime.fromtimestamp(ros_now)
            time_str = dt.strftime('%Y-%m-%d_%H:%M:%S')
            filename = os.path.join(self.dataset_dir, f"image_{time_str}.jpg")
            
            # Save the image using OpenCV
            cv2.imwrite(filename, cv_image)
            rospy.loginfo("Saved image: %s", filename)
            log_remaining_disk_space()
            
            # Update the last save time
            self.last_save_time = current_time

if __name__ == '__main__':
    try:
        collector = ImageDatasetCollector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
