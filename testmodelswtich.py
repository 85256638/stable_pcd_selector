#!/usr/bin/env python
import rospy
from livox_ros_driver import SwitchLidarMode, SwitchLidarModeRequest
import time

# Define LiDAR mode constants as per livox_def.h
kLidarModeNormal = 1
kLidarModePowerSaving = 2
kLidarModeStandby = 3

def switch_mode_client(handle, mode):
    rospy.wait_for_service('switch_lidar_mode')
    try:
        switch_mode = rospy.ServiceProxy('switch_lidar_mode', SwitchLidarMode)
        req = SwitchLidarModeRequest()
        req.handle = handle
        req.mode = mode
        response = switch_mode(req)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None

def test_switch_mode():
    rospy.init_node('test_switch_lidar_mode', anonymous=True)
    handle = 0  # Change this if you want to test a different LiDAR handle
    
    rospy.loginfo("Switching LiDAR %d to power-saving mode...", handle)
    resp = switch_mode_client(handle, kLidarModePowerSaving)
    rospy.loginfo("Response: %s", resp)
    
    time.sleep(5)  # wait for 5 seconds
    
    rospy.loginfo("Switching LiDAR %d to normal mode...", handle)
    resp = switch_mode_client(handle, kLidarModeNormal)
    rospy.loginfo("Response: %s", resp)

if __name__ == '__main__':
    test_switch_mode()
