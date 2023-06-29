#!/usr/bin/python3

import rospy
import math
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu


def imu_callback(msg):
    global angular_velocity_w
    # Extract the yaw angle from the IMU data
    q = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    
    # roll (x-axis rotation)
    sinr_cosp = +2.0 * (msg.orientation.w * msg.orientation.x + msg.orientation.y * msg.orientation.z)
    cosr_cosp = +1.0 - 2.0 * (msg.orientation.x * msg.orientation.x + msg.orientation.y * msg.orientation.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = +2.0 * (msg.orientation.w * msg.orientation.y - msg.orientation.z * msg.orientation.x)
    if (math.fabs(sinp) >= 1):
        pitch = math.copysign(M_PI / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = +2.0 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y)
    cosy_cosp = +1.0 - 2.0 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z)  
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    print(f"yaw = {yaw}, roll = {roll}, pitch = {pitch}")
    
if __name__ == '__main__':
	rospy.init_node('Imu_data')
	rospy.Subscriber('/imu_feedback', Imu, imu_callback)
	rospy.spin()
