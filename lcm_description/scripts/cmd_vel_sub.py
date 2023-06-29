#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist

def callback(data):
    linear_vel = data.linear.x
    angular_vel = data.angular.z

    rospy.loginfo("Received linear velocity: %f" % linear_vel)
    rospy.loginfo("Received angular velocity: %f" % angular_vel)
    rospy.loginfo("Time Taken: %f" % rospy.get_time())

def listener():
    rospy.init_node('cmd_vel_sub', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, callback)

    rospy.spin()
while(1):
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

