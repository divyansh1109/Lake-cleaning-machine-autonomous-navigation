#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

robot_position = None

def odometry_callback(msg):
    global robot_position
    robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

def get_robot_position():
    global robot_position
    return robot_position

def trace_path():
    # Initialize the ROS node
    rospy.init_node('path_tracer', anonymous=True)

    # Create a publisher for the path
    path_pub = rospy.Publisher('/path', Path, queue_size=10)

    # Create an empty path message
    path_msg = Path()

    # Set the frame ID for the path
    path_msg.header.frame_id = 'map'  # Modify the frame ID to match your environment

    # Set the update rate for the path (adjust as needed)
    rate = rospy.Rate(5)  # 10 Hz

    # Subscribe to the odometry topic
    rospy.Subscriber('/odom', Odometry, odometry_callback)

    while not rospy.is_shutdown():
        # Get the robot's current position from Gazebo
        robot_position = get_robot_position()

        if robot_position is not None:
            # Create a new pose stamp with the current position
            pose_stamp = PoseStamped()
            pose_stamp.header.frame_id = path_msg.header.frame_id
            pose_stamp.pose.position.x = robot_position[0]
            pose_stamp.pose.position.y = robot_position[1]
            pose_stamp.pose.position.z = robot_position[2]  # Adjust if needed

            # Add the pose stamp to the path
            path_msg.poses.append(pose_stamp)

            # Publish the updated path
            path_pub.publish(path_msg)

        # Sleep to maintain the desired update rate
        rate.sleep()


if __name__ == '__main__':
    try:
        trace_path()
    except rospy.ROSInterruptException:
        pass
