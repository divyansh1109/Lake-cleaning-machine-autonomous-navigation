#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu

robot_position = None

def imu_callback(msg):
    # Extract the yaw angle from the IMU data
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    _, _, yaw = euler_from_quaternion(quaternion)

    # Determine the desired yaw angle for the next 90-degree turn
    target_yaw = yaw + math.pi / 2

    # Calculate the angular velocity needed to reach the target yaw
    angular_velocity = 1.0  # Set an initial angular velocity
    kp = 1.0  # Proportional gain for controlling the angular velocity
    error = target_yaw - yaw
    while error > math.pi:
        error -= 2 * math.pi
    while error < -math.pi:
        error += 2 * math.pi
    angular_velocity = kp * error

    # Set the angular velocity in the Twist message
    cmd_vel_msg.angular.z = angular_velocity

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
    #rospy.Subscriber('/imu_feedback', Imu, imu_callback)

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
