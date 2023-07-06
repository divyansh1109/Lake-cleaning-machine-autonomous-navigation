#!/usr/bin/python3

import rospy
import math
import time
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

angular_velocity_w = 1.49


def imu_callback(msg, cmd_vel_msg):
    global angular_velocity_w, yaw, target_yaw

    # Extract the yaw angle from the IMU data
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )

    # yaw (z-axis rotation)
    siny_cosp = +2.0 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y)
    cosy_cosp = +1.0 - 2.0 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    # print(f"yaw = {yaw}")
    target_yaw = yaw + math.pi / 2
    if (label):
        if (alternate_direction == False):
            # Determine the desired yaw angle for the next 90-degree turn
            target_yaw = yaw + math.pi / 2
        elif (alternate_direction == True):
            # Determine the desired yaw angle for the next 90-degree turn
            target_yaw = abs(yaw) - math.pi / 2

        # print(f"imu_data = {yaw}, ang-vel = {angular_velocity_w}")


# Helper function to sort the polygon coordinates in clockwise order
def sort_coordinates_clockwise(coordinates):
    center_lat = sum([coord[0] for coord in coordinates]) / len(coordinates)
    center_lon = sum([coord[1] for coord in coordinates]) / len(coordinates)
    return sorted(coordinates,
                  key=lambda coord: (math.atan2(coord[1] - center_lon, coord[0] - center_lat) + 2 * math.pi) % (
                          2 * math.pi))


# Helper function to check if a point is inside the polygon using the ray casting algorithm
def is_inside_polygon(lat, lon, polygon_coordinates):
    inside = False
    j = len(polygon_coordinates) - 1

    for i in range(len(polygon_coordinates)):
        if ((polygon_coordinates[i][1] > lon) != (polygon_coordinates[j][1] > lon)) and (
                lat < (polygon_coordinates[j][0] - polygon_coordinates[i][0]) * (lon - polygon_coordinates[i][1]) /
                (polygon_coordinates[j][1] - polygon_coordinates[i][1]) + polygon_coordinates[i][0]):
            inside = not inside
        j = i

    return inside


def control_robot(polygon_coordinates):
    # Initialize the ROS node
    rospy.init_node('robot_control', anonymous=True)

    # Create a publisher to control the robot's velocity
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create a Twist message to set the robot's velocity
    cmd_vel_msg = Twist()
    global label, alternate_direction
    alternate_direction = False
    label = False

    rospy.Subscriber('/imu_feedback', Imu, lambda msg: imu_callback(msg, cmd_vel_msg))
    # Sort the polygon coordinates in clockwise order
    polygon_coordinates = sort_coordinates_clockwise(polygon_coordinates)

    # Extract the minimum and maximum latitude and longitude values
    min_lat = int(min(polygon_coordinates, key=lambda x: x[0])[0])
    max_lat = int(max(polygon_coordinates, key=lambda x: x[0])[0])
    min_lon = int(min(polygon_coordinates, key=lambda x: x[1])[1])
    max_lon = int(max(polygon_coordinates, key=lambda x: x[1])[1])

    # Set the desired linear velocity
    linear_velocity = 0.49

    # Initialize the flag to alternate longitude direction

    start_lon = min_lon
    end_lon = max_lon
    step = 1

    # Traverse the entire area within the polygon
    for lat in range(min_lat, max_lat):

        # Perform the cleaning action for each longitude in the current latitude
        for lon in range(start_lon, end_lon, step):
            # Check if the current coordinates are within the polygon
            if is_inside_polygon(lat, lon, polygon_coordinates):
                # Set the linear and angular velocities in the Twist message
                cmd_vel_msg.linear.x = linear_velocity
                cmd_vel_msg.angular.z = 0.0

                # Publish the Twist message to control the robot
                cmd_vel_pub.publish(cmd_vel_msg)

                # Sleep for a small duration to simulate the cleaning action
                time.sleep(2.15)

        if lat != (max_lat - 1) and is_inside_polygon(lat, lon, polygon_coordinates):
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel_msg)
            time.sleep(1.5)
            label = True
            # print(f"target_yaw1 = {target_yaw}")
            #c1_yaw = target_yaw
            t0 = rospy.Time.now().to_sec()
            if (alternate_direction == False):
                while True:
                    t1 = rospy.Time.now().to_sec()
                    cmd_vel_msg.linear.x = 0.0
                    cmd_vel_msg.angular.z = angular_velocity_w
                    cmd_vel_pub.publish(cmd_vel_msg)
                    current_angle_degree = (t1 - t0) * angular_velocity_w
                    if (current_angle_degree >= math.pi / 2 - 0.02):
                        break
            else:
                while True:
                    t1 = rospy.Time.now().to_sec()
                    cmd_vel_msg.linear.x = 0.0
                    cmd_vel_msg.angular.z = -angular_velocity_w
                    cmd_vel_pub.publish(cmd_vel_msg)
                    current_angle_degree = (t1 - t0) * angular_velocity_w
                    if (current_angle_degree >= math.pi / 2 - 0.02):
                        break

            label = False
            print(f"current_angular1: {cmd_vel_msg.angular.z}")
            time.sleep(0.5)
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel_msg)
            time.sleep(1.5)
            cmd_vel_msg.linear.x = linear_velocity
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel_msg)
            time.sleep(2.15)
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel_msg)
            time.sleep(1.5)
            label = True
            # print(f"target_yaw2 = {target_yaw}")
            #c2_yaw = target_yaw
            t0 = rospy.Time.now().to_sec()
            if (alternate_direction == False):
                while True:
                    t1 = rospy.Time.now().to_sec()
                    cmd_vel_msg.linear.x = 0.0
                    cmd_vel_msg.angular.z = angular_velocity_w
                    cmd_vel_pub.publish(cmd_vel_msg)
                    current_angle_degree = (t1 - t0) * angular_velocity_w
                    if (current_angle_degree >= math.pi / 2 - 0.02):
                        break
            else:
                while True:
                    t1 = rospy.Time.now().to_sec()
                    cmd_vel_msg.linear.x = 0.0
                    cmd_vel_msg.angular.z = -angular_velocity_w
                    cmd_vel_pub.publish(cmd_vel_msg)
                    current_angle_degree = (t1 - t0) * angular_velocity_w
                    if (current_angle_degree >= math.pi / 2 - 0.02):
                        break

            label = False
            print(f"current_angular2: {cmd_vel_msg.angular.z}")
            time.sleep(0.5)
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel_msg)
            time.sleep(1.5)

        else:
            print("Cleaning complete")

        # Toggle the alternate direction flag
        alternate_direction = not alternate_direction

        if alternate_direction:
            start_lon = max_lon - 1
            end_lon = min_lon
            step = -1
        else:
            start_lon = min_lon
            end_lon = max_lon - 1
            step = 1

    # Stop the robot by setting linear and angular velocities to zero
    cmd_vel_msg.linear.x = 0.0
    cmd_vel_msg.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel_msg)

    # Shutdown the ROS node
    rospy.signal_shutdown("Cleaning Complete")


if __name__ == '__main__':
    polygon_coordinates = [(0.0, 0.0), (0.0, 5.0), (5.0, 5.0), (5.0, 0.0)]
    try:
        control_robot(polygon_coordinates)
    except rospy.ROSInterruptException:
        pass
