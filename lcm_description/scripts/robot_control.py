#!/usr/bin/python3

import rospy
import math
import time
from geometry_msgs.msg import Twist


# Helper function to sort the polygon coordinates in clockwise order
def sort_coordinates_clockwise(coordinates):
    center_lat = sum([coord[0] for coord in coordinates]) / len(coordinates)
    center_lon = sum([coord[1] for coord in coordinates]) / len(coordinates)
    return sorted(coordinates,
                  key=lambda coord: (math.atan2(coord[1] - center_lon, coord[0] - center_lat) + 2 * math.pi) % (
                          2 * math.pi))


# Helper function to check if a point is inside the polygon using ray casting algorithm
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

    # Sort the polygon coordinates in clockwise order
    polygon_coordinates = sort_coordinates_clockwise(polygon_coordinates)

    # Extract the minimum and maximum latitude and longitude values
    min_lat = int(min(polygon_coordinates, key=lambda x: x[0])[0])
    max_lat = int(max(polygon_coordinates, key=lambda x: x[0])[0])
    min_lon = int(min(polygon_coordinates, key=lambda x: x[1])[1])
    max_lon = int(max(polygon_coordinates, key=lambda x: x[1])[1])

    # Set the desired linear velocity
    linear_velocity = 0.7

    # Set the desired angular velocity
    angular_velocity = math.pi/2

    # Initialize the flag to alternate longitude direction
    alternate_direction = False
    start_lon = min_lon
    end_lon = max_lon
    step = 1

    # Traverse the entire area within the polygon
    for lat in range(min_lat, max_lat):

        # Set the starting longitude based on the alternate direction flag
        
        # Perform cleaning action for each longitude in the current latitude
        for lon in range(start_lon, end_lon, step):
            # Check if the current coordinates are within the polygon
            if is_inside_polygon(lat, lon, polygon_coordinates):
                # Perform cleaning action at (lat, lon)

                # Perform your cleaning action here

                # Set the linear and angular velocities in the Twist message
                cmd_vel_msg.linear.x = linear_velocity
                cmd_vel_msg.angular.z = 0.0

                # Publish the Twist message to control the robot
                cmd_vel_pub.publish(cmd_vel_msg)
                
                # Sleep for a small duration to simulate the cleaning action
                time.sleep(1.8)
                
            else:
                print("Stop")
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                cmd_vel_pub.publish(cmd_vel_msg)

        if (lat != (max_lat - 1) and is_inside_polygon(lat, lon, polygon_coordinates)):
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel_msg)
            time.sleep(1)
            if (lat%2 == 0):
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = angular_velocity
            else:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = -math.pi/2
            cmd_vel_pub.publish(cmd_vel_msg)
            time.sleep(1.5)
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel_msg)
            time.sleep(1)
            cmd_vel_msg.linear.x = linear_velocity
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel_msg)
            time.sleep(1.8)
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel_msg)
            if (lat%2 == 0):
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = angular_velocity
            else:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = -math.pi/2
            cmd_vel_pub.publish(cmd_vel_msg)
            time.sleep(1.5)
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel_msg)
            time.sleep(1)
        else:
            print("Cleaning complete")
        # Toggle the alternate direction flag
        alternate_direction = not alternate_direction
        if alternate_direction:
            start_lon = max_lon -1
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
    polygon_coordinates = [(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, 0.0)]
    try:
        control_robot(polygon_coordinates)
    except rospy.ROSInterruptException:
        pass
