#!/usr/bin/env python3
import numpy as np
import math
import rospy
import os
from duckietown_msgs.msg import WheelEncoderStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose
import tf.transformations as tf_trans



class CustomOdometry:

    def __init__(self, max_ticks, wheel_radius, wheel_base):
        self.max_ticks = max_ticks  # N_tot = total number of ticks per revolution
        # Radius "R" of duckiebots wheel is 3,3cm.
        self.wheel_radius = wheel_radius / 100  # Insert value measured by ruler, in *meters*
        # Distance between wheels = baseline_wheel2wheel = 10cm / 100 (in meters)
        self.baseline_wheel2wheel = wheel_base / 100
        self.veh_name = os.environ["VEHICLE_NAME"]

        # This data going to be be recieved by subscribing to ros topic
        self.ticks_left = 0
        self.ticks_right = 0

        # construct wheel encoders and tof sensor subscribers
        self.left_encoder_data = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick',
                                                  WheelEncoderStamped, self._left_encoder_data)
        self.right_encoder_data = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick',
                                                   WheelEncoderStamped, self._right_encoder_data)

        self.prev_tick_left = 0
        self.prev_tick_right = 0
        # Initial X, Y and angular positions
        self.prev_x = 0
        self.prev_y = 0
        self.prev_angular_pos = 0
        self.prev_distance = 0

    def _left_encoder_data(self, data):
        self.ticks_left = data.data

    def _right_encoder_data(self, data):
        self.ticks_right = data.data

    def get_ticks(self):
        return self.ticks_left, self.ticks_right

    def overwrite_prev_data(self, current_x, current_y, current_ang, tick_l, tick_r, total_distance):
        self.prev_tick_left = tick_l
        self.prev_tick_right = tick_r
        self.prev_x = current_x
        self.prev_y = current_y
        self.prev_angular_pos = current_ang
        self.prev_distance = total_distance

    def get_alpha(self):
        alpha = (2 * math.pi) / self.max_ticks  # "ALPHA" means rotation per tick in radians.
        return alpha

    def delta_phi(self):
        """
        How much would the wheels rotate with the above tick measurements?
        DELTA means the difference between two values.
        Delta ticks means the difference of current and previous encoder ticks.
        """
        delta_ticks_left = self.ticks_left - self.prev_tick_left  # delta ticks of left wheel
        delta_ticks_right = self.ticks_right - self.prev_tick_right  # delta ticks of right wheel
        """
        Calculate Delta-PHI (wheel rotation in radians)
        Formula: DELTA-PHI = DELTA TICKS * ALPHA
        """
        wheel_rotation_left = delta_ticks_left * self.get_alpha()  # total rotation of left wheel
        wheel_rotation_right = delta_ticks_right * self.get_alpha()  # total rotation of right wheel

        return wheel_rotation_left, wheel_rotation_right

    def pose_estimate(self):
        """
        What is the distance travelled by each wheel?
        Delta-Phi is wheel rotation in radians
        Arc distance formula: ARC = Delta-Phi * Radius
        """
        wheel_rotation_left, wheel_rotation_right = self.delta_phi()
        wheel_distance_left = wheel_rotation_left * self.wheel_radius
        wheel_distance_right = wheel_rotation_right * self.wheel_radius
        """
        How much has the robot travelled?
        Point "A" is absolute distance measured from the center of the robots frame (center of wheelbase).
        Robots distance travelled in robot frame [meters], measured from point A.
        """
        absolute_distance = (wheel_distance_left + wheel_distance_right) / 2

        """
        # Calculate robots angular position "Delta-THETA"
        # Formula: (Right wheels distance - Left wheel distance) / distance between wheels

        # New position in enviroment. Expressed by "X" and "Y" cordinates.
        # Delta-X is the change in X axis.
        # Formula: Delta-X = absolute_distance(distance of point "A") * cosines of angular_pos(delta-theta)

        # Delta-Y is the change in Y axis.
        # Formula: Delta-X = absolute_distance(distance of point "A") * sine of angular_pos(delta-theta)
        """
        delta_angular_pos = (wheel_distance_right - wheel_distance_left) / self.baseline_wheel2wheel  # delta-theta in radians
        delta_x = absolute_distance * np.cos(delta_angular_pos)
        delta_y = absolute_distance * np.sin(delta_angular_pos)

        current_x = self.prev_x + delta_x
        current_y = self.prev_y + delta_y
        current_angular_pos = self.prev_angular_pos + delta_angular_pos
        total_distance = self.prev_distance + absolute_distance


        return current_x, current_y, current_angular_pos, total_distance  # x_curr, y_curr, theta_curr


def talker():
    veh_name = os.environ["VEHICLE_NAME"]
    odometry = CustomOdometry(max_ticks=135, wheel_radius=0.0318, wheel_base=0.10)
    pub = rospy.Publisher(f'/{veh_name}/odometry', Odometry, queue_size=10)
    rospy.init_node('odometry', anonymous=True)
    rate = rospy.Rate(20)  # 20hz
    while not rospy.is_shutdown():
        x_pos, y_pos, angular_pos, total_distance = odometry.pose_estimate()
        ticks_l, ticks_r = odometry.get_ticks()

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = x_pos
        odom_msg.pose.pose.position.y = y_pos
        odom_msg.pose.pose.position.z = 0

        quat = tf_trans.quaternion_from_euler(0, 0, angular_pos)
        odom_msg.pose.pose.orientation = Quaternion(*quat)

        pub.publish(odom_msg)
        odometry.overwrite_prev_data(x_pos, y_pos, angular_pos, ticks_l, ticks_r, total_distance)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



        

#
# max_ticks = 135  # N_tot = total number of ticks per revolution
# pi = math.pi
# alpha = (2 * pi) / max_ticks  # "ALPHA" means rotation per tick in radians.
#
# # Ticks data from the encoders to calculate the difference
# ticks_left = 100  # This data should be recieved by subscribing to ros topic
# prev_tick_left = 0  # This should be overwritten in the end of each loop with the value of ticks_left.
#
# ticks_right = 10  # This data should be recieved by subscribing to ros topic
# prev_tick_right = 0  # This should be overwritten in the end of each loop with the value of ticks_right.
#
# # Initial position of duckiebot
# initial_x = initial_y = 0  # linear position in meters along X and Y axis
# initial_angular_pos = 0  # theta0 angular position in radians
#
# # How much would the wheels rotate with the above tick measurements?
# # DELTA means the difference between two values.
# # Delta ticks means the difference of current and previous encoder ticks.
# delta_ticks_left = ticks_left - prev_tick_left  # delta ticks of left wheel
# delta_ticks_right = ticks_right - prev_tick_right  # delta ticks of right wheel
#
# # Calculate Delta-PHI (wheel rotation)
# # Formula: DELTA-PHI = DELTA TICKS * ALPHA
# wheel_rotation_left = delta_ticks_left * alpha  # total rotation of left wheel
# wheel_rotation_right = delta_ticks_right * alpha  # total rotation of right wheel
#
# print(f"The left wheel rotated: {np.rad2deg(wheel_rotation_left)} degrees")
# print(f"The right wheel rotated: {np.rad2deg(wheel_rotation_right)} degrees")
#
# # Radius "R" of duckiebots wheel is 3,3cm.
# wheel_radius = 3.3 / 100  # Insert value measured by ruler, in *meters*
#
# # What is the distance travelled by each wheel?
# # Delta-Phi is wheel rotation in radians
# # Arc distance formula: ARC = Delta-Phi * Radius
# wheel_distance_left = wheel_rotation_left * wheel_radius
# wheel_distance_right = wheel_rotation_right * wheel_radius
#
# print(f"The left wheel travelled: {wheel_distance_left} meters")
# print(f"The right wheel rotated: {wheel_rotation_right} meters")
#
# # How much has the robot travelled?
# # Point "A" is absolute distance measeured from the center of the robots frame (center of wheelbase).
# # robot distance travelled in robot frame [meters], measured from point A.
# absolute_distance = (wheel_distance_left + wheel_distance_right) / 2
# print(f"The robot has travelled: {absolute_distance} meters")
#
# # Distance between wheels = baseline_wheel2wheel = 10cm / 100 (for meters)
# baseline_wheel2wheel = 10 / 100
#
# # Calculate robots angular position "Delta-THETA"
# # Formula: Right wheels distance - Left wheel distance / distance between wheels
# angular_pos = (wheel_distance_right - wheel_distance_left) / 10  # delta-theta in radians
#
# # New position in enviroment. Expressed by "X" and "Y" scale.
# # Delta-X is the change in X axis.
# # Formula: Delta-X = absolute_distance(distance of point "A") * cosines of angular_pos(delta-theta)
# delta_x = absolute_distance * np.cos(angular_pos)
#
# # Delta-Y is the change in Y axis.
# # Formula: Delta-X = absolute_distance(distance of point "A") * sine of angular_pos(delta-theta)
# delta_y = absolute_distance * np.sin(angular_pos)

# !/usr/bin/env python
# license removed for brevity