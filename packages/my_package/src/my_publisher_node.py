#!/usr/bin/env python3
import os
import rospy
import smbus2
import numpy as np
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
import time


class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # Create a publisher for sending wheel commands
        self.wheels_publisher = rospy.Publisher('/taobot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

        # Create subscribers for the front center range, right wheel encoder, and left wheel encoder
        self.tof_subscriber = rospy.Subscriber('/taobot/front_center_tof_driver_node/range', Range, self.tof_callback)
        self.rwheel_subscriber = rospy.Subscriber('/taobot/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.right_wheel_callback)
        self.lwheel_subscriber = rospy.Subscriber('/taobot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_wheel_callback)

        # Create subscribers for the right and left wheel sequence numbers
        self.seq_left_subscriber = rospy.Subscriber('/taobot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.time_left_wheel_callback)
        self.seq_right_subscriber = rospy.Subscriber('/taobot/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.time_right_wheel_callback)

        # PID controller constants and parameters
        self.setpoint = 4.5
        self.kp = 0.09
        self.ki = 0.01
        self.kd = 0.9
        self.integral_max = 0.5
        self.integral_min = -0.5

        # PID controller state variables
        self.integral = 0.001
        self.prev_integral = 0
        self.last_error = 0.2
        self.last_time = rospy.get_time()
        self.total_error = 0
        self.best_error = float('inf')
        self.counter = 0
        self.index = 0
        self.dp = [0.01, 0.0001, 0.5]
        self.params = [self.kp, self.ki, self.kd]

        self.bus = smbus2.SMBus(1)   # Create an SMBus object for I2C communication with the hardware
        self.range = 1              # Current range measured by the TOF sensor
        self.right = 0              # Current tick count for the right wheel encoder
        self.left = 0               # Current tick count for the left wheel encoder
        self.time_left = 0          # Time of the last left encoder measurement
        self.time_right = 0         # Time of the last right encoder measurement
        self.ticks_left = 0         # Total number of ticks for the left encoder
        self.prev_tick_left = 0     # Previous tick count for the left encoder
        self.ticks_right = 0        # Total number of ticks for the right encoder
        self.prev_tick_right = 0    # Previous tick count for the right encoder
        self.rotation_wheel_left = 0   # Amount of rotation in radians for the left wheel
        self.rotation_wheel_right = 0  # Amount of rotation in radians for the right wheel
        self.delta_ticks_left = 0   # Change in tick count for the left encoder since the last measurement
        self.delta_ticks_right = 0  # Change in tick count for the right encoder since the last measurement
        self.x_curr = 0             # Current x-coordinate of the robot's position
        self.y_curr = 0             # Current y-coordinate of the robot's position
        self.theta_curr = 0         # Current orientation of the robot
        self.n_tot = 135            # Total number of ticks for one full rotation of a wheel
        self.baseline_wheel2wheel = 0.095  # Distance between the centers of the two wheels in meters
     


    def rightwheel(self, data):
        self.r_tick = data.data
        rospy.loginfo("Parem ratas: %s", data.data)
    
    def leftwheel(self, data):
        self.l_tick = data.data
        rospy.loginfo("Vasak ratas: %s", data.data)
    
    def time_leftwheel(self, data):
        self.timeL = data.header.seq

    def time_rightwheel(self, data):
        self.timeR = data.header.seq

    

    def on_shutdown(self):
        speed = WheelsCmdStamped()
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown(self.stop_robot)

    def tof_callback(self, data):
        self.tof_dist = data.range
        print(data)

    def twiddle(self):
        # If the counter is 0, add the delta parameter to the current parameter
        if self.counter == 0:
            self.params[self.index] += self.dp[self.index]
        
        # If the counter is 1, check if the current error is better than the best error. 
        # If it is, update the best error and increase the delta parameter for the current index. 
        # If it's not, decrease the delta parameter and move to the next index.
        elif self.counter == 1:
            if self.total_error < self.best_error:
                self.best_error = self.total_error
                self.dp[self.index] *= 1.1
                self.index = (self.index + 1) % 3
            else:
                self.params[self.index] -= 2 * self.dp[self.index]

        # If the counter is 2, check if the current error is better than the best error. 
        # If it is, update the best error and increase the delta parameter for the current index. 
        # If it's not, decrease the delta parameter and move to the next index.
        elif self.counter == 2:
            if self.total_error < self.best_error:
                self.best_error = self.total_error
                self.dp[self.index] *= 1.1
            else:
                self.params[self.index] += self.dp[self.index]
                self.dp[self.index] *= 0.9
                self.index = (self.index + 1) % 3
        
        # Update the counter to cycle through the three steps
        self.counter = (self.counter + 1) % 3


        # Apply the optimized parameters to the PID controller
        self.kp, self.ki, self.kd = self.params
        
    def run(self):
        # set the rate of loop
        rate = rospy.Rate(28)

        # keep running while ros is not shutdown
        while not rospy.is_shutdown():
            # Run the twiddle optimization algorithm
            self.twiddle()

            # Create a new WheelsCmdStamped object with current speed
            speed = WheelsCmdStamped()

            # Set variables for wheel and encoder configuration
            n_tot = 135
            R = 0.0318
            self.ticks_right = self.right
            self.ticks_left = self.left
            self.delta_ticks_left = self.ticks_left - self.prev_tick_left
            self.delta_ticks_right = self.ticks_right - self.prev_tick_right
            self.rotation_wheel_left = (2 * np.pi / n_tot) * self.delta_ticks_left
            self.rotation_wheel_right = (2 * np.pi / n_tot) * self.delta_ticks_right
            d_left = R * self.rotation_wheel_left
            d_right = R * self.rotation_wheel_right

            # Calculate the change in orientation of the robot based on the movement of the wheels
            Dtheta = (d_right - d_left) / self.baseline_wheel2wheel

            # print values for debugging
            print(self.ticks_left)
            print(self.ticks_right)
            print(np.rad2deg(Dtheta))

            # update the current position of the robot
            self.prev_tick_left = self.ticks_left
            self.prev_tick_right = self.ticks_right

            # read the temperature data from the sensors
            temp = self.bus.read_byte_data(62, 17)

            # Check if temp is positive or negative
            # Convert to binary
            binary = bin(temp)[2:].zfill(8)

            #enumerate the values starting from 1
            # Add a threshold value
             l1 = [index + 1 for index, ele in enumerate(binary) if ele == "1"]

            # calculate the error between the setpoint and the average of l1
            error = self.setpoint - np.average(l1)

            # calculate the derivative error and integral error for the PID controller
            dt = rospy.get_time() - self.last_time
            derivative = (error - self.last_error) / dt
            self.integral = max(self.integral_min, min(self.integral, self.integral_max))
            self.integral += error * dt

            # calculate the PID correction factor
            correction = self.kp * error + self.ki * self.integral + self.kd * derivative

            # set the speed of the left and right wheels
            speed.vel_left = 0.7 - correction
            speed.vel_right = 0.7 + correction

            # if no data is available from the sensors, use the previous speed values
            if len(l1) == 0:
                speed.vel_left = self.previous_left
                speed.vel_right = self.previous_right

            # store the current speed values as previous speed values
            self.previous_left = speed.vel_left
            self.previous_right = speed.vel_right

            # set the minimum and maximum speed values
            speed.vel_left = max(0.01, min(speed.vel_left, 0.9))
            speed.vel_right = max(0.01, min(speed.vel_right, 0.9))

            # publish the speed values
            self.pub.publish(speed)

            # update the previous error value and time value for the next iteration
            self.last_error = error
            self.last_time = rospy.get_time()

            # sleep until the next iteration
            rate.sleep()




if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    node.run()
    rospy.spin()
