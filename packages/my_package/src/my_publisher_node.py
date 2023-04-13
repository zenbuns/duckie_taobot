#!/usr/bin/env python3
import os
import rospy
import smbus2
import numpy as np
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Int32MultiArray

  
class MyPublisherNode(DTROS):
      def __init__(self, node_name):
          super().__init__(node_name=node_name, node_type=NodeType.GENERIC)
  
          # Create a publisher for sending wheel commands
          self.wheels_publisher = rospy.Publisher('taobot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
  
          # PID controller constants and parameters
          self.setpoint = 4.5
          self.kp = 0.3
          self.ki = 0.0001
          self.kd = 1.2
          self.integral_max = 4.5
          self.integral_min = -4.5
          self.l1 = np.array([])
          # PID controller state variables
          self.integral = 0.001
          self.prev_integral = 0
          self.last_error = 0.2
          self.last_time = rospy.Time.now()
          self.error_buffer = []
          self.buffer_size = 14

          self.cm_distance = 0
          self.wtravel = 0
          self.wtraveltmp = 0

          
  
          # Create an SMBus object for I2C communication with the hardware
          self.bus = smbus2.SMBus(1)
  
          # Subscribe to the range sensor topic
          rospy.Subscriber('taobot/front_distance_sensor_node/range', Range, self.tof_callback)
          rospy.Subscriber('taobot/i2c_line_data', Int32MultiArray, self.i2c_line_data_callback)


      def i2c_line_data_callback(self, data):
            self.l1 = np.array(data.data)

      def spin_ros(self):
          rospy.spin()
  
      def on_shutdown(self):
          speed = WheelsCmdStamped()
          speed.vel_left = 0
          speed.vel_right = 0
          self.wheels_publisher.publish(speed)
          rospy.on_shutdown(self.stop_robot)
  
      def tof_callback(self, data):
          self.tof_dist = data.range

      def obstacle(self):
            speed = WheelsCmdStamped()
            while self.cm_distance < 0.35:  # Detecting object 35cm away, the robot turns right
                speed.vel_left = 0.33
                speed.vel_right = 0.05
                self.wheels_publisher.publish(speed) 
                self.cm_distance = round(self.tof_dist*100,1)
                rospy.sleep(0.2)

            while self.wtraveltmp < 0.3:  # Robot travels 30cm straight
                speed.vel_left = 0.3
                speed.vel_right = 0.3
                self.wheels_publisher.publish(speed)
                self.wtraveltmp = self.wtraveltmp + self.wtravel
                rospy.sleep(0.2)

            speed.vel_left = 0.05  # Robot turns left
            speed.vel_right = 0.4
            self.wheels_publisher.publish(speed)
            rospy.sleep(1.2)

            speed.vel_left = 0.3  # Robot turns right
            speed.vel_right = 0.05
            self.wheels_publisher.publish(speed)
            rospy.sleep(0.5)
            self.wtraveltmp = 0

        

       
        
      
      
      def run(self):
        # Set the rate of loop
        rate = rospy.Rate(20)
        
        # Create a new WheelsCmdStamped object with current speed
        speed = WheelsCmdStamped()

        # Keep running while ROS is not shutdown
        while not rospy.is_shutdown():
            try:
                
                if self.cm_distance <= 0.35:
                    self.obstacle()

                # Calculate the error between the setpoint and the average of l1
                error = np.average(self.l1) - self.setpoint

                # Calculate the derivative error and integral error for the PID controller
                dt = rospy.Time.now() - self.last_time
                derivative = (error - self.last_error) / dt.to_sec()
                self.integral += sum(self.error_buffer) * error * dt.to_sec()
                self.integral = max(self.integral_min, min(self.integral, self.integral_max))

                # Calculate the PID correction factor
                correction = self.kp * error + self.ki * self.integral + self.kd * derivative
                correction = max(-1, min(correction, 1))

                if correction > 1 or correction < -1:
                    correction = self.last_correction
                else:
                    self.last_corrections = correction


                if max(self.l1) - min(self.l1) > 3:
                    turning_speed = 0.3
                if np.argmax(self.l1) > len(self.l1) // 2:
                    # Turn left
                    speed.vel_left = turning_speed
                    speed.vel_right = -turning_speed
                else:
                    # Turn right
                    speed.vel_left = -turning_speed
                    speed.vel_right = turning_speed



                # Set the speed of the left and right wheels
                speed.vel_left = 0.7 + correction
                speed.vel_right = 0.7 - correction

                # If no data is available from the sensors, use the previous speed values
                if len(self.l1) == 0:
                    speed.vel_left = self.previous_left
                    speed.vel_right = self.previous_right

                # Store the current speed values as previous speed values
                self.previous_left = speed.vel_left
                self.previous_right = speed.vel_right

                # Set the minimum and maximum speed values
                speed.vel_left = max(0.01, min(speed.vel_left, 0.9))
                speed.vel_right = max(0.01, min(speed.vel_right, 0.9))

                # Publish the speed values
                self.wheels_publisher.publish(speed)

                # Update the previous error value and time value for the next iteration
                self.last_error = error
                self.last_time = rospy.Time.now()

                # Sleep until the next iteration
                rate.sleep()

            except Exception as e:
                rospy.logerr("I2C communication error: {}".format(str(e)))
    
              
if __name__ == '__main__':
      node = MyPublisherNode(node_name='my_publisher_node')
     
  
      # Run the main control loop
      node.run()
      rospy.spin()
  


