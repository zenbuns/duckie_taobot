#!/usr/bin/env python3

import rospy
import smbus2
import numpy as np
from std_msgs.msg import Int32MultiArray

class I2CLineArrayPublisherNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)

        # Create a publisher for sending I2C line reading array
        self.i2c_line_data_publisher = rospy.Publisher('taobot/i2c_line_data', Int32MultiArray, queue_size=10)

        # Create an SMBus object for I2C communication with the hardware
        self.bus = smbus2.SMBus(1)

    def run(self):
        # Set the rate of loop
        rate = rospy.Rate(20)

        # Keep running while ROS is not shutdown
        while not rospy.is_shutdown():
            try:
                # Read the temperature data from the sensors
                temp = self.bus.read_byte_data(62, 17)

                # Check if temp is positive or negative
                # Convert to binary
                binary = np.zeros(8, dtype=np.uint8)
                for i in range(8):
                    binary[7-i] = (temp & (1 << i)) >> i

                # Find the positions of the set bits in the binary representation
                l1 = np.nonzero(binary)[0] + 1

                # Publish the I2C line reading array
                i2c_line_data_msg = Int32MultiArray()
                i2c_line_data_msg.data = l1.tolist()
                self.i2c_line_data_publisher.publish(i2c_line_data_msg)

                # Sleep until the next iteration
                rate.sleep()

            except Exception as e:
                rospy.logerr("I2C communication error: {}".format(str(e)))


if __name__ == '__main__':
    node = I2CLineArrayPublisherNode(node_name='i2c_line_array_publisher_node')
    node.run()
    rospy.spin()
