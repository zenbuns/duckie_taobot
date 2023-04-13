#!/usr/bin/env python3
import os
import numpy as np
import rospy
from std_msgs.msg import String
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
from sensor_msgs.msg import Range
import time
speed = WheelsCmdStamped()
class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        # Publisherid ja subscriberid
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        #rospy.init_node('odometry', anonymous=True)
        self.pub = rospy.Publisher('/taobot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.tof = rospy.Subscriber('/taobot/front_center_tof_driver_node/range', Range, self.callback)
        self.rwheel = rospy.Subscriber('/taobot/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.rightwheel)
        self.lwheel = rospy.Subscriber('/taobot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.leftwheel)
        self.range = 1
        self.right = 0
        self.left = 0
        self.ticks_left = 0
        self.prev_tick_left = 0
        self.ticks_right = 0
        self.prev_tick_right = 0
        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0
        self.delta_ticks_left = 0
        self.delta_ticks_right = 0
        self.n_tot = 135
        self.kaugus_cm = 0
        self.wtravel = 0
        self.wtraveltmp = 0

    def callback(self, data):
        self.range = data.range
    def rightwheel(self, data):
        self.right = data.data
    def leftwheel(self, data):
        self.left = data.data

    def obstacle(self):
        while self.kaugus_cm <= 35: #tuvastades objekti 35cm kauguselt, pöörab robot paremale
            speed.vel_left = 0.33
            speed.vel_right = 0.05
            self.pub.publish(speed)
            self.kaugus_cm = round(self.range*100, 1)
        time.sleep(0.2)
        while self.wtraveltmp < 30: #robot sõidab 30cm otse
            speed.vel_left = 0.3
            speed.vel_right = 0.3
            self.pub.publish(speed)
            self.wtraveltmp = self.wtraveltmp + self.wtravel
        time.sleep(0.4)
        speed.vel_left = 0.05 #robot pöörab vasakule
        speed.vel_right = 0.4
        self.pub.publish(speed)
        time.sleep(1.2)
        speed.vel_left = 0.3  #robot pöörab paremale
        speed.vel_right = 0.05
        self.pub.publish(speed)
        time.sleep(0.7)
        self.wtraveltmp = 0

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        time.sleep(0.2)
        rospy.on_shutdown()
    def talker(self):
        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            R = 0.0318 #ratta raadius
            self.ticks_right = self.right
            self.ticks_left = self.left
            self.delta_ticks_left = self.ticks_left-self.prev_tick_left
            self.delta_ticks_right = self.ticks_right-self.prev_tick_right
            self.rotation_wheel_left = (2*np.pi/self.n_tot)*self.delta_ticks_left #vasaku ratta pöördenurk (rad)
            self.rotation_wheel_right = (2*np.pi/self.n_tot)*self.delta_ticks_right #parema ratta pöördenurk (rad)
            d_left = R * self.rotation_wheel_left #vasaku ratta läbitud vahemaa (cm)
            d_right = R * self.rotation_wheel_right #parema ratta läbitud vahemaa (cm)
            self.prev_tick_left = self.ticks_left
            self.prev_tick_right = self.ticks_right
            self.wtravel = round(((d_left + d_right)*100)/2, 1) #roboti läbitud vahemaa (cm)
            self.kaugus_cm = round(self.range*100, 1) #TOF sensori tuvastatud kaugus (cm)
            tof = self.kaugus_cm

            #string = f"range: {tof}"
            if self.kaugus_cm <= 35:
                self.odom.publish("tof in progress")
                while self.kaugus_cm <= 35: #tuvastades objekti 35cm kauguselt, pöörab robot paremale
                    speed.vel_left = 0.33
                    speed.vel_right = 0.05
                    self.pub.publish(speed)
                    self.kaugus_cm = round(self.range*100, 1)
                    print("tof part 1")
                time.sleep(0.2)
                while self.wtraveltmp < 30: #robot sõidab 30cm otse
                    speed.vel_left = 0.3 #0.3
                    speed.vel_right = 0.3 #0.3
                    self.pub.publish(speed)
                    self.wtraveltmp = self.wtraveltmp + self.wtravel
                    print("tof part 2")
                time.sleep(0.5)
                speed.vel_left = 0.05 #robot pöörab vasakule
                speed.vel_right = 0.4
                self.pub.publish(speed)
                time.sleep(1.2)
                speed.vel_left = 0.3  #robot pöörab paremale
                speed.vel_right = 0.05
                print("tof part 3")
                self.pub.publish(speed)
                time.sleep(0.5)
                self.wtraveltmp = 0
                
            else:
                self.tof.publish("tof NOT in progress")
            rate.sleep()


if __name__ == '__main__':
    node = MyPublisherNode(node_name="tof_node")
    node.talker()
    rospy.spin()