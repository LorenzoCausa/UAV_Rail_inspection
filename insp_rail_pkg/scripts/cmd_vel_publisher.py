#!/usr/bin/env python3

import sys
import socket
import json
import time
import os
# ROS
import rospy 
from my_custom_interfaces.msg import Drone_cmd

yaw=0
pitch=0
roll=0
throttle=0

def callback(data):
    global yaw,pitch,roll,throttle
    yaw = data.yaw
    pitch = data.pitch
    roll = data.roll
    throttle = data.throttle

if __name__ == "__main__":
    rospy.init_node('cmd_vel_publisher')
    pub=rospy.Publisher('cmd_vel',Drone_cmd,queue_size=1)
    rospy.Subscriber("command", Drone_cmd, callback)

    print("Publishing")
    rate = rospy.Rate(15) # 15hz
    while not rospy.is_shutdown():
        command=Drone_cmd()
        command.yaw=yaw
        command.pitch=pitch
        command.roll=roll
        command.throttle=throttle
        pub.publish(command)
        rate.sleep()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")