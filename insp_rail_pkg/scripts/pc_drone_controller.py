#!/usr/bin/env python3

# IMPORTS
import rospy
import roslib

from geometry_msgs.msg import Pose 
from my_custom_interfaces.msg import Drone_cmd
from std_msgs.msg import Float32

def main():
    rospy.init_node('pc_drone_controller', anonymous=False)
    command_pub=rospy.Publisher("command", Drone_cmd, queue_size=1) #maybe is better to use cmd_vel

    cmd=Drone_cmd()
    rate = rospy.Rate(20) # 20hz 

    while not rospy.is_shutdown():
        key = input("Enter command: ")

        #   w 
        # a s d -> Per muoversi a croce 
        #
        # q e -> Per girare yaw
        #
        # z x -> Per salire e scendere

        if(key=='w'):
            cmd.roll=cmd.roll+0.01
        elif(key=="a"):
            cmd.pitch=cmd.pitch+0.01
        elif(key=="s"):
            cmd.roll=cmd.roll-0.01
        elif(key=="d"):
            cmd.pitch=cmd.pitch-0.01
        elif(key=="q"):
            cmd.yaw=cmd.yaw+0.5
        elif(key=="e"):
            cmd.yaw=cmd.yaw-0.5
        elif(key=="z"):
            cmd.throttle=cmd.throttle+0.01
        elif(key=="x"):
            cmd.throttle=cmd.throttle-0.01
        else:
            cmd.roll=0
            cmd.pitch=0
            cmd.yaw=0
            cmd.throttle=0

        command_pub.publish(cmd)
        print("Current command:")
        print(cmd,"\n")
        rate.sleep()

if __name__ == "__main__":
    main()
