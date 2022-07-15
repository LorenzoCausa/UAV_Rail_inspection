#!/usr/bin/env python3

# IMPORTS
import rospy
import roslib

from geometry_msgs.msg import Pose 
from my_custom_interfaces.msg import Drone_cmd
from std_msgs.msg import Float32

# GLOBAL VARIABLES
x=float(0)
y=float(0)
angle=float(0)
ground_distance=float(3)
rail_detected=float(0)

old_x=float(0)
old_y=float(0)
old_angle=float(0)
old_ground_distance=float(3)

P_gain_yaw=1
D_gain_yaw=20

P_gain_throttle=1
D_gain_throttle=10

P_gain_pitch=0.001
D_gain_pitch=0.005


altitude=3 # meters

# FUNCTIONs
def update_olds():
    global old_x,old_y,old_angle,old_ground_distance
    old_x=x 
    old_y=y
    old_angle=angle
    old_ground_distance=ground_distance

# SUBSCRIBERs CALLBACK
def callback_loc(pose):
    global x,y,angle,rail_detected
    x=pose.position.x
    y=pose.position.y
    angle=pose.orientation.z
    rail_detected=pose.orientation.w

def callback_ground(distance):
    global ground_distance
    ground_distance=distance.data

def main():
    rospy.init_node('drone_controller', anonymous=False)
    rospy.Subscriber("localization", Pose, callback_loc,queue_size=1)
    rospy.Subscriber("ground_distance", Float32, callback_ground,queue_size=1)
    command_pub=rospy.Publisher("command", Drone_cmd, queue_size=1) #maybe is better to use cmd_vel

    cmd=Drone_cmd()
    rate = rospy.Rate(20) # 20hz 

    while not rospy.is_shutdown():

        cmd.yaw = -P_gain_yaw*angle - D_gain_yaw*(angle-old_angle) # signs may be due to the inverted image of the simulation
        if(abs(cmd.yaw)>30): # MAX yaw DJI= 100 degree/s 
            cmd.yaw=30*(abs(cmd.yaw)/cmd.yaw)

        cmd.throttle = P_gain_throttle*(altitude - ground_distance) - D_gain_throttle*(ground_distance-old_ground_distance)
        if(abs(cmd.throttle)>4): # MAX throttle DJI= 4m/s
            cmd.throttle=4*(abs(cmd.throttle)/cmd.throttle)

        cmd.pitch =    P_gain_pitch*x - D_gain_pitch*(x-old_x)
        if(abs(cmd.pitch)>5): # MAX roll/pitch DJI= 15m/s 
            cmd.pitch=5*(abs(cmd.pitch)/cmd.pitch)

        #print("P part: ", -P_gain_yaw*angle,", D part: ",- D_gain_yaw*(angle-old_angle)) 
        update_olds()

        # speed management1
        #if(abs(x)<10 and abs(angle<5)):
        #    cmd.roll=1
        #elif(abs(x)<100 and abs(angle<20)):
        #    cmd.roll=0.5
        #else:
        #    cmd.roll=0

        # speed management2
        cmd.roll=max(2-abs(x)/50,0)+max(2-abs(angle)/10,0) # MAX =2+2=4  best for now 

        # speed management3
        #cmd.roll=max(2-abs(x)/50,0)*max(2-abs(angle)/10,0) # MAX =2*2=4          

        if(rail_detected==42): # It means rails not detected, so keep the drone still
            cmd.yaw = 0
            cmd.pitch = 0
            cmd.roll = 0

        command_pub.publish(cmd)
        print("rail detected: ",(rail_detected!=42)," x:",x,", y:",y,", angle:",angle,", ground distance:",ground_distance)
        rate.sleep()

if __name__ == "__main__":
    main()
