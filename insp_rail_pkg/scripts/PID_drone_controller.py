#!/usr/bin/env python3

# IMPORTS
import rospy
import roslib

from geometry_msgs.msg import Pose 
from my_custom_interfaces.msg import Drone_cmd
from std_msgs.msg import Float32
import math

import argparse
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# GLOBAL VARIABLES
x=float(0)
y=float(0)
angle=float(0)
ground_distance=float(0)
rail_detected=float(0)

rad_angle=float(0)
x_perp=float(0)

old_x=float(0)
old_y=float(0)
old_angle=float(0)
old_ground_distance=float(0)

P_gain_yaw=0.2
D_gain_yaw=0
I_gain_yaw=0

P_gain_throttle=0.1
D_gain_throttle=0
I_gain_throttle=0

P_gain_pitch=0.001
D_gain_pitch=0
I_gain_pitch=0

yaw_integral=0
throttle_integral=0
pitch_integral=0

altitude=4 # meters

# FUNCTIONs
def update_olds():
    global old_x,old_y,old_angle,old_ground_distance
    old_x=x 
    old_y=y
    old_angle=angle
    old_ground_distance=ground_distance

def update_integrals(yaw_e, throttle_e,pitch_e):
    global yaw_integral,throttle_integral,pitch_integral
    yaw_integral=yaw_integral+yaw_e
    throttle_integral=throttle_integral+throttle_e
    pitch_integral=pitch_integral+pitch_e

# SUBSCRIBERs CALLBACK
def callback_loc(pose):
    global x,y,angle,rail_detected,rad_angle,x_perp
    x=pose.position.x
    y=pose.position.y
    angle=pose.orientation.z
    rail_detected=pose.orientation.w
    im_width=pose.orientation.x
    im_height=pose.orientation.y

    x=im_width*x/1000
    y=im_height*y/1000

    rad_angle=math.radians(angle)

    if(angle==0):
        x_line=x
        y_line=0

    else:
        m=math.tan(-math.pi/2-rad_angle)
        x_line=m*(m*x-y)/(1+m*m)
        y_line=-(m*x-y)/(1+m*m)

    if(x_line>0):
        x_perp=math.sqrt(x_line*x_line+y_line*y_line)
    else:
        x_perp=-math.sqrt(x_line*x_line+y_line*y_line)

    x_perp=x_perp/im_width

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
    print("PID controller started!")

    while not rospy.is_shutdown():

        cmd.yaw = -P_gain_yaw*angle - D_gain_yaw*(angle-old_angle) -I_gain_yaw*yaw_integral # signs may be due to the inverted image of the simulation
        if(abs(cmd.yaw)>30): # MAX yaw DJI= 100 degree/s 
            cmd.yaw=30*(abs(cmd.yaw)/cmd.yaw)

        cmd.throttle = P_gain_throttle*(altitude - ground_distance) + D_gain_throttle*(ground_distance-old_ground_distance) + I_gain_throttle*throttle_integral
        if(abs(cmd.throttle)>4): # MAX throttle DJI= 4m/s
            cmd.throttle=4*(abs(cmd.throttle)/cmd.throttle)

        cmd.pitch =    P_gain_pitch*x + D_gain_pitch*(x-old_x) + I_gain_pitch*pitch_integral
        if(abs(cmd.pitch)>5): # MAX roll/pitch DJI= 15m/s 
            cmd.pitch=5*(abs(cmd.pitch)/cmd.pitch)

        #print("P part: ", -P_gain_yaw*angle,", D part: ",- D_gain_yaw*(angle-old_angle)) 
        update_olds()
        update_integrals(angle,(altitude-ground_distance),x)

        # speed management1
        #if(abs(x)<10 and abs(angle<5)):
        #    cmd.roll=1
        #elif(abs(x)<100 and abs(angle<20)):
        #    cmd.roll=0.5
        #else:
        #    cmd.roll=0

        # speed management2
        cmd.roll=max(1-abs(x)/100,0)+max(1-abs(angle)/20,0) # MAX =2+2=4  best for now 

        # speed management3
        #cmd.roll=max(2-abs(x)/50,0)*max(2-abs(angle)/10,0) # MAX =2*2=4          

        if(rail_detected==42): # It means rails not detected, so keep the drone still
            cmd.yaw = 0
            cmd.pitch = 0
            cmd.roll = 0

        command_pub.publish(cmd)

        #-----------------------PRINT-----------------------------------------------
        #print("\nrail detected: ",(rail_detected!=42)," x:",x,", y:",y,", angle:",angle,", ground distance:",ground_distance)
        #print("commands: ")
        #print("yaw: ", cmd.yaw)
        #print("pitch: ",cmd.pitch)
        #print("roll: ", cmd.roll)
        #print("throttle: ", cmd.throttle)
        #print("x_perp: ",x_perp)
         #----------------------CONTROL ERROR FILE-----------------------------------------
        file_txt=open(os.path.join(ROOT,"control_errors"), "a")
        text=("control errors and commands: \nAngle:\n" + str(angle) + "\nPerpendicular_distance:\n" + str(x_perp) + "\nAltitude\n" + str(ground_distance) + "\nYaw:\n" + str(cmd.yaw) + "\nPitch:\n" + str(cmd.pitch) + "\nRoll:\n" + str(cmd.roll) + "\nThrottle:\n" + str(cmd.throttle) +"\n\n")
        file_txt.write(text)
        file_txt.close()
        #-------------------------------------------------------------------------------

        rate.sleep()

if __name__ == "__main__":
    main()
