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
ground_distance=float(0)
rail_detected=float(0)

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

altitude=2 # meters

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
        #cmd.roll=max(2-abs(x)/50,0)+max(2-abs(angle)/10,0) # MAX =2+2=4  best for now 

        # speed management3
        #cmd.roll=max(2-abs(x)/50,0)*max(2-abs(angle)/10,0) # MAX =2*2=4          

        if(rail_detected==42): # It means rails not detected, so keep the drone still
            cmd.yaw = 0
            cmd.pitch = 0
            cmd.roll = 0

        command_pub.publish(cmd)
        print("rail detected: ",(rail_detected!=42)," x:",x,", y:",y,", angle:",angle,", ground distance:",ground_distance)
        print("commands: ")
        print("yaw: ", cmd.yaw)
        print("pitch: ",cmd.pitch)
        print("roll: ", cmd.roll)
        print("throttle: ", cmd.throttle)
        rate.sleep()

if __name__ == "__main__":
    main()
