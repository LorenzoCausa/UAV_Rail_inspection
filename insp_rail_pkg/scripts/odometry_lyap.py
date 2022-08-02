#!/usr/bin/env python3

# IMPORTS
import math
import rospy
import roslib

from geometry_msgs.msg import Pose 
from my_custom_interfaces.msg import Drone_cmd
from std_msgs.msg import Float32

# GLOBAL VARIABLES
x=float(0)
y=float(0)
x_perp=float(0)
angle=float(0)
ground_distance=float(1)
rail_detected=float(0)

old_x=float(0)
old_y=float(0)
old_angle=float(0)
old_ground_distance=float(0)

P_gain_yaw=0.2
D_gain_yaw=0#0.5
I_gain_yaw=0#0.0001

P_gain_throttle=0.5
D_gain_throttle=0#0.1
I_gain_throttle=0#0.0001

P_gain_pitch=0.002
D_gain_pitch=0#0.002
I_gain_pitch=0#0.00001

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
    global x,y,angle,rail_detected,x_perp
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
    rospy.init_node('odometry_lyapunov_drone_controller', anonymous=False)
    rospy.Subscriber("localization", Pose, callback_loc,queue_size=1)
    rospy.Subscriber("ground_distance", Float32, callback_ground,queue_size=1)
    command_pub=rospy.Publisher("command", Drone_cmd, queue_size=1) #maybe is better to use cmd_vel

    cmd=Drone_cmd()
    f=20
    rate = rospy.Rate(f) # 20hz 

    while not rospy.is_shutdown():

        global angle,x_perp
        rad_angle=math.radians(angle)
        
        V_x=0.3             # NOTA IMPORTANTE: Puoi scegliere qualsiasi V_x
        V_y=0.5*x_perp

        cmd.roll=V_x*math.cos(rad_angle)+V_y*math.sin(rad_angle)
        cmd.pitch=-V_x*math.sin(rad_angle)+V_y*math.cos(rad_angle)
        cmd.yaw= -0.1*angle
        cmd.throttle = P_gain_throttle*(altitude - ground_distance) + D_gain_throttle*(ground_distance-old_ground_distance) + I_gain_throttle*throttle_integral
        
        if(abs(cmd.throttle)>4): # MAX throttle DJI= 4m/s
            cmd.throttle=4*(abs(cmd.throttle)/cmd.throttle)

        if(abs(cmd.yaw)>30): # MAX yaw DJI= 100 degree/s 
            cmd.yaw=30*(abs(cmd.yaw)/cmd.yaw)
        
        if(abs(cmd.roll)>5): # MAX roll/pitch DJI= 15m/s 
            cmd.roll=5*(abs(cmd.roll)/cmd.roll)

        if(abs(cmd.pitch)>5): # MAX roll/pitch DJI= 15m/s 
            cmd.pitch=5*(abs(cmd.pitch)/cmd.pitch)
        

        if(rail_detected==42): # It means rails not detected, so keep the drone still
            cmd.yaw = 0
            cmd.pitch = 0
            cmd.roll = 0

        command_pub.publish(cmd)
        print("\nrail detected: ",(rail_detected!=42)," x:",x,", y:",y,", x_perp:",x_perp,", angle:",angle,", ground distance:",ground_distance)
        print("commands: ")
        print("yaw: ", cmd.yaw)
        print("pitch: ",cmd.pitch)
        print("roll: ", cmd.roll)
        print("throttle: ", cmd.throttle)

        # ODOMETRY
        angle=angle+cmd.yaw/f
        if(ground_distance>0.1):
            x_perp=x_perp-(V_y/f)*0.5/ground_distance

        rate.sleep()

if __name__ == "__main__":
    main()
