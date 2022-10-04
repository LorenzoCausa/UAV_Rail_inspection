#!/usr/bin/env python3

# IMPORTS
import rospy
import roslib
import numpy as np
import cv2
import time
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as SensImage
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float32 

import argparse
import os
import sys

from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# CALLBACKS
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

    #----------------------CONTROL ERROR FILE-----------------------------------------
    file_txt=open(os.path.join(ROOT,"real_control_errors"), "a")
    text=("Real control errors and commands: \nAngle:\n" + str(angle) + "\nPerpendicular_distance:\n" + str(x_perp)  +"\n\n")
    file_txt.write(text)
    file_txt.close()
    #-------------------------------------------------------------------------------

def main():
    global pub_image,pub_perp_d
    rospy.init_node('real_errors', anonymous=False)
    
    rospy.Subscriber("localization", Pose, callback_loc,queue_size=1)
    

    print("started")

    rospy.spin()

if __name__ == "__main__":
    main()
