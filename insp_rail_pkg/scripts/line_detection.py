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


from PIL import Image 

#GLOBAL VARIABLES
cv_image=None
x=float(0)
y=float(0)
angle=float(0)
ground_distance=float(0)
rail_detected=float(0)

rad_angle=float(0)
x_perp=float(0)
pub_image=None
pub_perp_d=None

# SUBSCRIBERs CALLBACK
def callback(startImg):
    global cv_image
    cv_bridge=CvBridge()
    cv_image = cv_bridge.imgmsg_to_cv2(startImg, 'bgr8')
    #print("image receveid")

def callback_loc(pose):
    global x,y,angle,rail_detected,rad_angle,x_perp,cv_image
    im_height,im_width,_=cv_image.shape

    x=pose.position.x
    y=pose.position.y
    angle=pose.orientation.z
    rail_detected=pose.orientation.w
    m=100000

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

    cv_bridge=CvBridge()

    cv_image = cv2.circle(cv_image, (int(im_width/2), int(im_height/2)), radius=10, color=(255, 255, 0), thickness=-1)

    x_line_image=(x_line)+0.5*im_width
    y_line_image=(y_line)+0.5*im_height
    cv_image = cv2.circle(cv_image, (int(x_line_image), int(y_line_image)), radius=10, color=(255, 255, 0), thickness=-1)

    cv_image=cv2.line(cv_image, (int(x_line_image),int(y_line_image)), (int(im_width/2),int(im_height/2)), (255,255,0),4)

    box_center_x=(x)+0.5*im_width
    box_center_y=(y)+0.5*im_height
    cv_image = cv2.circle(cv_image, (int(box_center_x), int(box_center_y)), radius=10, color=(255, 255, 0), thickness=-1)

    C=box_center_y-m*box_center_x
    y2_image=0
    y3_image=im_height

    if(abs(m)>0.00000001):
        x2_image=(y2_image-C)/m
        x3_image=(y3_image-C)/m
    else:
        x2_image=-1000000
        x3_image=1000000

    label = "x perp: " + str(x_perp)
    cv_image=cv2.line(cv_image, (int(x2_image),int(y2_image)), (int(x3_image),int(y3_image)), (255,255,0),4)
    cv2.putText(cv_image, label, (int(im_width/2), int(im_height/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,0,0), 1, cv2.LINE_AA) 



    #print("x_line:",x_line," y_line:",y_line," x:",x," y:",y," m:",m," x_line_image:",x_line_image," y_line_image:",y_line_image)

    if(x_line!=0 and y_line!=0):
        print("perpendicolarita:",(-y_line/x_line)*m)

    pub_image.publish(cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    pub_perp_d.publish(100*x_perp/im_width)

def main():
    global pub_image,pub_perp_d
    rospy.init_node('line_detection', anonymous=False)
    
    rospy.Subscriber('boxes_and_mask', SensImage,callback, queue_size=1)
    rospy.Subscriber("localization", Pose, callback_loc,queue_size=1)
    pub_image = rospy.Publisher('line_to_follow', SensImage, queue_size=1)
    pub_perp_d=rospy.Publisher('perpendicular_distance', Float32, queue_size=1)

    print("started")

    rospy.spin()

if __name__ == "__main__":
    main()
