#!/usr/bin/env python3

import sys, time
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
import socket 
import os
import struct
FORMAT = "utf-8"      

def UDP_server(localIP,localPort): 
    # Create UDP server socket
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPServerSocket.bind((localIP,localPort))
    print("images UDP server up and listening")
        
    count = 0
    while not rospy.is_shutdown():
        bytesAddressPair = UDPServerSocket.recvfrom(65536)
        data = bytesAddressPair[0]
        #address = bytesAddressPair[1]
        count = count+1   
        #print('last image dimension: ',sys.getsizeof(data))
        #print("received: ", count," images")     
        img = CompressedImage()
        img.data = data
        img.format = "jpg"
        image_pub.publish(img)
        if(count%10==0):
            print("received: ", count," images")
        

if __name__ == "__main__":
    rospy.init_node('image_publisher')
    image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)

	# getting the IP address 
    ip_add = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]
    print("IP Address: ", ip_add)
    
    #creating the server at the specified ip and port
    UDP_server(ip_add,8888)

    #uncomment this if the auto-recognition of the Ip doesn't work
    #sub_server(("192.168.1.195",8888)) 
