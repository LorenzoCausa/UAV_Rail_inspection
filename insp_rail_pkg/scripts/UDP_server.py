#!/usr/bin/env python3

import sys, time
import numpy as np
import cv2
import roslib
import rospy
from std_msgs.msg import Float32
import socket 
import os
import struct
FORMAT = "utf-8"      

def sub_server(localIP,localPort):
    # Create a datagram socket
    UDPServerSocket1 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPServerSocket2 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    # Bind to address and ip
    UDPServerSocket1.bind((localIP,localPort))
    UDPServerSocket2.bind((localIP,localPort+1))
    print("UDP servers up and listening")
    # Listen for incoming datagrams
    while not rospy.is_shutdown():
        bytesAddressPair1 = UDPServerSocket1.recvfrom(8192)
        message1 = bytesAddressPair1[0]
        bytesAddressPair2 = UDPServerSocket2.recvfrom(8192)
        message2 = bytesAddressPair2[0]
        [altitude] = struct.unpack('>f',message1)
        [barometer_altitude] = struct.unpack('>f',message2)
        altitude_pub.publish(altitude)
        barometer_altitude_pub.publish(barometer_altitude)
        print("ground distance: " , altitude)
        print("altitude from take off point: ",barometer_altitude)

if __name__ == "__main__":
    '''
        initialize the node and the publisher
    '''

    rospy.init_node('UDP_server')
    
	## getting the IP address 
    ip_add = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]
    print("IP Address: ", ip_add)
    altitude_pub = rospy.Publisher("/ground_distance", Float32, queue_size=1)
    barometer_altitude_pub = rospy.Publisher("/barometer_altitude", Float32, queue_size=1)
    
    #creating the server at the specified ip and port
    sub_server(ip_add,8081)

    #uncomment this if the auto-recognition of the Ip doesn't work
    #sub_server(("192.168.1.195",8888)) 
