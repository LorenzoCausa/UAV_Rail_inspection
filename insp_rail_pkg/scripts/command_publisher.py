#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import socket
import json
import time
import os
# ROS
import rospy 
from geometry_msgs.msg import Point
from my_custom_interfaces.msg import Drone_cmd


class send_cmd_socket():
    FORMAT = "utf-8"
    DISCONNECT_MESSAGE = "!DISCONNECT"

    def __init__(self, ip_server, port):

        rospy.init_node('send_target_socket')
        self.PORT = port
        self.SERVER =ip_server  # IP del server
        self.ADDR = (self.SERVER, self.PORT)
        # mettere try 
        path = os.path.dirname(os.path.abspath(__file__))+"/cmd.JSON"
        print(path)
        self.data = json.load(open(path))
        print("Started")

        self.sub = rospy.Subscriber("/cmd_vel", Drone_cmd, self.send_cmd_to_app, queue_size=1)

    def send_msg(self, msg, client):  # FUNZIONE DA ELIMINARE
        message = msg.encode(self.FORMAT)  # codificarlo in binario per la trasmissione
        client.send(message)  # mando msg
        # print(client.recv(2048).decode(FORMAT))# decodifico risposta e la printo
        client.close()

    
    def send_cmd_to_app(self, cmd):
        self.data["yaw"] = cmd.yaw
        self.data["pitch"] = cmd.pitch
        self.data["roll"] = cmd.roll
        self.data["throttle"] = cmd.throttle

        msg = json.dumps(self.data)
        #print("before sendig")
        client = socket.socket(
        socket.AF_INET, socket.SOCK_STREAM)  # creo il client
        client.connect(self.ADDR)  # indirizzo del server a cui devo connettermi
        #print("Sending the jason msg...")
        message = msg.encode(self.FORMAT)  # codificarlo in binario per la trasmissione
        client.send(message)  # mando msg
        #print("jason msg sent")
        # print(client.recv(2048).decode(FORMAT))# decodifico risposta e la printo
        client.close()
        #self.send_msg(msg, client)


if __name__ == "__main__":
    
    ip_server = input("Enter IP of your mobile device: ")
    port = 8080
    client = send_cmd_socket(ip_server, port)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")