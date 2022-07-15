#!/bin/bash

xterm -e bash -c "roslaunch insp_rail_pkg view_uav_image.launch ; exec bash" &
sleep 2
xterm -e bash -c "rosrun insp_rail_pkg img_listener_UDP.py ; exec bash" &
xterm -e bash -c "rosrun insp_rail_pkg UDP_server.py ; exec bash" &
xterm -e bash -c "rosrun insp_rail_pkg command_publisher.py ; exec bash" &
xterm -e bash -c "rosrun insp_rail_pkg cmd_vel_publisher.py ; exec bash" &
echo "script finished"
