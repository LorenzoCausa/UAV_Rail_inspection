# UAV Rail Inspection
Method for the automated inspection of a rail section. 

**important:** The repo does not contains the weights

## How to install it
This are two ROS packages (ROS noetic), you can install them just like a normal ros package. 

Put this repo in your ros work space and run  `catkin_make ` or  `catkin build `

## How to build your model
To train your model, and download the correspondant weights you can use my colab notebook:

<https://colab.research.google.com/drive/1cuWjSNl8cpsKzjk40bJD1z3yOXSI98_v?usp=sharing>

## Test
All the test were carried out with Mavic2 enterprise dual UAV and in simulation ( with CoppeliaSim ).

## Info
This repository contains Robotics Operating Systems (ROS) packages for the algorithm implementation and uses Detectron2 for rail detection.

 - Detectron2: <https://github.com/facebookresearch/detectron2> (released under the Apache 2.0 license)
 - ROS: <https://www.ros.org/>
 
The repo also contains scripts for data augmentation operations and video generation from frame collections.
 
## Note
This repo is part of my master thesis project, for more info see the correspondant chapter of the written paper.

LINK ALLA TESI

## Other thesis repos

This repo is to be used with the rest of the thesis project

 - The coppelia simulation: <https://github.com/LorenzoCausa/coppelia_rail_simulation> 
 - he bridge application between Linux computer and the drone controller: <https://github.com/LorenzoCausa/bridge_DJI_ROS>
