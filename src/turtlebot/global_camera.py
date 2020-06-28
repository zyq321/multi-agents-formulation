#!/usr/bin/env python
# Copyright (c) 2020, zyq-hit.
# All rights reserved.

import rospy
import tf
from geometry_msgs.msg import Twist
import sys, os
import select as se
from nav_msgs.msg import Odometry
from numpy import *
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
from cv2 import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
  

height=1080
width=1920


distCoeffD=array([0.0,0.0,0.0,0.0,0.0])
camera_param=[1.0,540.5,960.5,1206.89,1206.89]
camera_matrix=array([ [1206.8897, 0.0,        960.5],      # camera Matrix
                       [0.0,       1206.8897,  540.5],            
                       [0.0,       0.0,         1.0]]) 
dist = array( [0.0, 0.0, 0.0, 0.0, 0.0] )


# target_3D=[(-0.080287736758833828, -0.028566797723180576, 0.72583121061325073), 
#            (-0.00090173848091813704, -0.028555051895741007, 0.72553277015686035), 
#            (-0.00090181819149380462, 0.050802424787484327, 0.72559690475463867), 
#            (-0.080294837593229251, 0.050823324169497162, 0.72589540481567383)] 
target_3D=[(1.0, -1.0, 1.0),
            (1.0, 1.0, 1.0),
            (-1.0, 1.0, 1.0),
            (-1.0,-1.0, 1.0)] 
class global_camera(object):
    def __init__(self):
        
        self.image_sub= rospy.Subscriber('global_camera/camera/rgb/image_raw',Image,self.image_callback,queue_size=1)
        self.depth_sub= rospy.Subscriber('global_camera/camera/depth/image_raw',Image,self.depth_callback,queue_size=1)
        self.bridge = CvBridge()
        self.rgb_image=zeros((height,width,3), uint8)
        self.depth_image=zeros((height,width,1), float32)

    def image_callback(self,data):
             self.rgb_image=cvtColor(self.bridge.imgmsg_to_cv2(data, "rgb8"),COLOR_BGR2RGB) 
    
    def depth_callback(self,data):
             self.depth_image =self.bridge.imgmsg_to_cv2(data, "32FC1")

    def get_positon_matrix(self):
      self.frame=self.rgb_image.copy()
      gray = cvtColor(self.frame, COLOR_BGR2GRAY)  
      threshold(gray,50,255,THRESH_BINARY,gray)
      imshow("123123",gray)
      aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250) 
      parameters =  aruco.DetectorParameters_create() 
      corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, 
                                                          aruco_dict, 
                                                          parameters=parameters) 
      robot_list=[]
      if ids is not None:
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.35, camera_matrix, dist) 
        (rvec-tvec).any()
        robot_n=len(ids)
        res=zeros([robot_n,4])
        for i in range(len(ids)):
          n=ids[i][0]
          tvec[i][0][1]=-tvec[i][0][1]
          res[n][0:3]=tvec[i][0]
          success,rvec1,tvec1=solvePnP(array(target_3D),corners[i],camera_matrix,distCoeffD)
          res[n][3]=-rvec1[2][0]
          # print(rvec1)
        print(res)
        return res

      else :
        return None