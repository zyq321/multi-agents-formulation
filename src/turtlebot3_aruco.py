#!/usr/bin/env python

# Copyright (c) 2020, zyq_hit, Inc.
# All rights reserved.
# This is a simple example for formation

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import select as se
from numpy import *
import traceback
import sys, os
import rospy
import math
import tf
import cv2
import datetime
import time

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

from turtlebot.turtlebot_class import*    # import the turylrbot_class  
from turtlebot.turtlebot_visual import*


target_3D=[(-0.080287736758833828, -0.028566797723180576, 0.72583121061325073), 
           (-0.00090173848091813704, -0.028555051895741007, 0.72553277015686035), 
           (-0.00090181819149380462, 0.050802424787484327, 0.72559690475463867), 
           (-0.080294837593229251, 0.050823324169497162, 0.72589540481567383)]
           
aruco_size=0.12
au=camera_matrix[0,0]
av=camera_matrix[1,1]
u0=camera_matrix[0,2]
v0=camera_matrix[1,2]
ud=881.5
vd=559.0
zi=-0.011128040623720761

def getKey():                             # get keyboard value
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = se.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class aruco_robot(turtlebot):
    def __init__(self,agent_no,target=-1):
        super(aruco_robot,self).__init__(agent_no)
        self.aruco_angle=0
        self.aruco_center=matrix([ [0.0],      
                               [0.0],
                               [1.0]])
        self.m_ji=matrix([ [0.0],      
                           [0.0],
                           [1.0]])
        self.Gji=zeros(2)
        self.gji=matrix([ [0.0],      
                          [0.0]])
        self.velo_ji_target=0.0
        self.target=target
        self.show=zeros((height,width,3), uint8)

    def measurement(self):
        frame=self.rgb_image
        self.show=self.rgb_image.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        threshold(gray,50,255,THRESH_BINARY,gray)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250) 
        parameters =  aruco.DetectorParameters_create() 
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, 
                                                          aruco_dict, 
                                                         parameters=parameters) 
        
        if ids is None:
            return False
        # print ids
        
        for corner,no in zip(corners,ids):
             if no!=self.target:
                 continue
             
             self.aruco_center[0,0]= corner[0,:,0].mean()
             self.aruco_center[1,0]= corner[0,:,1].mean()
             success,rvec,tvec=cv2.solvePnP(array(target_3D),array(corner),camera_matrix,distCoeffD)
             self.aruco_angle=self.angular_vel_judge(-rvec[1][0])
             aruco.drawDetectedMarkers(self.show, [corner],no)
            #  u=corner[0,:,0].mean()
            #  v=corner[0,:,1].mean()
            #  print getxyz(u,v,self.depth_image[v,u],camera_param)
            
             return True
             
        return False

    def follow_leader(self):
        M_ji=dot(linalg.inv(camera_matrix),self.aruco_center)
        u_ji=self.aruco_center[0,0]
        v_ji=self.aruco_center[1,0]
        q_ji=M_ji[1,0]
        p_ji=M_ji[0,0]
        theta_ji=self.aruco_angle
        gij=matrix([ [0.0],   
                 [0.0]]) 
        delta_uv=matrix([ [0.0],      # pixel error
                      [0.0]])
        Gij=matrix([ [0.0,0.0],   
                  [0.0,0.0]]) 

        gij[0,0]=(au*math.sin(theta_ji)-(u0-u_ji)*math.cos(theta_ji))*q_ji
        gij[1,0]=-(v0-v_ji)*math.cos(theta_ji)*q_ji
        delta_uv[0,0]=u_ji-ud
        delta_uv[1,0]=v_ji-vd
        self.velo_ji_target=self.velo_ji_target+0.02*(-0.0001*self.velo_ji_target+1/zi*0.00002*delta_uv.T*gij)
        Gij[0,0]=1/zi*(u0-u_ji)*q_ji
        Gij[0,1]=au-(u0-u_ji)*p_ji
        Gij[1,0]=1/zi*(v0-v_ji)*q_ji
        Gij[1,1]=-(u0-u_ji)*p_ji
        control=0.5*Gij.I*(-0.1*delta_uv-1/zi*gij* self.velo_ji_target)
        print gij
        self.turtlebot_control(control[0,0],control[1,0])
        cv2.putText(self.show, "(u,v)=("+str(u_ji)+","+str(v_ji)+")", (int(u_ji)+200, int(v_ji)), font, 1.0, (0, 0, 255), 2)
        cv2.putText(self.show, "theta="+str(theta_ji),(int(u_ji)+200, int(v_ji)+30), font, 1.0, (0, 0, 255), 2)
        cv2.putText(self.show, "v_hat:"+str(self.velo_ji_target), (int(u_ji)+200,  int(v_ji)+60), font, 1.0, (0, 0, 255), 2)

if __name__=="__main__":                 # main function
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('turtlebot3_telep') #init a ros node
    Num=3
    agent=[]
    for i in range(0,Num):
        tb=aruco_robot(i,i-1)      #init a instance of turtlebot class                 
        agent.append(tb)
    ros_rate=rospy.Rate(10)             #set the cycle Time   
    time.sleep(1)
    n=0
    try:  
        while(1):  
            key = getKey()                          #get the keyboard value when input is ctrl+c then exit 
            if (key == 'q'):
                    break
            elif (key == 'c'):
                 n=1
            if(n==1):
                agent[0].turtlebot_control(0.1,-0.02)
            # agent[0].turtlebot_control(0.0,-0.00)
            for i in range(1,Num):
                if(agent[i].measurement()):
                    if(n==1):
                        agent[i].follow_leader()
                    imshow("image for agent_"+str(i),cv2.resize(agent[i].show,(480,270),interpolation=cv2.INTER_CUBIC))
            waitKey(1)          
     
    
        
    except Exception as e:
        print(e)
        

    finally:
        for i in range(0,Num):
            agent[i].turtlebot_control(0,0)


    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
