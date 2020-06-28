#!/usr/bin/env python

# Copyright (c) 2020, zyq_hit, Inc.
# All rights reserved.
# This is a sample example for DKF (Distributed Kalman Filter)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import select as se
from numpy import *
import traceback
import sys, os
import rospy
import math
import tf
import cv2
import datetime
import matplotlib.pyplot as plt
import time
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

from turtlebot.turtlebot_class import*   
from turtlebot.turtlebot_visual import*
from turtlebot.global_camera import*
x_init=[2.0,0]
target_3D=[(-0.080287736758833828, -0.028566797723180576, 0.72583121061325073), 
           (-0.00090173848091813704, -0.028555051895741007, 0.72553277015686035), 
           (-0.00090181819149380462, 0.050802424787484327, 0.72559690475463867), 
           (-0.080294837593229251, 0.050823324169497162, 0.72589540481567383)]       
lower_red = array([100, 43,46])
upper_red = array([180, 255, 255])

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

class optimal_robot(turtlebot):
    def __init__(self,agent_no):
        super(optimal_robot,self).__init__(agent_no)
        std=0.02
        run_t=2000
        self.measurement=[0,0]
        self.x_noise=std*random.standard_normal(size=run_t)
        self.y_noise=std*random.standard_normal(size=run_t)

    def DFK(self,target_velo):





    def camera_measurement(self,run_t):
        self.frame=self.rgb_image.copy()
        gray = cvtColor(self.frame, COLOR_BGR2GRAY)
        mask =cv2.inRange(gray, array([250]), array([255]))
        result,contours,hierarchy = cv2.findContours(mask , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours)==1:
            for cnt in contours:
                (x, y, w, h) = cv2.boundingRect(cnt)
                u=x+w/2
                v=y+h/2  
                x,y,z=getxyz(u,v,self.depth_image[v][u],camera_param)
        # threshold(gray,5,255,THRESH_BINARY,gray)
        # aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250) 
        # parameters =  aruco.DetectorParameters_create() 
        # corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, 
        #                                                   aruco_dict, 
        #                                                   parameters=parameters) 
        # imshow("123",gray)
        # if ids is not None:

        #     rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.20, camera_matrix, dist) 
        #     (rvec-tvec).any()
                y=z+0.064
                x=x+0.065
                cos_angle=math.cos(self.angle)
                sin_angle=math.sin(self.angle)
                tran_matrix=matrix([ 
                    [sin_angle,cos_angle],      # Adjacent Matrix
                    [-cos_angle,sin_angle]
                    ])
                r=dot(tran_matrix,matrix([[x],
                                      [y]]))
                
                x=r[0,0]+self.position[0,0]+self.x_noise[run_t]
                y=r[1,0]+self.position[1,0]+self.y_noise[run_t]
                self.measurement[0]=x
                self.measurement[1]=y
                return self.measurement
        
        else :
            return None

class target_robot(object):
    def __init__(self,agent_no):
        self.position=zeros((2,1))
        self.velo=zeros((2,1))
        self.No=agent_no
        self.pub = rospy.Publisher('/tb3_'+str(self.No)+'/cmd_vel', Twist, queue_size=10)
        self.sub=  rospy.Subscriber('/tb3_'+str(self.No)+'/odom', Odometry, self.turtlebot_callback,queue_size=1) 
   
    def turtlebot_callback(self,data):
        Agent_Quaternionn=data.pose.pose.orientation
        self.position[0,0]=data.pose.pose.position.x
        self.position[1,0]=data.pose.pose.position.y
        self.velo[0,0]=data.twist.twist.linear.x
        self.velo[1,0]=data.twist.twist.linear.y
   
    def turtlebot_control(self,linear_x,linear_y):
            twist = Twist()
            control_linear_x =checkLinearLimitVelocity(linear_x)
            control_linear_y =checkLinearLimitVelocity(linear_y)
            twist.linear.x = control_linear_x; twist.linear.y = control_linear_y; twist.linear.z = 0.0
            control_angular_vel =checkAngularLimitVelocity(0)
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0
            self.pub.publish(twist)



if __name__=="__main__":                 # main function
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('turtlebot3_telep') 
    Num=3                            
    ros_rate=rospy.Rate(10)             #set the cycle Time   
    x=[x_init[0]]
    y=[x_init[1]]
    x1=[x_init[0]]
    y1=[x_init[1]]
    fig=plt.figure()
    ax=fig.add_subplot(1,1,1)
    camera=global_camera()
    target_R=target_robot(0)
    tb=optimal_robot(1)
    time.sleep(1)
    run_t=0
    try:  
        begin=rospy.get_time()
        while(1):  
            print(rospy.get_time()-begin)
            vel_x=-0.1*target_R.position[1][0]+0.075
            vel_y=0.1*target_R.position[0][0]-0.20
            x.append(target_R.position[0][0])
            y.append(target_R.position[1][0])
            target_R.turtlebot_control(vel_x,vel_y) 
            # target_R.turtlebot_control(0,0)
            r= tb.camera_measurement(run_t)
            if(r !=None):
                x1.append(r[0])
                y1.append(r[1])
            # plt.cla()
            # plt.xlim([0.5,3.5])
            # plt.ylim([-1,2])
            # ax.plot(x,y,'-g')
            # ax.plot(x1,y1,'-r')
            # plt.pause(0.00001)
  
            run_t+=1
            key = getKey() #get the keyboard value when input is ctrl+c then exit 
            if (key == 'q'):
                target_R.turtlebot_control(0,0)
                break
            waitKey(1) 
            ros_rate.sleep()
       
    except Exception as e:
        target_R.turtlebot_control(0,0)
        print(e)
        

    finally:
        
        target_R.turtlebot_control(0,0)


    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
