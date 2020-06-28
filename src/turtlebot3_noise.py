#!/usr/bin/env python

# Copyright (c) 2020, zyq_hit, Inc.
# All rights reserved.
# This is a simple example for formation

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

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

from turtlebot.turtlebot_class import*    # import the turylrbot_class  
from turtlebot.turtlebot_visual import*
from turtlebot.global_camera import*

# target_3D=[(-0.080287736758833828, -0.028566797723180576, 0.72583121061325073), 
#            (-0.00090173848091813704, -0.028555051895741007, 0.72553277015686035), 
#            (-0.00090181819149380462, 0.050802424787484327, 0.72559690475463867), 
#            (-0.080294837593229251, 0.050823324169497162, 0.72589540481567383)]
target_3D=[(-0.080287736758833828, -0.028566797723180576, 0.72583121061325073), 
           (-0.00090173848091813704, -0.028555051895741007, 0.72553277015686035), 
           (-0.00090181819149380462, 0.050802424787484327, 0.72559690475463867), 
           (-0.080294837593229251, 0.050823324169497162, 0.72589540481567383)]           
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

class noise_robot(turtlebot):
    def __init__(self,agent_no):
        super(noise_robot,self).__init__(agent_no)
        self.gps_sub=  rospy.Subscriber('/tb3_'+str(self.No)+'/sensor_msgs/NavSatFix', NavSatFix, self.gps_callback,queue_size=1) 
        self.gps_info=[0,0]
        self.EARTH_RADIUS = 6378137.0 
        self.POLE_RADIUS  = 6356725.0
        self.init_lat_=49.89999999973975
        self.init_lon_=8.899999999962901
        self.M_PI=3.1415926 
    def gps_callback(self,data):
        self.gps_info=[data.latitude,data.longitude]
    def get_gps_positon(self):
        x=self.EARTH_RADIUS * (self.gps_info[0] - self.init_lat_) * self.M_PI / 180.0
        y=self.EARTH_RADIUS * cos(self.init_lat_ * self.M_PI / 180.0) * (self.gps_info[1] - self.init_lon_) * self.M_PI / 180.0
        return [x,y]

if __name__=="__main__":                 # main function
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('turtlebot3_telep') #init a ros node
    Num=3
    tb=noise_robot(0)                    #init a instance of turtlebot class                 
    ros_rate=rospy.Rate(50)             #set the cycle Time   
    x=[]
    y=[]
    t=[]
    i=0
    fig=plt.figure()
    ax=fig.add_subplot(1,1,1)
    camera=global_camera()
    try:  
        while(1):  
            result=camera.get_positon_matrix()
            imshow("1234", camera.frame)

            waitKey(1)          
            # ros_rate.sleep()
            # position=tb.get_gps_positon()
            # x.append(position[0])
            # y.append(position[1])
            # t.append(i)
            # i=i+1
            # plt.cla()
            # ax.plot(t,x,'-g')
            # plt.pause(0.00001)
            key = getKey() #get the keyboard value when input is ctrl+c then exit 
            if (key == '\x03'):
                break
        

       
    except Exception as e:
        print(e)
        

    finally:
        for i in range(0,Num):
            tb.turtlebot_control(0,0)


    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
