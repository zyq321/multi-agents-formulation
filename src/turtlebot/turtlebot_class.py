#!/usr/bin/env python
# Copyright (c) 2019, zyq-hit.
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
  

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1
turtlebot3_model="waffle"

class turtlebot:
    def __init__(self,agent_no):
        
        self.position=zeros((2,1))
        self.angle=0
        self.No=agent_no
        self.pub = rospy.Publisher('/tb3_'+str(self.No)+'/cmd_vel', Twist, queue_size=10)
        self.sub=  rospy.Subscriber('/tb3_'+str(self.No)+'/odom', Odometry, self.turtlebot_callback,queue_size=1) 

    def turtlebot_callback(self,data):
            Agent_Quaternionn=data.pose.pose.orientation
            self.position[0,0]=data.pose.pose.position.x
            self.position[1,0]=data.pose.pose.position.y
            self.angle=tf.transformations.euler_from_quaternion([Agent_Quaternionn.x,Agent_Quaternionn.y,Agent_Quaternionn.z,Agent_Quaternionn.w])[2]
   
    def turtlebot_control(self,linear_vel,angle_vel):
            twist = Twist()
            control_linear_vel =checkLinearLimitVelocity(linear_vel)
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
            control_angular_vel =checkAngularLimitVelocity(angle_vel)
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            self.pub.publish(twist)
   
    def angular_vel_judge(angual_vel):
        if(abs(angual_vel)>3.14):
            return (-1*sign(angual_vel)*(6.28-abs(angual_vel)))    
        else:
            return angual_vel



def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input
    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    return vel

def cal_Laplace(Adj_mat):
    Lap_mat=Adj_mat.copy()
    Lap_mat=-Lap_mat
    row=Adj_mat.shape[0]
    col=Adj_mat.shape[1]
    for i in range(0,row):
        for j in range(0,row):
            if(i != j):
                Lap_mat[i,i]+=Adj_mat[i,j]
    return -Lap_mat
