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
import numpy.matlib as matlib
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

from turtlebot.turtlebot_class import*   
from turtlebot.turtlebot_visual import*
loc='left'
font_dict={'fontsize': 14,\
         'fontweight' : 8.2,\
         'verticalalignment': 'baseline',\
         'horizontalalignment': loc}

x_init=[0.0,0.0]
target_3D=[(-0.080287736758833828, -0.028566797723180576, 0.72583121061325073), 
           (-0.00090173848091813704, -0.028555051895741007, 0.72553277015686035), 
           (-0.00090181819149380462, 0.050802424787484327, 0.72559690475463867), 
           (-0.080294837593229251, 0.050823324169497162, 0.72589540481567383)]       
lower_red = array([100, 43,46])
upper_red = array([180, 255, 255])
Is_measure=[0,0,0,0]
m_color=['#FF0000','#0000FF','#CC9900','#66CC00']
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
        m_std=0.15
        s_std=0.1  
        run_t=2000

        self.measurement=matrix([ 
                    [0.0],      # Adjacent Matrix
                    [0.0]
                    ])
        self.x_m_noise=m_std*random.normal(0,m_std,size=run_t)
        self.y_m_noise=m_std*random.normal(0,m_std,size=run_t)

        self.p_check= 1.0 * matlib.eye(2)
        self.p_hat= 1.0 * matlib.eye(2)
        self.s_check=matrix([[x_init[0]],
                             [x_init[1]]])
        self.s_hat=matrix([[x_init[0]],
                             [x_init[1]]])
        self.Q=0.15* s_std**2*matlib.eye(2)
        self.R=m_std**2*matlib.eye(2)
        self.R_inv=linalg.inv(self.R)
        self.K=matlib.eye(2)
        self.m_x_plot=[]
        self.m_y_plot=[]
        self.s_x_plot=[]
        self.s_y_plot=[]
        self.is_visual=[]
         

    def state_inter(self,target_velo,run_t):
        self.s_check=self.s_hat+target_velo*0.2
        self.p_check=self.p_hat+self.Q

    def DKF(self,agent):
        first_s_term=matlib.zeros((2,1))
        second_s_term=matlib.zeros((2,1))
        first_p_term=0.001*matlib.eye(2)
        second_p_term=0.001*matlib.eye(2)
        for i in range(4):
            if Is_measure[i]==0 :
                continue
            elif i==self.No-1:
                first_p_term+=agent[i].R_inv
                first_s_term+=0.15*dot(agent[i].R_inv,(agent[i].measurement-self.s_check))
            else:
                first_p_term+=agent[i].R_inv
                first_s_term+=0.15*dot(agent[i].R_inv,(agent[i].measurement-self.s_check))
                second_p_term+=linalg.inv(agent[i].p_check)
                second_s_term+=0.01*dot(linalg.inv(agent[i].p_check),(self.s_check-agent[i].s_check))
        self.s_hat=self.s_check+dot(self.p_check,first_s_term)+dot(self.p_check,second_s_term)
        self.p_hat=linalg.inv(self.p_check)+first_p_term+second_p_term
        self.p_hat=linalg.inv(self.p_hat)
        self.s_x_plot.append(self.s_hat[0,0]+random.normal(0,0.0005,size=1))
        self.s_y_plot.append(self.s_hat[1,0]+random.normal(0,0.0005,size=1))
        return self.s_hat

    def KF(self,agent):
        first_term=linalg.inv(self.p_check+self.R)
        self.K =dot(self.p_check,first_term)
        self.s_hat=self.s_check+dot( self.K,(self.measurement-self.s_check) )
        self.p_hat=self.p_check-dot(self.K,self.p_check)
        self.s_x_plot.append(self.s_check[0,0])
        self.s_y_plot.append(self.s_check[1,0])
        print(self.s_hat)
        print(self.s_check)
        return self.s_hat

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
                # print(x,y)
                r=matrix([[0.0],
                          [0.0]])
                r[0,0]=x
                r[1,0]=y
                r=dot(tran_matrix,r)
                
                x=r[0,0]+self.position[0,0]+self.x_m_noise[run_t]
                y=r[1,0]+self.position[1,0]+self.y_m_noise[run_t]
                self.measurement[0,0]=x
                self.measurement[1,0]=y
                Is_measure[self.No-1]=1
                self.is_visual.append(1+(self.No-1)*2)
                self.m_x_plot.append(self.measurement[0,0])
                self.m_y_plot.append(self.measurement[1,0])
                return self.measurement
        else :
            Is_measure[self.No-1]=0
            self.is_visual.append((self.No-1)*2)
            return None




class target_robot(object):
    def __init__(self,agent_no):
        self.position=zeros((2,1))
        self.velo=zeros((2,1))
        self.angle=0.0
        self.No=agent_no
        self.pub = rospy.Publisher('/tb3_'+str(self.No)+'/cmd_vel', Twist, queue_size=10)
        self.sub=  rospy.Subscriber('/tb3_'+str(self.No)+'/odom', Odometry, self.turtlebot_callback,queue_size=1) 
   
    def turtlebot_callback(self,data):
        Agent_Quaternionn=data.pose.pose.orientation
        self.position[0,0]=data.pose.pose.position.x
        self.position[1,0]=data.pose.pose.position.y
        self.angle=tf.transformations.euler_from_quaternion([Agent_Quaternionn.x,Agent_Quaternionn.y,Agent_Quaternionn.z,Agent_Quaternionn.w])[2]
        self.velo[0,0]=0.075*math.cos(self.angle)
        self.velo[1,0]=0.075*math.sin(self.angle)
   
    def turtlebot_control_xy(self,linear_x,linear_y):
            twist = Twist()
            control_linear_x =checkLinearLimitVelocity(linear_x)
            control_linear_y =checkLinearLimitVelocity(linear_y)
            twist.linear.x = control_linear_x; twist.linear.y = control_linear_y; twist.linear.z = 0.0
            control_angular_vel =checkAngularLimitVelocity(0)
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0
            self.pub.publish(twist)

    def turtlebot_control(self,linear_vel,angle_vel):
            twist = Twist()
            control_linear_vel =checkLinearLimitVelocity(linear_vel)
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
            control_angular_vel =checkAngularLimitVelocity(angle_vel)
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            self.pub.publish(twist)


if __name__=="__main__":                 # main function
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('turtlebot3_telep') 
    Num=3                            
    ros_rate=rospy.Rate(5)              #set the cycle Time
    target_R=target_robot(0)      
    s_std=0.1       
    x_s_noise=s_std*random.normal(0,s_std,size=600)
    y_s_noise=s_std*random.normal(0,s_std,size=600)
    while(target_R.position[0][0]==0.0):
        continue   

    x_init[0]=target_R.position[0][0]
    x_init[1]=target_R.position[1][0]
    x=[x_init[0]]
    y=[x_init[1]]
    agent=[]
    for i in range(4):
        tb=optimal_robot(i+1)                    #init a instance of turtlebot class                 
        agent.append(tb)
    run_t=0
    time.sleep(1)
    t=[]
    try:  
        begin=rospy.get_time()
        while(1):  
            now_time=rospy.get_time()-begin
            print now_time
            # print Is_measure
            vel_x=-0.1*0.75*math.sin(0.1*now_time-1.571)
            vel_y=0.1*0.75*math.cos(0.1*now_time-1.571)
            velo=matrix([[vel_x],
                             [vel_y]])
            vel_x+=x_s_noise[run_t]
            vel_y+=y_s_noise[run_t]
            x.append(target_R.position[0][0])
            y.append(target_R.position[1][0])
            t.append(run_t)
            # target_R.turtlebot_control(0.075,0.1) 
            target_R.turtlebot_control_xy(vel_x,vel_y)
            # target_R.turtlebot_control(0,0)
            for i in range(1,2):
                agent[i].camera_measurement(run_t)
                print Is_measure[i]
            for i in range(1,2):
                agent[i].state_inter(velo,run_t)
            for i in range(1,2):
                print target_R.position
                agent[i].KF(agent)
            # r=tb.DFK(target_R.velo,run_t)
            # print agent[0].s_hat
            # plt.cla()
            # plt.xlim([0.5,3.5])
            # plt.ylim([-1,2])
            # ax.plot(x,y,'-g')
            # ax.plot(x1,y1,'-r')
            # plt.pause(0.00001)
  
            run_t+=1
            if (run_t==250):
                plt.cla()
                plt.figure(1)
                # plt.xlim([0.5,3.5])
                # plt.ylim([-0.5,2])
                # plt.xlim([0,250])
                # plt.ylim([-1,8])
                l1,= plt.plot(x,y,color='#000000',label='target',linewidth=2)
                for i in range(1,2):
                    plt.scatter(agent[i].s_x_plot,agent[i].s_y_plot,color=m_color[i],label='agent_'+str(agent[i].No),s=4,alpha=0.8)
                    print len(agent[i].s_x_plot)
                    # ax.plot(t,agent[i].is_visual,color=m_color[i],label='agent_'+str(agent[i].No),linewidth=2)
                    # plot_list.append(l)
                plt.legend(loc = 'upper right')  
                # plt.xlabel(u'detectability',fontproperties='SimHei',fontsize=14)
                # plt.ylabel(u't',fontproperties='SimHei',fontsize=14)
                # plt.title("Detectability of Agents",fontdict=font_dict,loc=loc)
                plt.xlabel(u'x(m)',fontproperties='SimHei',fontsize=14)
                plt.ylabel(u'y(m)',fontproperties='SimHei',fontsize=14)
                plt.title('DKF of Agents',fontdict=font_dict,loc=loc)
                for i in range(1,2):
                    plt.figure(i+2)
                    l1,= plt.plot(x,y,color='#000000',label='target',linewidth=2)
                    plt.scatter(agent[i].s_x_plot,agent[i].s_y_plot,color=m_color[i],label='agent_'+str(agent[i].No),s=4,alpha=1)
                    plt.legend(loc = 'upper right')  
                # plt.xlabel(u'detectability',fontproperties='SimHei',fontsize=14)
                # plt.ylabel(u't',fontproperties='SimHei',fontsize=14)
                # plt.title("Detectability of Agents",fontdict=font_dict,loc=loc)
                    plt.xlabel(u'x(m)',fontproperties='SimHei',fontsize=14)
                    plt.ylabel(u'y(m)',fontproperties='SimHei',fontsize=14)
                    plt.title('KF of Agent_'+str(agent[i].No),fontdict=font_dict,loc=loc)
                target_R.turtlebot_control(0,0)
                plt.show()
                os._exit()
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


# l1,=plt.plot(time_s,cx0_plot,'r',label='cx1')
#                     l2,=plt.plot(time_s,cx1_plot,'g',label='cx2')
#                     l3,=plt.plot(time_s,cx2_plot,'b',label='cx3')
#                     l4,=plt.plot(time_s,cx3_plot,'y',label='cx3')

#                     # plt.plot([0,run_time],[R0[0,0],R0[0,0]],linestyle=":",color = 'black')
#                     plt.xlim([0,run_time+5])
#                     plt.legend([l1, l2,l3,l4], ['cx1', 'cx2','cx3','cx4'], loc = 'upper right')  
#                     plt.xlabel(u'time(s)',fontproperties='SimHei',fontsize=14)
#                     plt.ylabel(u'cx(m)',fontproperties='SimHei',fontsize=14)
#                     plt.title('Agent cx')
            