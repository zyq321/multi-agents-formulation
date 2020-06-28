#!/usr/bin/env python
# Copyright (c) 2020, zyq-hit.
# All rights reserved.

import sys, os
import select as se
from numpy import *
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
import cv2


distCoeffD=array([0.0,0.0,0.0,0.0,0.0])
camera_param=[1.0,540.5,960.5,1206.89,1206.89]
camera_matrix=array([ [1206.8897, 0.0,        960.5],      # camera Matrix
                       [0.0,       1206.8897,  540.5],            
                       [0.0,       0.0,         1.0]]) 
dist = array( [0.0, 0.0, 0.0, 0.0, 0.0] )
font = cv2.FONT_HERSHEY_SIMPLEX



def PointArrangement(feature_set):
    center_point=array([0.0,0.0])
    nolist=[]
    for feature_point in feature_set:
        center_point+=feature_point
    center_point=center_point/4
    for feature_point in feature_set:
        if (feature_point[0]>center_point[0] and feature_point[1]>center_point[1]):
            nolist.append(0)
        elif(feature_point[0]<center_point[0] and feature_point[1]>center_point[1]):
            nolist.append(1)
        elif(feature_point[0]>center_point[0] and feature_point[1]<center_point[1]):
            nolist.append(2)
        elif(feature_point[0]<center_point[0] and feature_point[1]<center_point[1]):
            nolist.append(3)
    return nolist
  
def getxyz(u,v,depth,camera_param):
    camera_factor=camera_param[0]
    cx=camera_param[2]
    cy=camera_param[1]
    fx=camera_param[3]
    fy=camera_param[4]
    z=depth/ camera_factor
    x=(u - cx) * z / fx
    y=(v - cy) * z / fy
    return x,y,z
    
    
def cameraPoseFromHomography(H):
    H1 = H[:, 0]
    H2 = H[:, 1]
    H3 = cross(H1, H2)
    norm1 = linalg.norm(H1)
    norm2 = linalg.norm(H2)
    tnorm = (norm1 + norm2) / 2.0
    T = H[:, 2] / tnorm
    return cv2.Rodrigues(mat([H1, H2, H3]))


def calcRTfromHomo(H):
    H1 = H[:, 0]
    H2 = H[:, 1]
    H3 = H[:, 2]
    norm1 = linalg.norm(H1)
    norm2 = linalg.norm(H2)
    R1=H1/norm1
    R2=H2/norm1
    R3=cross(R1,R2)
    R=mat([R1, R2, R3]).T
    U,W,V=linalg.svd(R)
    R=dot(U, V)

    return cv2.Rodrigues(R)

def calcCenterfromColorRec(hsv_image,lowerb,upperb):
  mask =cv2.inRange(hsv_image, lowerb, upperb)
  # cv2.imshow("123", mask)
  # cv2.waitKey(1)
  result,contours,hierarchy = cv2.findContours(mask , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
  if len(contours)==1:
    for cnt in contours:
      (x, y, w, h) = cv2.boundingRect(cnt)
      u=x+w/2
      v=y+h/2
           
  else:
     print("error in detect rectangle")
     u=0
     v=0
  return u,v


def calcCenterfromEllipse(robot,hsv_image,lowerb,upperb):
  feature_center=[]  
  Point3D=[]
  ellipse_center=[]
  i=0
  camera_param=[1.0,540.5,960.5,1206.89,1206.89]
  mask =cv2.inRange(hsv_image, lowerb, upperb)
  result,contours,hierarchy = cv2.findContours(mask , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
  if len(contours)==4:
    for cnt in contours:
      i=i+1
      (x, y, w, h) = cv2.boundingRect(cnt)
      if w > 10 and h > 10:
        f_ellipse = cv2.fitEllipse(cnt)
        cv2.ellipse(robot.rgb_image, f_ellipse, (0,255,0), 1)
        ellipse_center.append(f_ellipse[0])
  else:
     print("error in detect rectangle")
     u=0
     v=0
     return 0,feature_center,Point3D
  No_list=array([0.0,0.0,0.0,0.0])
  No_list=PointArrangement(ellipse_center)
  for i in range(0,4):
    point2d=ellipse_center[No_list[i]]
    depth=robot.depth_image[int(point2d[1]),int(point2d[0])]
    feature_center.append((point2d))
    Point3D.append(getxyz(point2d[0],point2d[1],depth,camera_param))
    cv2.putText(robot.rgb_image, str(No_list[i]), (int(ellipse_center[i][0]), int(ellipse_center[i][1])), font, 1.0, (0, 0, 255), 2)
  return 1,feature_center,Point3D

