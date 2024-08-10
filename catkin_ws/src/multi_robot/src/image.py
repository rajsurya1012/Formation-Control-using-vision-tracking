#!/usr/bin/env python3
import roslib
roslib.load_manifest('multi_robot')
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import os
import time
import matplotlib.pyplot as plt

class image_converter:

  def __init__(self):
    
    self.bridge = CvBridge()
    self.flag=0
    self.centers=[]
    self.homo_flag=False
    self.homo=[[1/60,0,-133/20],[0,1/60,-20/3],[0,0,1]]
    self.r1=[0,0]
    self.r2=[-1 ,-1]
    self.r3=[-1 ,1]
    self.i1=[399,400]
    self.i2=[339,340]
    self.i3=[339,460]

    self.prev_r2x = 0
    self.prev_r2y = 0
    self.prev_r3x = 0
    self.prev_r3y = 0

    self.plotxi1 = []
    self.plotyi1 = []
    self.plotxi2 = []
    self.plotyi2 = []
    self.plotxi3 = []
    self.plotyi3 = []



    self.prev_time = time.time()
    
    self.pub2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=1)
    self.pub3 = rospy.Publisher('/robot3/cmd_vel', Twist, queue_size=1)
    self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback)

  def callback(self,data):
    if self.flag<10:
      self.prev_time = time.time()
      self.flag+=1
      return
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    blue_img, green_img, red_img = cv2.split(cv_image)
 
    blur = cv2.GaussianBlur(red_img, (5, 5), cv2.BORDER_DEFAULT)
    ret, thresh = cv2.threshold(blur, 58, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
      rect = cv2.minAreaRect(c)
      area=rect[1][0]*rect[1][1]
      if area<150 or area>400:
        continue
      box = cv2.boxPoints(rect)
      box = np.int0(box)
      cv_image = cv2.drawContours(cv_image,[box],0,(0,255,0),2)
      M = cv2.moments(c)
      if M['m00'] != 0:
          cx = int(M['m10']/M['m00'])
          cy = int(M['m01']/M['m00'])
          cv2.circle(cv_image, (cx, cy), 1, (0, 0, 255), -1)
          self.centers.append([cx,cy])
          
    cv2.imshow("Identification", cv_image)
    mini=1e10
    temp=[0,0]
    for p in self.centers:
      # print(p[0]-self.i1[0])
      dist=math.sqrt(((p[0]-self.i1[0])**2)+((p[1]-self.i1[1])**2))
      # print(dist)
      if(dist<mini):
        mini=dist
        temp[0]=p[0]
        temp[1]=p[1]
    self.i1[0]=temp[0]
    self.i1[1]=temp[1]
    # input("dist 1")
    mini=1e10
    for p in self.centers:
      dist=math.sqrt(((p[0]-self.i2[0])**2)+((p[1]-self.i2[1])**2))
      # print(dist)
      if(dist<mini):
        mini=dist
        temp[0]=p[0]
        temp[1]=p[1]
    self.i2[0]=temp[0]
    self.i2[1]=temp[1]
    # input("dist 2")
    mini=1e10
    for p in self.centers:
      dist=math.sqrt(((p[0]-self.i3[0])**2)+((p[1]-self.i3[1])**2))
      # print(dist)
      if(dist<mini):
        mini=dist
        temp[0]=p[0]
        temp[1]=p[1]
    self.i3[0]=temp[0]
    self.i3[1]=temp[1]
    print(self.i1,self.i2,self.i3)
    self.plotxi1.append(self.i1[0])
    self.plotyi1.append(self.i1[1])
    self.plotxi2.append(self.i2[0])
    self.plotyi2.append(self.i2[1])
    self.plotxi3.append(self.i3[0])
    self.plotyi3.append(self.i3[1])

    # input("dist 3")
    if self.homo_flag==False:
      im=[[self.i1[0],self.i2[0],self.i3[0]],[self.i1[1],self.i2[1],self.i3[1]],[1,1,1]]
      rm=[[self.r1[0],self.r2[0],self.r3[0]],[self.r1[1],self.r2[1],self.r3[1]],[1,1,1]]
      self.homo=np.dot(rm,np.linalg.inv(im))
      self.homo_flag=True
    self.update()
    self.centers.clear()
    

    
  def update(self):
    i_arr=[[self.i1[0],self.i2[0],self.i3[0]],[self.i1[1],self.i2[1],self.i3[1]],[1,1,1]]
    new_r=np.dot(self.homo,i_arr)
    self.r1[0]=new_r[0][0]
    self.r1[1]=new_r[1][0]

    self.r2[0]=new_r[0][1]
    self.r2[1]=new_r[1][1]

    self.r3[0]=new_r[0][2]
    self.r3[1]=new_r[1][2]

    print(self.r1,self.i1)
    print(self.r2,self.i2)
    print(self.r3,self.i3)

    self.update_homo()
    # input()
    rob2_xdist = self.r1[0] - self.r2[0]
    rob2_ydist = self.r1[1] - self.r2[1]

    rob3_xdist = self.r1[0] - self.r3[0]
    rob3_ydist = self.r1[1] - self.r3[1]

    r2_xerr = 1 - rob2_xdist
    r2_yerr = 1 - rob2_ydist

    r3_xerr = 1 - rob3_xdist
    r3_yerr = -1 - rob3_ydist

    print(r2_xerr, r2_yerr, r3_xerr, r3_yerr)


    Kd = 0
    r2_derrx = (r2_xerr - self.prev_r2x) / (time.time() - self.prev_time)
    r2_derry = (r2_yerr - self.prev_r2y) / (time.time() - self.prev_time)
    r3_derrx = (r3_xerr - self.prev_r3x) / (time.time() - self.prev_time)
    r3_derry = (r3_yerr - self.prev_r3y) / (time.time() - self.prev_time)

    self.prev_time = time.time()

    Kp = 0.5
    r2_xvel = (-Kp * r2_xerr) + (Kd * r2_derrx)
    r2_yvel = (Kp * r2_yerr) + (Kd * r2_derry)
    r3_xvel = (-Kp * r3_xerr) + (Kd * r3_derrx)
    r3_yvel = (Kp * r3_yerr) + (Kd * r3_derry)
    # print(r2_yvel,r3_yvel)
    
    r2_msg=Twist()
    r3_msg=Twist()
    r2_msg.linear.x = r2_xvel
    r2_msg.linear.y = r2_yvel
    r3_msg.linear.x = r3_xvel
    r3_msg.linear.y = r3_yvel
    print(r2_msg)
    print(r3_msg)



    self.pub2.publish(r2_msg)
    self.pub3.publish(r3_msg)

  def update_homo(self):
    im=[[self.i1[0],self.i2[0],self.i3[0]],[self.i1[1],self.i2[1],self.i3[1]],[1,1,1]]
    rm=[[self.r1[0],self.r2[0],self.r3[0]],[self.r1[1],self.r2[1],self.r3[1]],[1,1,1]]
    self.homo=np.dot(rm,np.linalg.inv(im))

  

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    plt.plot(ic.plotx, ic.ploty)
    plt.show()
    input()
    print("Shutting down")
  cv2.destroyAllWindows()
  plt.figure(1)
  plt.title("Robot Position")
  plt.plot(ic.plotxi1,ic.plotyi1, label = "Robot 1 Position")
  plt.plot(ic.plotxi2,ic.plotyi2, label = "Robot 2 Position")
  plt.plot(ic.plotxi3,ic.plotyi3, label = "Robot 3 Position")
  plt.show()
  # input()

if __name__ == '__main__':
    main(sys.argv)