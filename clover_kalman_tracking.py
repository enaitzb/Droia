#!/usr/bin/env python

from typing_extensions import Self
import rospy
import cv2
from KalmanFilter import KalmanFilter
from aruco_pose.msg import MarkerArray
from clover.srv import SetVelocity, Navigate
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import numpy as np

class cloverKalman:

  def __init__(self):
    #Nodoa hasieratu
    rospy.init_node("clover_kalman_tracking")
    #Zerbitzuak
    self.set_velocity = rospy.ServiceProxy('set_velocity', SetVelocity)
    self.navigate = rospy.ServiceProxy('navigate', Navigate)
    self.land = rospy.ServiceProxy('land', Trigger)
    #Subscriber
    rospy.Subscriber('aruco_detect/markers', MarkerArray, self.markers_callback)
    rospy.Subscriber("/main_camera/image_raw", Image, self.image_callback)
    #Publisher
    self.pub = rospy.Publisher('aruco_kalman_image', Image,queue_size=10)
    #Kalman filter objeta sortu
    #KF = (dt, u_x, u_y, std_acc, x_std_meas, y_std_meas)
    self.KF = KalmanFilter (0.1, 1,1,1,0.1,0.1)
    #Irudia
    self.image = None
    self.br = CvBridge()
    self.velx=0.0
    self.vely=0.0
    self.velz=0.0
    self.kp=0.005
    self.marker_found=False
    self.frame_id = 'body'
    self.kont=0
    self.xz=0
    self.yz=0
    # Shutdown seinalea jasotzean stop funtzioa exekutatu
    rospy.on_shutdown(self.stop) 

  def run(self):
    self.navigate(x=0, y=0, z=1.0, frame_id = self.frame_id, auto_arm = True)
    rospy.sleep(10)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      
      if self.marker_found == True:
        self.set_velocity(vx=self.vely, vy=self.velx, vz=self.velz, frame_id= self.frame_id)
      else:
        self.set_velocity(vx=0.0, vy=0.0, vz=0.0, frame_id=self.frame_id)
      
      if self.image is not None:
        cv2.line(self.image, (int(self.xz)-5, int(self.yz)-5), (int(self.xz)+5, int(self.yz)+5), (0, 0, 255), 5)
        cv2.line(self.image, (int(self.xz)-5, int(self.yz)+5), (int(self.xz)+5, int(self.yz)-5), (0, 0, 255), 5)
        self.pub.publish(self.br.cv2_to_imgmsg(self.image))
      r.sleep()
    
  def markers_callback(self,msg):
	  #Objektua detektatu
    if len(msg.markers)==0:
      self.marker_found = False
      self.kont=0
    else:  
      self.marker_found = True
      if self.kont==0:
        #self.navigate(x=0, y=0, z=-0.5, frame_id = self.frame_id, auto_arm = True)
        self.kont+=1
       
      #rospy.loginfo("callback")
      for marker in msg.markers:
        #2 ertz kalkulatu
        x1 = marker.c1.x
        x3 = marker.c3.x
        y1 = marker.c1.y
        y3 = marker.c3.y
        #Ertzen x eta y kalkulatuta duzula zentroa kalkulatu
        self.xz =(x1+x3)/ 2
        self.yz = (y1+y3)/2
        
        # Predict
        (pre_x,pre_y) = self.KF.predict()
        
        #[[11 22]]
        #update
        estimatu = []
        estimatu.append(np.array([[self.xz],[self.yz]]))
        (up_x,up_y) = self.KF.update(estimatu[0])
        
        #Zentroa kalkulatu
        zen_x= 320/2
        zen_y= 240/2
        
        #Arucoa eta zentroaren distantzia
        dist_x=zen_x-up_x
        dist_y=zen_y-up_y
        #print("Dist x", dist_x)
        #print("Dist y", disty)
        
        #Abiadurak kalkulatu
        self.velx=self.kp*dist_x
        if self.velx>0.4:
          self.velx=0.4
        if self.velx<-0.4:
          self.velx=-0.4
        self.vely=0.005*dist_y
        if self.vely>0.4:
          self.vely=0.4
        if self.vely<-0.4:
          self.vely=-0.4
        #print("ABIADURAx", velx)
        #print("ABIADURAy", vely)
      

  def image_callback(self, msg):
    self.image = self.br.imgmsg_to_cv2(msg)

  def stop(self):
    self.land()
    print("Landing")

def main():
  try:
    clover_kalman= cloverKalman()
    clover_kalman.run()
  except rospy.ROSInterruptException:
    pass
  

if __name__ == '__main__':
  main()


