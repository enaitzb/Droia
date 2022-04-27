#!/usr/bin/env python
import cv2
from KalmanFilter import KalmanFilter
import rospy
from aruco_pose.msg import MarkerArray
from clover import srv
from std_srvs.srv import Trigger
import math
import numpy as np



rospy.init_node('my_node')
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
#Kalman filter objeta sortu
#KF = (dt, u_x, u_y, std_acc, x_std_meas, y_std_meas)
KF = KalmanFilter (0.1, 1,1,1,0.1,0.1)
navigate(x=0, y=0, z=1, frame_id = 'body', auto_arm = True)
rospy.sleep(1)

def markers_callback(msg):
	#Objektua detektatu
	for marker in msg.markers:
		#2 ertz kalkulatu
		x1 = marker.c1.x
		x3 = marker.c3.x
		y1 = marker.c1.y
		y3 = marker.c3.y
		#Ertzen x eta y kalkulatuta duzula zentroa kalkulatu
		xz =(x1+x3)/ 2
		yz = (y1+y3)/2
		
		# Predict
		(pre_x,pre_y) = KF.predict()
		
		#[[11 22]]
		#update
		estimatu = []
		estimatu.append(np.array([[xz],[yz]]))
		(up_x,up_y) = KF.update(estimatu[0])
		
		#Zentroa kalkulatu
		zen_x= 320/2
		zen_y= 240/2
		
		#Arucoa eta zentroaren distantzia
		dist_x=zen_x-up_x
		dist_y=zen_y-up_y
		#print("Dist x", dist_x)
		#print("Dist y", disty)
		
		#Abiadurak kalkulatu
		velx=0.005*dist_x
		if velx>0.4:
			velx=0.4
		if velx<-0.4:
			velx=-0.4
		vely=0.005*dist_y
		if vely>0.4:
			vely=0.4
		if vely<-0.4:
			vely=-0.4
		#print("ABIADURAx", velx)
		#print("ABIADURAy", vely)
		
		#Setvelocity erabili
		set_velocity(vx=vely, vy=velx, vz=0, frame_id='body')
	

rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)

rospy.spin()
		
