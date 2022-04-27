#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt

class KalmanFilter(object):
	def __init__(self,dt,u_x,u_y,std_acc, x_std_meas, y_std_meas):
		"""
		dt: Ziklo bat egiteko denbora
		u_x: azelerazioa x norabidean
		u_y: azelerazioa y norabidean
		std_acc : prozesuko zarataren magnitudea
		x_std_meas : Desbiderapen estandarra x norabidean
		y_std_meas : Desbiderapen estandarra y norabidean
		"""
		#denbora
		self.dt = dt
		
		#Kontrol sarrerako aldagaiak
		self.u = np.matrix([[u_x],[u_y]])
		
		#Initial state
		self.x = np.matrix([[0], [0] ,[0] , [0]])
		
		#A matrizea
		self.A = np.matrix([[1, 0, self.dt, 0],
				     [0, 1, 0, self.dt],
				     [0, 0, 1, 0],
				     [0, 0, 0, 1]])
				     
				     		     
		#B matrizea
		self.B = np.matrix ([[(self.dt**2)/2, 0],
				      [0, (self.dt**2)/2],
				      [self.dt, 0],
				      [0, self.dt]])
				      		      
		#H matrizea
		self.H = np.matrix([[1, 0, 0, 0],
				     [0, 1, 0, 0]])
				    		    
		#Q matrizea
		self.Q = np.matrix([[(self.dt**4)/4, 0 ,(self.dt**3)/2, 0],
				      [0, (self.dt**4)/4 , 0 , (self.dt**3)/2],
				      [(self.dt**3)/2 , 0 , self.dt**2 , 0 ],
				      [0, (self.dt**3)/2 , 0 , self.dt**2]]) * std_acc**2
				      
				
				      
				      
				      
		#R matrizea
		self.R = np.matrix ([[x_std_meas**2, 0],
				      [0,y_std_meas**2]])
				      
		#P, identitate matrizea bat A matrizearen tamainakoa
		self.P = np.eye(self.A.shape[1])
		
	
	
	def predict(self):
	
		#Eguneratu
		# Ekuazioa , x_k = Ax_(k-1)+ Bu_(k-1)
		self.x = np.dot(self.A,self.x) + np.dot(self.B, self.u)
		
		#Errore kobariantza
		#Ekuazioa, P =A*P*A' + Q
		self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
		return self.x[0:2]
		
	def update(self, z):
	
		# S = H*P*H' + R
		S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
		#Kalman gain
		# K = P * H' * inv ( H*P*H' + R) , noin H*P*H' + R = S den
		K = np.dot(np.dot(self.P ,self.H.T ), np.linalg.inv(S))
		
		#x_k = x_k⁻ + K(z_k - H*x_k⁻
		self.x = np.round (self.x + np.dot ( K, (z-np.dot(self.H, self.x))))

		#Identitate matrizea
		I = np.eye (self.H.shape[1])
		
		#Errore kobariantza eguneeratu
		#P_k = (I-K*H)*P_k⁻
		self.P = (I- (K*self.H)) * self.P
		
		return self.x[0:2]
		
