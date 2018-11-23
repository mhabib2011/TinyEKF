import pdb
import numpy as np
from numpy.linalg import inv

class KF:
    def __init__(self, alt0, P0):
    	self.alt_est = alt0
    	# print("alt init %f " % self.alt_est)
    	self.P = P0

    def kf(self, alt_meas, F, H, Q, R):
    	# EKF predict stage
        self.alt_est = F * self.alt_est
        # print("alt_est predict %f " % self.alt_est)
    	
    	self.P = F * self.P * F + Q
        # print("p predict %f " % self.P)
    	
        # EKF update stage
        if np.size(H ) < 2:
    	   gain = self.P * H / (H * self.P * H + R)
           self.alt_est = self.alt_est + gain * (alt_meas - H * self.alt_est)
           self.P = (1 - gain * H) * self.P
        else:
           gain = (self.P * np.transpose(H)).dot(inv((H * self.P).dot(np.transpose(H)) + R))
           self.alt_est = self.alt_est + gain.dot(alt_meas - H * self.alt_est)
           self.P = (1 - gain.dot(H)) * self.P
    	
    	# print("alt_est update %f " % self.alt_est)
    	# print("p update %f " % self.P)

class EKF:
    def __init__(self, S0, P0):
        self.S_est = S0
        # print("alt init %f " % self.alt_est)
        self.P = P0

    def ekf(self, Z, F, H, Q, R):
        # EKF predict stage
        self.S_est = np.dot(F , self.S_est)
        # print("alt_est predict %f " % self.alt_est)
        
        self.P = np.dot(np.dot(F , self.P) , np.transpose(F)) + Q
        # print("p predict %f " % self.P)
        
        # EKF update stage
        gain = np.dot(np.dot(self.P , np.transpose(H)) , inv(np.dot(H * self.P , np.transpose(H)) + R))
        self.S_est = self.S_est + np.dot(gain , (Z - np.dot(H , self.S_est)))
        self.P = np.dot((np.eye(2) - np.dot(gain , H)) , self.P)
        
        # print("alt_est update %f " % self.alt_est)
        # print("p update %f " % self.P)