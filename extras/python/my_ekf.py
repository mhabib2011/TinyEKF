# from realtime_plot import RealtimePlotter
# from time import sleep
# from math import sin, pi
import pdb
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

# The loop size is used to the determined the number of loop we want to run the sim
LOOPSIZE = 1000
# This is a parameter from the model, it is the same of all EKF tested in this example
# regardless of the sensor and sensor fusion
a = 1

def simulation():
    # The two sensor data need to be close to each other because they are measurements of 
    # altitude from 2 sensors, but should be slightly different to test how well sensor
    # fusion is working
    altitude_meas1 = 10 + 2 * np.random.rand()
    altitude_meas2 = altitude_meas1 + 0.5 * np.random.rand()
    print("  altitude meas1 %f altitude meas2 %f" % (altitude_meas1, altitude_meas2))
    return altitude_meas1, altitude_meas2 

class EKF:
    def __init__(self, alt0, p0, c, r):
    	self.alt_est = alt0
    	print("alt init %f " % self.alt_est)
    	self.p = p0
        self.c = c;
        self.r= r;

    def ekf(self, alt_meas):
    	# EKF predict stage
        self.alt_est = a * self.alt_est
        print("alt_est predict %f " % self.alt_est)
    	
    	self.p = a * self.p * a
        print("p predict %f " % self.p)
    	
        # EKF update stage
        if np.size(self.c ) < 2:
    	   gain = self.p * self.c / (self.c * self.p * self.c + self.r)
           self.alt_est = self.alt_est + gain * (alt_meas - self.c * self.alt_est)
           self.p = (1 - gain * self.c) * self.p
        else:
           gain = (self.p * np.transpose(self.c)).dot(inv( (self.c * self.p).dot(np.transpose(self.c)) + self.r))
           self.alt_est = self.alt_est + gain.dot(alt_meas - self.c * self.alt_est)
           self.p = (1 - gain.dot(self.c)) * self.p
    	
    	print("alt_est update %f " % self.alt_est)
    	print("p update %f " % self.p)

# This function is for plotting sim and EKF data generated from 1 sensor
def plot_one_sensor_ekf(count_list, altitude_meas_list, altitude_est_list):
    plt.plot(count_list, altitude_meas_list,'b')
    plt.plot(count_list, altitude_est_list,'r')
    plt.legend(('alt meas','alt est'))
    plt.show()

# This function is for plotting sim and EKF data generated from 2 sensors
def plot_two_sensor_ekf(count_list, altitude_meas1_list, altitude_meas2_list, altitude_est_list):
    plt.plot(count_list, altitude_meas1_list,'b')
    plt.plot(count_list, altitude_meas2_list,'g')
    plt.plot(count_list, altitude_est_list,'r')
    plt.legend(('alt meas','alt est'))
    plt.show()

if __name__ == '__main__':

    # these lists are needed for plotting and get populated by sim and EKF
    count_list = []
    alt_meas1_list = []
    alt_meas2_list = []
    ekf_alt_est_list = []

    ################################
    # Running EKF with one gps sensor
    ################################

    count = 0
    while count < LOOPSIZE:
    	# Storing the count in a list for plotting
        count_list.append(count)

    	# extracting simulation data and storing it in a list for plotting
        alt_meas1, alt_meas2 = simulation()
    	alt_meas1_list.append(alt_meas1)

    	# Initializing EKF for one gps sensor measurement   
        if count == 0:
            alt_meas0 = alt_meas1
            p0 = 1.0
            c = 1.0
            r = 10.0
            ekf_test = EKF(alt_meas0,p0, c, r)
    	
        # Running EKF with 2 gps sensor measurements   
        ekf_test.ekf(alt_meas1)
    	print("  ekf alt_est %f " % ekf_test.alt_est)
    	ekf_alt_est_list.append(ekf_test.alt_est)

    	count += 1

    print("----plotting ekf one gps----")
    plot_one_sensor_ekf(count_list, alt_meas1_list, ekf_alt_est_list)

    ################################
    # Running EKF with two gps sensor
    ################################

    # Resetting data for the second run
    count = 0
    count_list = []
    alt_meas1_list = []
    alt_meas2_list = []
    ekf_alt_est_list = []
    while count < LOOPSIZE:
        # Storing the count in a list for plotting
        count_list.append(count)

        # Extracting simulation data and storing it in a list for plotting
        alt_meas1, alt_meas2 = simulation()
        alt_meas1_list.append(alt_meas1)
        alt_meas2_list.append(alt_meas2)

        # Initializing EKF for two gps sensor measurements 
        if count == 0:
            alt_meas0 = (alt_meas1 + alt_meas2)/2
            p0 = 1.0
            c = np.ones((2, 1))
            r = 100.0 * np.eye(2)
            ekf_test = EKF(alt_meas0, p0, c, r)
        # Running EKF with 2 gps sensor measurements    
        ekf_test.ekf([[alt_meas1],[alt_meas2]])
        print("  ekf alt_est %f " % ekf_test.alt_est)
        ekf_alt_est_list.append(ekf_test.alt_est[0,0])

        count += 1

    print("----plotting ekf two gps----")
    plot_two_sensor_ekf(count_list, alt_meas1_list, alt_meas2_list, ekf_alt_est_list)







