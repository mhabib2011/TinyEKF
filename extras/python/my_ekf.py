# from realtime_plot import RealtimePlotter
# from time import sleep
# from math import sin, pi
# import pdb
import numpy as np
import matplotlib.pyplot as plt

OFFSET = 2
LOOPSIZE = 1000
a = 1
r = 10

def simulation():
    altitude_meas = 10 + 2 * np.random.rand()
    print("  altitude meas %f" % altitude_meas)
    return altitude_meas 

class EKF:
    def __init__(self, alt0, p0):
    	self.alt_est = alt0
    	print("alt init %f " % self.alt_est)
    	self.p = p0
    	print("p init %f " % self.p)

    def ekf(self, altitude_meas):
    	self.alt_est = a * self.alt_est
    	print("alt_est predict %f " % self.alt_est)
    	
    	self.p = a * self.p * a
    	print("p predict %f " % self.p)
    	
    	gain = self.p / (self.p + r)
    	print("gain %f " % gain)
    	
    	self.alt_est = self.alt_est + gain * (altitude_meas - self.alt_est)
    	print("alt_est update %f " % self.alt_est)
    	
    	self.p = (1 - gain) * self.p
    	print("p update %f " % self.p)

def plot(count_list, altitude_meas_list, altitude_est_list):
    plt.axis([0, 10, 0, 20])
    plt.plot(count_list, altitude_meas_list,'b')
    plt.plot(count_list, altitude_est_list,'r')
    plt.legend(('alt meas','alt est'))
    plt.show()

if __name__ == '__main__':
    
    count = 0
    count_list = []
    alt_meas_list = []
    alt_est_list = []
    plt.axis([0, 10, 0, 1])

    while count < LOOPSIZE:
    	count_list.append(count)

    	alt_meas = simulation()
    	alt_meas_list.append(alt_meas)

    	if count == 0:
    		ekf_test = EKF(alt_meas,1.0)
    	ekf_test.ekf(alt_meas)
    	print("  ekf alt_est %f " % ekf_test.alt_est)
    	alt_est_list.append(ekf_test.alt_est)

    	count += 1

    print("----plotting")
    plot(count_list, alt_meas_list, alt_est_list)







