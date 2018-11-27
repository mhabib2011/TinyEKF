# from realtime_plot import RealtimePlotter
# from time import sleep
from ekf import KF, EKF
from math import sin, pi, cos
import pdb
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

# The loop size is used to the determined the number of loop we want to run the sim
LOOPSIZE = 1000
# These are parameter of EKF, for sensor and process noise. we are using the same one for the EKFs
r = 1.0
q = 0.2

def simulation():
    # Sensor data noise need to be Gaussian.  I am also using 0 mean here for simplicity
    alt_truth = 10.0;
    alt_meas1 = alt_truth + r * (np.random.rand()-0.5)
    alt_meas2 = alt_truth + r * (np.random.rand()-0.5)
    return alt_truth, alt_meas1, alt_meas2 

def nl_simulation(count):
    # Sensor data noise need to be Gaussian.  I am also using 0 mean here for simplicity
    # The state is position and velocity.  The velocity is the derivetive of position
    S_truth = [[10 * sin(count * pi/180)] ,  [10 * pi/180 * cos(count * pi/180)]]
    noise = [[r * (np.random.rand() - 0.5)], [r * (np.random.rand() - 0.5)]]
    S1_meas = np.add(S_truth, noise)
    S2_meas = np.add(S_truth, noise)
    return S_truth, S1_meas, S2_meas

# This function is for plotting sim and EKF data generated from 1 sensor
def plot_one_sensor(count_list, alt_truth_list, alt_meas_list, alt_est_list):
    plt.axis([0, LOOPSIZE, 5, 15])
    plt.plot(count_list, alt_truth_list,'b')
    plt.plot(count_list, alt_meas_list,'g')
    plt.plot(count_list, alt_est_list,'r')
    plt.legend(('Alt truth', 'Alt meas','Alt est'))
    plt.show()

# This function is for plotting sim and EKF data generated from 2 sensors
def plot_two_sensors(count_list, alt_truth_list, alt_meas1_list, alt_meas2_list, alt_est_list):
    plt.plot(count_list, alt_truth_list,'b')
    plt.plot(count_list, alt_meas1_list,'g')
    plt.plot(count_list, alt_meas2_list,'y')
    plt.plot(count_list, alt_est_list,'r')
    plt.legend(('Alt truth', 'Alt meas from gps1', 'Alt meas from gps2','Alt est from EKF'))
    plt.show()

if __name__ == '__main__':

    # These lists are needed for plotting and get populated by sim and EKF
    count_list = []
    alt_truth_list = []
    alt_meas1_list = []
    alt_meas2_list = []
    ekf_alt_est_list = []

    ################################
    # Running KF with one gps sensor
    ################################

    count = 0
    while count < LOOPSIZE:
    	# Storing the count in a list for plotting
        count_list.append(count)

    	# extracting simulation data and storing it in a list for plotting
        alt_truth, alt_meas1, alt_meas2 = simulation()
        alt_truth_list.append(alt_truth)
    	alt_meas1_list.append(alt_meas1)

    	# Initializing KF for one gps sensor measurement   
        if count == 0:
            alt_meas0 = alt_meas1
            p0 = 1.0
            one_sensor_kf = KF(alt_meas0,p0)
    	
        # Running KF with one gps sensor measurements 
        F = 1.0
        H = 1.0
        Q = np.power(q,2) 
        R = np.power(r,2) 
        one_sensor_kf.kf(alt_meas1, F, H, Q, R)
    	# print("alt truth %f  ekf alt_est %f " % (alt_truth, one_sensor_kf.alt_est))
    	ekf_alt_est_list.append(one_sensor_kf.alt_est)

    	count += 1

    # print("----plotting ekf for one gps----")
    # plot_one_sensor(count_list, alt_truth_list, alt_meas1_list, ekf_alt_est_list)

    ################################
    # Running KF with two gps sensor
    ################################

    # Resetting data for the second run
    count = 0
    count_list = []
    alt_truth_list = []
    alt_meas1_list = []
    alt_meas2_list = []
    ekf_alt_est_list = []
    while count < LOOPSIZE:
        # Storing the count in a list for plotting
        count_list.append(count)

        # Extracting simulation data and storing it in a list for plotting
        alt_truth, alt_meas1, alt_meas2 = simulation()
        alt_truth_list.append(alt_truth)
        alt_meas1_list.append(alt_meas1)
        alt_meas2_list.append(alt_meas2)

        # Initializing KF for two gps sensor measurements 
        if count == 0:
            alt_meas0 = (alt_meas1 + alt_meas2)/2
            p0 = 1.0
            two_sensor_kf= KF(alt_meas0, p0)

        # Running KF with 2 gps sensor measurements
        F = 1.0
        H = np.ones((2, 1))
        Q = np.power(q,2)   
        R = np.power(r,2) * np.eye(2)
        two_sensor_kf.kf([[alt_meas1],[alt_meas2]], F, H, Q, R)
        # print("alt truth %f  ekf alt_est %f " % (alt_truth, two_sensor_kf.alt_est[0,0]))
        ekf_alt_est_list.append(two_sensor_kf.alt_est[0,0])

        count += 1

    print("----plotting kf for two gps----")
    plt.axis([0, LOOPSIZE, 5, 15])
    plot_two_sensors(count_list, alt_truth_list, alt_meas1_list, alt_meas2_list, ekf_alt_est_list)


    ###################################
    # Running NL EKF with two gps sensor
    ###################################

    # Resetting data for the second run
    count = 0
    count_list = []
    alt_truth_list = []
    v_truth_list = []
    alt_meas1_list = []
    v_meas1_list = []
    alt_meas2_list = []
    v_meas2_list = []
    ekf_alt_est_list = []
    while count < LOOPSIZE:
        # Storing the count in a list for plotting
        count_list.append(count)

        # Extracting simulation data and storing it in a list for plotting
        S_truth, S1_meas, S2_meas = nl_simulation(count)
        alt_truth_list.append(S_truth[0])
        v_truth_list.append(S_truth[1])
        alt_meas1_list.append(S1_meas[0])
        v_meas1_list.append(S1_meas[1])
        alt_meas2_list.append(S2_meas[0])
        v_meas2_list.append(S2_meas[1])

        # Initializing KF for two gps sensor measurements 
        if count == 0:
            S0 = S1_meas
            P0 = np.eye((2))
            two_sensor_ekf= EKF(S0, P0)

        # Running KF with 2 gps sensor measurements
        F = [[1, pi/180], [0, 1]]
        H = np.eye((2))
        Q = np.power(q,2) * np.eye((2)) 
        R = np.power(r,2) * np.eye((2))
        two_sensor_ekf.ekf(S1_meas, F, H, Q, R)
        # print("alt truth %f  ekf alt_est %f " % (S_truth[0], two_sensor_ekf.S_est[0]))
        # print("v truth %f  ekf v_est %f " % (S_truth[1], two_sensor_ekf.S_est[1]))
        ekf_alt_est_list.append(two_sensor_ekf.S_est[0])

        count += 1

    print("----plotting NL EKF----")
    plot_two_sensors(count_list, alt_truth_list, alt_meas1_list, alt_meas2_list, ekf_alt_est_list)











