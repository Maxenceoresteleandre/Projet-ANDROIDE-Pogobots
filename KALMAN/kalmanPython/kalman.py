import numpy as np
import matplotlib.pyplot as plt
 
# adapté de:
# Author: Addison Sears-Collins
# https://automaticaddison.com
# Description: Extended Kalman Filter example (two-wheeled mobile robot)
 
# Supress scientific notation when printing NumPy arrays
np.set_printoptions(precision=3,suppress=True)
 
# A matrix
# 6x6 matrix -> number of states x number of states matrix
# Expresses how the state of the system [accx, accy, accz, gyrox, gyroy, gyroz] changes 
# from k-1 to k when no control command is executed.
# Typically a robot on wheels only drives when the wheels are told to turn.
# For this case, A is the identity matrix.
A_k_minus_1 = np.array([[1.0,   0,   0,   0,   0,   0],
                        [  0, 1.0,   0,   0,   0,   0],
                        [  0,   0, 1.0,   0,   0,   0],
                        [  0,   0,   0, 1.0,   0,   0],
                        [  0,   0,   0,   0, 1.0,   0],
                        [  0,   0,   0,   0,   0, 1.0]])
 
# Noise applied to the forward kinematics (calculation
# of the estimated state at time k from the state
# transition model of the mobile robot). This is a vector
# with the number of elements equal to the number of states
process_noise_v_k_minus_1 = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
     
# State model noise covariance matrix Q_k
# When Q is large, the Kalman Filter tracks large changes in 
# the sensor measurements more closely than for smaller Q.
# Q is a square matrix that has the same number of rows as states.
Q_k = np.array([[3.0,   0,   0,   0,   0,   0],
                [  0, 3.0,   0,   0,   0,   0],
                [  0,   0, 3.0,   0,   0,   0],
                [  0,   0,   0, 1.0,   0,   0],
                [  0,   0,   0,   0, 1.0,   0],
                [  0,   0,   0,   0,   0, 1.0]])
                 
# Measurement matrix H_k
# Used to convert the predicted state estimate at time k
# into predicted sensor measurements at time k.
# In this case, H will be the identity matrix since the 
# estimated state maps directly to state measurements from the 
# odometry data [accx, accy, accz, gyrox, gyroy, gyroz]
# H has the same number of rows as sensor measurements
# and same number of columns as states.
H_k = np.array([[1.0,   0,   0,   0,   0,   0],
                [  0, 1.0,   0,   0,   0,   0],
                [  0,   0, 1.0,   0,   0,   0],
                [  0,   0,   0, 1.0,   0,   0],
                [  0,   0,   0,   0, 1.0,   0],
                [  0,   0,   0,   0,   0, 1.0]])
                         
# Sensor measurement noise covariance matrix R_k
# Has the same number of rows and columns as sensor measurements.
# If we are sure about the measurements, R will be near zero.
R_k = np.array([[3.0,   0,   0,   0,   0,   0],
                [  0, 3.0,   0,   0,   0,   0],
                [  0,   0, 3.0,   0,   0,   0],
                [  0,   0,   0, 1.0,   0,   0],
                [  0,   0,   0,   0, 1.0,   0],
                [  0,   0,   0,   0,   0, 1.0]])  
                 
# Sensor noise. This is a vector with the
# number of elements equal to the number of sensor measurements.
sensor_noise_w_k = np.array([0.07, 0.07, 0.07, 0.04, 0.04, 0.04])


TEST = 1
 
def ekf(z_k_observation_vector, state_estimate_k_minus_1, P_k_minus_1, dk):
    """
    Extended Kalman Filter. Fuses noisy sensor measurement to 
    create an optimal estimate of the state of the robotic system.
         
    INPUT
        :param z_k_observation_vector The observation from the Odometry
            6x1 NumPy Array [accx, accy, accz, gyrox, gyroy, gyroz] in the global reference frame
            in [m/s2, m/s2, m/s2, rad, rad, rad].
        :param state_estimate_k_minus_1 The state estimate at time k-1
            6x1 NumPy Array [accx, accy, accz, gyrox, gyroy, gyroz] in the global reference frame
            in [m/s2, m/s2, m/s2, rad, rad, rad].
        :param P_k_minus_1 The state covariance matrix estimate at time k-1
            6x6 NumPy Array
        :param dk Time interval in seconds
             
    OUTPUT
        :return state_estimate_k near-optimal state estimate at time k  
            6x1 NumPy Array ---> [m/s2, m/s2, m/s2, rad, rad, rad]
        :return P_k state covariance_estimate for time k
            6x6 NumPy Array                 
    """
    ######################### Predict #############################
    # Predict the state estimate at time k based on the state 
    # estimate at time k-1 and the control input applied at time k-1.
    state_estimate_k = A_k_minus_1 @ state_estimate_k_minus_1 + process_noise_v_k_minus_1
    
    print(f'State Estimate Before EKF={state_estimate_k}')
             
    # Predict the state covariance estimate based on the previous
    # covariance and some noise
    P_k = A_k_minus_1 @ P_k_minus_1 @ A_k_minus_1.T + (
            Q_k)
         
    ################### Update (Correct) ##########################
    # Calculate the difference between the actual sensor measurements
    # at time k minus what the measurement model predicted 
    # the sensor measurements would be for the current timestep k.
    measurement_residual_y_k = z_k_observation_vector - (
            (H_k @ state_estimate_k) + (
            sensor_noise_w_k))
 
    print(f'Observation={z_k_observation_vector}')
             
    # Calculate the measurement residual covariance
    S_k = H_k @ P_k @ H_k.T + R_k
         
    # Calculate the near-optimal Kalman gain
    # We use pseudoinverse since some of the matrices might be
    # non-square or singular.
    K_k = P_k @ H_k.T @ np.linalg.pinv(S_k)
         
    # Calculate an updated state estimate for time k
    state_estimate_k = state_estimate_k + (K_k @ measurement_residual_y_k)
     
    # Update the state covariance estimate for time k
    P_k = P_k - (K_k @ H_k @ P_k)
     
    # Print the best (near-optimal) estimate of the current state of the robot
    print(f'State Estimate After EKF={state_estimate_k}')
 
    # Return the updated state and covariance estimates
    return state_estimate_k, P_k
     


def main():
 
    # We start at time k=1
    k = 1
     
    # Time interval in seconds
    dk = 1
 
    # Create a list of sensor observations at successive timesteps
    # Each list within z_k is an observation vector.

    if TEST == 1:
        f = open("../test_imu_kalman//imu_data.txt")
    elif TEST == 2:
        f = open("../test_imu_kalman/imu_motor512.txt")
    elif TEST == 3:
        f = open("../test_imu_kalman/imu_motor1023.txt")
    else: 
        print("TEST incorrect")

    l = f.readlines()
    L = len(l)
    f.close()

    z_k = np.zeros((len(l)//2, 6))
    print(z_k.shape)

    j = 0
    for i in range(len(l)):
        seqBis = l[i].split()
        # formatage:
        # seqBis = [sensor, x, coordX, y, coordY, z, coordZ] => len = 7

        # ACCELERATION READ
        if seqBis[0] == 'Acc':
            z_k[j][0] = (int(seqBis[2])/1000)
            z_k[j][1] = (int(seqBis[4])/1000)
            z_k[j][2] = (int(seqBis[6])/1000)
        # GYROSCOPE READ
        elif seqBis[0] == 'Gyro':
            z_k[j][3] = (int(seqBis[2])/100)
            z_k[j][4] = (int(seqBis[4])/100)
            z_k[j][5] = (int(seqBis[6])/100)
            j+=1
        else:
            print("fichier corrompu")
          
                     
    # The estimated state vector at time k-1 in the global reference frame.
    # [accx_k_minus_1, accy_k_minus_1, accyaw_k_minus_1, gyrox_k_minus_1, gyroy_k_minus_1, gyroyaw_k_minus_1]
    # [m/s2, m/s2, m/s2, rad, rad, rad]
    state_estimate_k_minus_1 = np.array([0.0, 0.0, -9.81, 0.0, 0.0, 0.0])
     
    # State covariance matrix P_k_minus_1
    # This matrix has the same number of rows (and columns) as the 
    # number of states (i.e. 6x6 matrix). P is sometimes referred
    # to as Sigma in the literature. It represents an estimate of 
    # the accuracy of the state estimate at time k made using the
    # state transition matrix. We start off with guessed values.
    P_k_minus_1 = np.array([[1.0,   0,   0,   0,   0,   0],
                            [  0, 1.0,   0,   0,   0,   0],
                            [  0,   0, 1.0,   0,   0,   0],
                            [  0,   0,   0, 1.0,   0,   0],
                            [  0,   0,   0,   0, 1.0,   0],
                            [  0,   0,   0,   0,   0, 1.0]])
                             
    # Start at k=1 and go through each of the 8 sensor observations, 
    # one at a time. 
    # We stop right after timestep k=8 (i.e. the last sensor observation)

    accX_after_EKF = []
    accY_after_EKF = []
    accZ_after_EKF = []
    gyroX_after_EKF = []
    gyroY_after_EKF = []
    gyroZ_after_EKF = []
    for k, obs_vector_z_k in enumerate(z_k,start=1):
     
        # Print the current timestep
        print(f'Timestep k={k}')  
         
        # Run the Extended Kalman Filter and store the 
        # near-optimal state and covariance estimates
        optimal_state_estimate_k, covariance_estimate_k = ekf(
            obs_vector_z_k, # Most recent sensor measurement
            state_estimate_k_minus_1, # Our most recent estimate of the state
            P_k_minus_1, # Our most recent state covariance matrix
            dk) # Time interval
         
        # Get ready for the next timestep by updating the variable values
        state_estimate_k_minus_1 = optimal_state_estimate_k
        P_k_minus_1 = covariance_estimate_k
        accX_after_EKF.append(optimal_state_estimate_k[0])
        accY_after_EKF.append(optimal_state_estimate_k[1])
        accZ_after_EKF.append(optimal_state_estimate_k[2])
        gyroX_after_EKF.append(optimal_state_estimate_k[3])
        gyroY_after_EKF.append(optimal_state_estimate_k[4])
        gyroZ_after_EKF.append(optimal_state_estimate_k[5])
        
        # Print a blank line
        print()
 
    return accX_after_EKF, accY_after_EKF, accZ_after_EKF, gyroX_after_EKF, gyroY_after_EKF, gyroZ_after_EKF, L


accX, accY, accZ, gyroX, gyroY, gyroZ, L = main()
iter = range(1, L//2+1)
# plot
pltAcc = plt.subplot(211)
pltAcc.plot(iter, accX, label="acc X")
pltAcc.plot(iter, accY, label="acc Y")
pltAcc.plot(iter, accZ,label="acc Z")
pltAcc.set_xlabel("timestep")
pltAcc.set_ylabel("acc (m/s^2)")

titre = "Données de l'IMU APRES EKF"
if TEST == 1:
    titre+=" (sans moteur)"
elif TEST == 2:
    titre+=" (moteur m à 512)"
elif TEST == 3:
    titre+=" (moteur m à 1023)"
pltAcc.set_title(titre)
pltAcc.legend()

pltGyro = plt.subplot(212)
pltGyro.plot(iter, gyroX, label="gyro X")
pltGyro.plot(iter, gyroY, label="gyro Y")
pltGyro.plot(iter, gyroZ, label="gyro Z")
pltGyro.set_xlabel("timestep")
pltGyro.set_ylabel("gyro (rads)")
pltGyro.set_yticks([-np.pi, -np.pi/2, 0, np.pi/2, np.pi], [r'$-\pi$', r'$-\frac{ \pi }{ 2 }$', '$0$', r'$\frac{ \pi }{ 2 }$',
                                                        r'$\pi$'])
pltGyro.legend()



plt.show()
