# INACHEVE (sans doute à jamais dsl)

import numpy as np
from filterpy.kalman import KalmanFilter

# Define the Kalman filter model
kf = KalmanFilter(dim_x=6, dim_z=3)

# Define the state transition matrix
kf.F = np.array([[1., 0., 0., 1., 0., 0.],
                 [0., 1., 0., 0., 1., 0.],
                 [0., 0., 1., 0., 0., 1.],
                 [0., 0., 0., 1., 0., 0.],
                 [0., 0., 0., 0., 1., 0.],
                 [0., 0., 0., 0., 0., 1.]])

# Define the measurement function matrix
kf.H = np.array([[1., 0., 0., 0., 0., 0.],
                 [0., 1., 0., 0., 0., 0.],
                 [0., 0., 1., 0., 0., 0.]])

# Define the measurement noise covariance matrix
R = np.diag([0.01, 0.01, 0.01])
kf.R = R

# Define the initial state estimate
kf.x = np.array([0., 0., 0., 0., 0., 0.])

# Define the initial covariance estimate
P = np.diag([1., 1., 1., 1., 1., 1.])
kf.P = P

# Define the process noise covariance matrix
Q = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
kf.Q = Q

# Loop through the data and update the filter
for i in range(len(accX)):
    z = np.array([accX[i], accY[i], accZ[i]])
    dt = 0.01 # assuming a 100 Hz sampling rate
    kf.F[0, 3] = dt
    kf.F[1, 4] = dt
    kf.F[2, 5] = dt
    kf.predict()
    kf.update(z)
    print(kf.x)