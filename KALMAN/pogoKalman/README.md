
# [Projet P-ANDROIDE - M1/S2 - 2023](http://androide.lip6.fr/?q=node/674)


## Pogobot Calibration 

Functions used to calibrate the Pogobot: find the correct motor values (between 0 and 1023) which make the robot move forward without turning. Keep in mind that Pogobots are very sensitive to environmental perturbations. Dust, an uneven ground and older toothbrushes can affect the trajectory of a robot.
To calibrate the robot, we turn on the motor for a short time, check if the robot turned left or right, ajust the values of the motors accordingly and repeat a few times.

**Functions:**

```C
void pogobot_quick_calibrate(int power, int* leftMotorVal, int* rightMotorVal);
```
Call this function to calibrate the motors of the Pogobot at roughly ***power***. The value of ***power*** has to be between 0 and 1023. However, the lower the value, the less precise the calibration will be. Keeping ***power*** between 524 and 716 is a good idea.
The values are returned as integers through ***leftMotorVal*** and ***rightMotorVal***.

```C
void pogobot_calibrate(int power, int startup_duration, int try_duration, int number_of_tries, float correction, int* leftMotorVal, int* rightMotorVal);
```
Same function as before, but gives more control to the user over its parameters.
Call this function to calibrate the motors of the Pogobot at roughly ***power***. During each try, the motors are turned on for ***startup_duration***ms before we actually collect IMU data. Then IMU data is collected during ***try_duration***ms. The experience is repeated ***number_of_tries*** times. Each time, robot_rotation****correction*** is applied to one of the motors. 

## Kalman filter implementation in C: 

The Kalman filter is executable at runtime. It is used to reduce the noise from the IMU. In particular, the z axis of the gyroscope (horizontal rotation) can be used to determine if the pogobot is turning. It is used in pogobot_calibrate() but can be used elsewhere.


**Functions:**

```C
void extendedKalmanFilter(
    float z_k_observation_vector[][C],        // [1][6] 6x1
    float state_estimate_k_minus_1[][C],      // [1][6] 6x1
    float P_k_minus_1[][C],                   // [6][6] 6x6
    float A_k_minus_1[][C],                   // [6][6] 6x6
    float process_noise_v_k_minus_1[][C],     // [1][6] 6x1
    float Q_k[][C],                           // [6][6] 6x6
    float R_k[][C],                           // [6][6] 6x6
    float H_k[][C],                           // [6][6] 6x6
    float sensor_noise_w_k[][C],              // [1][6] 6x1
    // returns:
    float state_estimate_k[][C],              // [1][6] 6x1
    float P_k[][C]                            // [6][6] 6x6
    );
```
Function to filter out noisy data (makes a prediction from the previous iteration then correct it with the new data). z_k_observation_vector should be the data read by the IMU : {acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]}. 
The results are stored in state_estimate_k and P_k. At the next iteration, you should assign their values to P_k_minus_1 and state_estimate_k_minus_1. The other parameters don't need to be changed (initialize once then forget about them).

```C
void initExtendedKalmanFilter(
    int power,
    float state_estimate_k_minus_1[][C],      // [1][6] 6x1
    float P_k_minus_1[][C],                   // [6][6] 6x6
    float A_k_minus_1[][C],                   // [6][6] 6x6
    float process_noise_v_k_minus_1[][C],     // [1][6] 6x1
    float Q_k[][C],                           // [6][6] 6x6
    float R_k[][C],                           // [6][6] 6x6
    float H_k[][C],                           // [6][6] 6x6
    float sensor_noise_w_k[][C]               // [1][6] 6x1
    );
```
Declare arrays and pass them as arguments of this function to initialize them, after which you can pass them as the corresponding parameters of extendedKalmanFilter().
