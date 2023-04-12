
# [Projet P-ANDROIDE - M1/S2 - 2023](http://androide.lip6.fr/?q=node/674)


### Pogobot Calibration 

Functions used to calibrate the Pogobot: find the correct motor values (between 0 and 1024) which make the robot move forward without turning. Keep in mind that Pogobots are very sensitive to environmental perturbations. Dust, an uneven ground and older toothbrushes can affect the trajectory of a robot.

**FUNCTIONS:**

```C
void pogobot_quick_calibrate(int power, int* leftMotorVal, int* rightMotorVal);
```

```C
void pogobot_calibrate(int power, int startup_duration, int try_duration, int number_of_tries, float correction, int* leftMotorVal, int* rightMotorVal);
```

### Kalman filter implementation in C: 

The Kalman filter is executable at runtime. It is used to reduce the noise from the IMU. In particular, the z axis of the gyroscope (horizontal rotation) can be used to determine if the pogobot is turning. This data is used to calibrate the pogobot (find motor values that make it move forward without turning).

**FUNCTIONS:**

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

