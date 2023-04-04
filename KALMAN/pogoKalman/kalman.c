
#include "pogobot.h"
#include "kalman.h"
#include "display.h"



//##############################################################
//##############################################################
// MATRIX OPERATIONS

void multMat(float matRes[][C], float mat1[][C], float mat2[][C], int r1) {
    for (int i = 0; i < r1; i++) {
        for (int j = 0; j < C; j++) {
            matRes[i][j] = 0;
            for (int k = 0; k < R2; k++) {
                matRes[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

void addMat(float matRes[][C], float mat1[][C], float mat2[][C], int r) {
  for (int i=0; i<r; i++) {
    for (int j=0; j<C; j++) {
      matRes[i][j] = mat1[i][j] + mat2[i][j];
    }
  }
}

void subtractMat(float matRes[][C], float mat1[][C], float mat2[][C], int r) {
  for (int i=0; i<r; i++) {
    for (int j=0; j<C; j++) {
      matRes[i][j] = mat1[i][j] - mat2[i][j];
    }
  }
}

void transpose(float matRes[][C], float mat[][C]) {
  for (int i=0; i<C; i++)
    for (int j=0; j<C; j++)
      matRes[i][j] = mat[j][i];
}

void identity_matrix(float matRes[][C]) {
  for (int i=0; i<C; i++) {
    for (int j=0; j<C; j++) {
      if (i==j) {
        matRes[i][j] = 1.0;
      } else {
        matRes[i][j] = 0.0;
      }
    }
  }
}

void copy_matrix(float matRes[][C], float mat[][C], int r) {
  for (int i=0; i<r; i++)
    for (int j=0; j<C; j++)
      matRes[i][j] = mat[i][j];
}

void pseudo_inverse(float IM[][C], float mat[][C]) {
  // from the python code here:
  // https://integratedmlai.com/matrixinverse/
  float AM[C][C];
  float I[C][C];
  copy_matrix(AM, mat, 6);
  identity_matrix(I);
  copy_matrix(IM, I, 6);

  for (int fd=0; fd<C; fd++) {
    float fdScaler = 1.0 / AM[fd][fd];
    int indices[C];
    for (int i=0; i<C; i++)
      indices[i] = i;
    for (int j=0; j<C; j++) {
      AM[fd][j] = AM[fd][j] * fdScaler;
      IM[fd][j] = IM[fd][j] * fdScaler;
    }
    for (int k=0; k<C; k++) {
      if (k != fd) {
        int i = indices[k];
        float crScaler = AM[i][fd];
        for (int j=0; j<C; j++) {
          AM[i][j] = AM[i][j] - crScaler * AM[fd][j];
          IM[i][j] = IM[i][j] - crScaler * IM[fd][j];
        }
      }
    }
  }
}

void combine_arrays(float res[], float arr1[], float arr2[], int len1, int len2) {
  int i;
  for (i=0; i<len1; i++) {
    res[i] = arr1[i];
  }
  for (int j=0; j<len2; j++) {
    res[i+j] = arr2[j];
  }
}

void split_array(float base[], float arr1[], float arr2[], int len1, int len2) {
  int i;
  for (i=0; i<len1; i++) {
    arr1[i] = base[i];
  }
  for (int j=0; j<len2; j++) {
    arr2[j] = base[i+j];
  }
}


//##############################################################
//##############################################################
// EXTENDED KALMAN FILTER
void init_ekf(
    int power,
    float state_estimate_k_minus_1[][C],       // [1][6] 6x1
    float P_k_minus_1[][C],                    // [6][6] 6x6
    float A_k_minus_1[][C],                    // [6][6] 6x6
    float process_noise_v_k_minus_1[][C],      // [1][6] 6x1
    float Q_k[][C],                            // [6][6] 6x6
    float R_k[][C],                            // [6][6] 6x6
    float H_k[][C],                            // [6][6] 6x6
    float sensor_noise_w_k[][C]                // [1][6] 6x1
    ) {
      float initial_state_estimate[1][6] = {
        {0.0, 0.0, -GRAVITY, 0.0, 0.0, 0.0}};
      float initial_P_k[6][6] = {
        {0.1, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.1, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.1, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.1, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.1, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.1}
        };
      float initial_Q_k[6][6] = {
        {0.01, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.01, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.01, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.01, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.01, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.1}
        };
      float initial_R_k[6][6] = {
        {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.2}
        };
      if (power != 512) {
        printf("For motorPower at 512, initial_R_k in written. For other values it is not.\n");
        printf("Fuck.\n");
        printf("Check kalman.py for reference.\n");
        exit(1);
      }
      float intial_process_noise[1][6] = {
        {0.1, 0.1, 0.1, 0.01, 0.01, 0.01}
      };
      float initial_sensor_noise[1][6] = {
        {0.07, 0.07, 0.07, 0.05, 0.05, 0.05}
      };
      copy_matrix(state_estimate_k_minus_1, initial_state_estimate, 1);
      copy_matrix(P_k_minus_1, initial_P_k, 6);
      identity_matrix(A_k_minus_1);
      copy_matrix(process_noise_v_k_minus_1, intial_process_noise, 1);
      copy_matrix(Q_k, initial_Q_k, 6);
      copy_matrix(R_k, initial_R_k, 6);
      identity_matrix(H_k);
      copy_matrix(sensor_noise_w_k, initial_sensor_noise, 1);

    }

void extendedKalmanFilter(
    float z_k_observation_vector[][C],         // [1][6] 6x1
    float state_estimate_k_minus_1[][C],       // [1][6] 6x1
    float P_k_minus_1[][C],                    // [6][6] 6x6
    float A_k_minus_1[][C],                    // [6][6] 6x6
    float process_noise_v_k_minus_1[][C],      // [1][6] 6x1
    float Q_k[][C],                            // [6][6] 6x6
    float R_k[][C],                            // [6][6] 6x6
    float H_k[][C],                            // [6][6] 6x6
    float sensor_noise_w_k[][C],               // [1][6] 6x1
    // returns:
    float state_estimate_k[][C],               // [1][6] 6x1
    float P_k[][C]                             // [6][6] 6x6
    ) {
      float tmpMat6x1[1][6];
      float tmpMat2_6x1[1][6];
      float tmpMat6x6[6][6];
      float tmpMat2_6x6[6][6];
      float tmpMat3_6x6[6][6];

      //######### Predict #########
      // Predict the state estimate
      addMat(tmpMat6x1, state_estimate_k_minus_1, process_noise_v_k_minus_1, 1);
      multMat(state_estimate_k, A_k_minus_1, tmpMat6x1, 1);
      // Predict the state covariance estimate
      multMat(tmpMat6x6, A_k_minus_1, P_k_minus_1, 6);
      transpose(tmpMat2_6x6, A_k_minus_1);
      multMat(tmpMat3_6x6, tmpMat6x6, tmpMat2_6x6, 6);
      addMat(P_k, tmpMat3_6x6, Q_k, 6);

      //######### Correct #########
      // Calculate the difference between the measurements and prediction
      float measurement_residual_y_k[1][6];
      multMat(tmpMat6x1, H_k, state_estimate_k, 1);
      addMat(tmpMat2_6x1, tmpMat6x1, sensor_noise_w_k, 1);
      subtractMat(measurement_residual_y_k, z_k_observation_vector, tmpMat2_6x1, 1);
      // Calculate the measurement residual covariance
      float S_k[6][6];
      multMat(tmpMat6x6, H_k, P_k, 6);
      transpose(tmpMat2_6x6, H_k);
      multMat(tmpMat3_6x6, tmpMat6x6, tmpMat2_6x6, 6);
      addMat(S_k, tmpMat3_6x6, R_k, 6);         //S_k = H_k @ P_k @ H_k.T + R_k
      // Calculate the near-optimal Kalman gain
      float K_k[6][6];
      multMat(tmpMat6x6, P_k, tmpMat2_6x6, 6);  //P_k @ H_k.T
      pseudo_inverse(tmpMat2_6x6, S_k);         //np.linalg.pinv(S_k)
      multMat(K_k, tmpMat6x6, tmpMat2_6x6, 6);  //K_k = P_k @ H_k.T @ np.linalg.pinv(S_k)
      // Calculate an updated state estimate for time k
      multMat(tmpMat2_6x1, K_k, measurement_residual_y_k, 1);
      addMat(state_estimate_k, state_estimate_k, tmpMat2_6x1, 1);
      // Update the state covariance estimate for time k
      multMat(tmpMat6x6, K_k, H_k, 6);
      multMat(tmpMat2_6x6, tmpMat6x6, P_k, 6);
      subtractMat(P_k, P_k, tmpMat2_6x6, 6);
    }

void calibrate_pogobot(int power, int* leftMotorVal, int* rightMotorVal) {
    float acc[3];
    float gyro[3];

    // kalman arguments
    float obs_vector_z_k[1][6];
    float state_estimate_k_minus_1[1][6];
    float P_k_minus_1[6][6];
    float A_k_minus_1[6][6];
    float process_noise_v_k_minus_1[1][6];
    float Q_k[6][6];
    float R_k[6][6];
    float H_k[6][6];
    float sensor_noise_w_k[1][6];
    init_ekf(
        512,
        state_estimate_k_minus_1,       // [1][6] 
        P_k_minus_1,                    // [6][6] 
        A_k_minus_1,                    // [6][6] 
        process_noise_v_k_minus_1,      // [1][6] 
        Q_k,                            // [6][6] 
        R_k,                            // [6][6] 
        H_k,                            // [6][6] 
        sensor_noise_w_k                // [1][6] 
    );
    // kalman results
    float state_estimate_k[1][6];
    float P_k[6][6];

    int powerLeft  = power + 200;
    int powerRight = power;

    pogobot_motor_set(motorL, powerLeft);
    pogobot_motor_set(motorR, powerRight);

    
    for (int i=0; i<CALIBRATION_DURATION; i++) {
        pogobot_imu_read(acc, gyro);
        combine_arrays(*obs_vector_z_k, acc, gyro, 3, 3);
        extendedKalmanFilter(
            obs_vector_z_k,                 // [1][6]
            state_estimate_k_minus_1,       // [1][6]
            P_k_minus_1,                    // [6][6]
            A_k_minus_1,                    // [6][6] 
            process_noise_v_k_minus_1,      // [1][6] 
            Q_k,                            // [6][6] 
            R_k,                            // [6][6] 
            H_k,                            // [6][6] 
            sensor_noise_w_k,               // [1][6] 
            // returns:
            state_estimate_k,               // [1][6] 
            P_k                             // [6][6] 
        );
        copy_matrix(P_k_minus_1, P_k, 6);
        copy_matrix(state_estimate_k_minus_1, state_estimate_k, 1);
        //print_kalman(i, state_estimate_k, acc, gyro);

        // CORRECT MOTOR VALUES
        float gyro_z = state_estimate_k[0][5];
        if ((gyro_z > 0.2f) || (gyro_z < -0.2f)) {
          powerLeft -= (int)(gyro_z * 7.5f);
        }
        pogobot_motor_set(motorL, powerLeft);
        pogobot_motor_set(motorR, powerRight); 
    }
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);


    printf("Calibration complete:\n\tLeft: %d\n\tRight: %d\n", powerLeft, powerRight);
    *leftMotorVal  = powerLeft;
    *rightMotorVal = powerRight;
    return;
}


void print_kalman(int i, float state_estimate_k[][6], float acc[], float gyro[3]) {
        float accNew[3];
        float gyroNew[3];
        split_array(*state_estimate_k, accNew, gyroNew, 3, 3);
        printf("---------------- Avant filtre[%d] ----------------\n", i);
        printf("Acc = (");
        print_f_list(acc, 3, 100);
        printf(")\n");
        printf("Gyro = (");
        print_f_list(gyro, 3, 100);
        printf(")\n");

        printf("---------------- Après [%d] ----------------\n", i);
        printf("Acc = (");
        print_f_list(accNew, 3, 100);
        printf(")\n");
        printf("Gyro = (");
        print_f_list(gyroNew, 3, 100);
        printf(")\n\n");
}