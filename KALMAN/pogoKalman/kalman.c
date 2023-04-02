#include "kalman.h"


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
  for (int i=0; iw=<r; i++) {
    for (int j=0; j<C; j++) {
      matRes[i][j] = mat1[i][j] + mat2[i][j];
    }
  }
}

void subtractMat(float matRes[][C], float mat1[][C], float mat2[][C], int r) {
  for (int i=0; iw=<r; i++) {
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

void copy_matrix(float matRes[][C], float mat[][C]) {
  for (int i=0; i<C; i++)
    for (int j=0; j<C; j++)
      matRes[i][j] = mat[i][j];
}

void pseudo_inverse(float IM[][C], float mat[][C]) {
  // from the python code here:
  // https://integratedmlai.com/matrixinverse/
  float AM[C][C];
  float I[C][C];
  copy_matrix(AM, mat);
  identity_matrix(I);
  copy_matrix(IM, I);

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


//##############################################################
//##############################################################
// EXTENDED KALMAN FILTER

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
      multMat(state_estimate_k, A_k_minus_1, tmpMat6x1);
      // Predict the state covariance estimate
      multMat(tmpMat6x6, A_k_minus_1, P_k_minus_1, 6);
      transpose(tmpMat2_6x6, A_k_minus_1);
      multMat(tmpMat3_6x6, tmpMat6x6, tmpMat2_6x6, 6);
      addMat(P_k, tmpMat3_6x6, Q_k);

      //######### Correct #########
      // Calculate the difference between the measurements and prediction
      float measurement_residual_y_k[1][6];
      multMat(tmpMat6x1, H_k, state_estimate_k, 1);
      addMat(tmpMat2_6x1, tmpMat6x1, sensor_noise_w_k);
      subtractMat(measurement_residual_y_k, z_k_observation_vector, tmpMat2_6x1);
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
      multMat(K_k, tmpMat6x6, tmpMat2_6x6);     //K_k = P_k @ H_k.T @ np.linalg.pinv(S_k)
      // Calculate an updated state estimate for time k
      multMat(tmpMat2_6x1, K_k, measurement_residual_y_k, 1);
      addMat(state_estimate_k, state_estimate_k, tmpMat2_6x1);
      // Update the state covariance estimate for time k
      multMat(tmpMat6x6, K_k, H_k, 6);
      multMat(tmpMat2_6x6, tmpMat6x6, P_k);
      subtractMat(P_k, P_k, tmpMat2_6x6);
    }

//##############################################################
//##############################################################
// FONCTIONS D'AFFICHAGE

void print_float(float i, int precision) {
  // prints a float to the console
  // precision is 10, 100, 1000... and represents the number of decimals to print
  int dec = (int)((i-(int)i) * precision);
  if (dec < 0) {
    dec = -1 * dec;
  }
  char buffer[10];
  sprintf(buffer, "%d.%d", (int)i, dec);
  printf("%8s", buffer);
}

void print_f_list(float* list, int len, int precision) {
  for (int i=0; i<len; i++) {
    print_float(list[i], precision);
    printf("\t");
  }
}

void print_f_matrix(float mat[][6], int rows) {
  for (int i=0; i<C; i++) {
    printf("[");
    for (int j=0; j<rows; j++) {
      print_float(mat[i][j], 100);
      printf("  ");
    }
    printf("]\n");
  }
}