#include "kalman.h"

// ETAPE D'INITIALISATION DE EKF
void kalman_init(kalmanSystem *kal, float PinitAcc, float PinitGyro, 
float QinitAcc, float QinitGyro, float RinitAcc, float RinitGyro) {
    for (int i=0;i<n;i++){
      kal->x[i] = (i == 2) ? -g : 0.0f;
    }

    // P
    for (int i=0;i<n;i++){
      kal->P[i][i] = (i < n/2) ? PinitAcc : PinitGyro;
    }

    // Q
    for (int i=0;i<n;i++){
      kal->Q[i][i] = (i < n/2) ? QinitAcc : QinitGyro;
    }

    // R
    for (int i=0;i<n;i++){
      kal->R[i][i] = (i < n/2) ? RinitAcc : RinitGyro;
    }

    // H
    for (int i = 0; i; i++) {
      for (int j = 0; j < 6; j++) {
        kal->H[i][j] = (i == j) ? 1.0f : 0.0f;
      }
    }

}

// ETAPE DE PREDICTION
void state_transition_jacobian(float *x, float *F) {
    for (int i=0;i<n;i++) {
		for (int j=0;j<n;j++) {
			F[i][j] = (i == j) ? 1.0 : 0.0;
		}
    }
}

void prediction_step(kalmanSystem *kal) {
	// P(k) = F(k-1) * P(k-1) * F'(k-1) + Q(k)

    // transition function f(x(k-1), u(k)) sauf que u est ignoré
    float fx[n];
    for (int i=0;i<n;i++) {
      	fx[i] = kal->x[i];
    }

    // jacobian F = df/dx(x(k-1))
    float F[6][6] = {{1, 0, 0, 0, 0, 0},
                     {0, 1, 0, 0, 0, 0},
                     {0, 0, 1, 0, 0, 0},
                     {0, 0, 0, 1, 0, 0},
                     {0, 0, 0, 0, 1, 0},
                     {0, 0, 0, 0, 0, 1}};

	// on peut calculer P (je l'ai fait avec Octave)
	kal->P = {}
    
}

// ETAPE DE CORRECTION
void update_step(float *x_pred, float *P_pred, float *z, float *R, float *H, float *K, float *x, float *P) {
    // measurement function h(x(k))
    float fx[n];
    for (int i=0;i<n;i++) {
      	fx[i] = kal->x[i];
    }

    // Define the Jacobian matrix of the measurement function dh/dx(x(k))
    measurement_jacobian(x_pred, H);

    // Compute the innovation vector y(k) = z(k) - h(x'(k))
    float y[3];
    matrix_subtract(z, H, y, 3, 1);  // y = z - H

    // Compute the innovation covariance S(k) = H(k) * P'(k) * H'(k) + R(k)
    float S[9];
    matrix_multiply(H, P_pred, S, 3, 6, 3);  // S = H * P'
    matrix_multiply(S, H, S, 3, 3, 6);  // S = S * H'
    matrix_add(S, R, S, 3, 3);  // S = S + R

    // Compute the Kalman gain K(k) = P'(k) * H'(k) * S^-1(k)
    matrix_invert(S, 3);
    matrix_multiply(P_pred, H, K, 6, 3, 3);  // K = P' * H'
    matrix_multiply(K, S, K, 6, 3, 3);  // K = K * S^-1

    // Compute the updated state x(k) = x'(k) + K(k) * y(k)
    matrix_multiply(K, y, x, 6, 1, 3);  // x = K * y
    matrix_add(x_pred, x, x, 6, 1);  // x = x' + x

    // Compute the updated covariance matrix P(k) = (I - K(k) * H(k)) * P'(k)
    float I[36] = {0};
    matrix_set_identity(I, 6, 6);  // Set I to the identity matrix
    float K_H[18];
    matrix_multiply(K, H, K_H, 6, 3, 6);  // K_H = K * H
    matrix_subtract(I, K_H, K_H, 6, 6);  // K_H = I - K * H
    matrix_multiply(K_H, P_pred, P, 6, 6, 6);  // P = K_H * P'
}


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