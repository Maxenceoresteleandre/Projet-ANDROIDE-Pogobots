
#include "pogobot.h"

#define g ((float) 9.81f)
#define n 6

typedef struct kalmanSystem {
    float x[n]; // ax, ay, az, gx, gy, gz

    float P[n][n];
    float Q[n][n];
    float R[n][n];
} kalmanSystem;

void kalman_init(kalmanSystem *kal, float Pinit, float Qinit, float *R);
void kalman_predict(kalmanSystem *kal);
void kalman_update(kalmanSystem *kal);
void print_float(float i, int precision);
void print_f_list(float* list, int len, int precision);