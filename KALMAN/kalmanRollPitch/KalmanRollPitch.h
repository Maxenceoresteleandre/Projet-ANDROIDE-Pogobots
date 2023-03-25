
#include "pogobot.h"
#include <math.h>

#define g ((float) 9.81f)

typedef struct KalmanRollPitch {
    float phi;
    float theta;

    float P[4];
    float Q[2];
    float R[3];
} KalmanRollPitch;

void KalmanRollPitch_Init(KalmanRollPitch *kal, float Pinit, float *Q, float *R);
void KalmanRollPitch_Predict(KalmanRollPitch *kal, float *gyro_mesures, float T);
void KalmanRollPitch_Update(KalmanRollPitch *kal, float *acc_mesures);
void print_float(float i, int precision);
void print_f_list(float* list, int len, int precision);