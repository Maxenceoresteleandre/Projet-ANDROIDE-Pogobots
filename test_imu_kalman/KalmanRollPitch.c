#include "KalmanRollPitch.h"

void KalmanRollPitch_Init(KalmanRollPitch *kal, float Pinit, float *Q, float *R) {
    kal->phi = 0.0f;
    kal->theta = 0.0f;

    kal->P[0] = Pinit;  kal->P[1] = 0.0f;
    kal->P[2] = 0.0f;   kal->P[3] = Pinit;

    kal->Q[0] = Q[0];   kal->Q[1] = Q[1];
    kal->R[0] = R[0];   kal->R[1] = R[1];   kal->R[2] = R[2];

}

void KalmanRollPitch_Predict(KalmanRollPitch *kal, float *gyro_mesures, float T) {
    float p = gyro_mesures[0];
    float q = gyro_mesures[1];
    float r = gyro_mesures[2];

    float sp = sin(kal->phi);   float cp = cos(kal->phi);
    float tt = tan(kal->theta);

    kal->phi = kal->phi + T*(p+tt*(q*sp+r*cp));
    kal->theta = kal->theta + T*(q*cp-r*sp);

    sp = sin(kal->phi);   cp = cos(kal->phi);
    float st = sin(kal->theta); float ct = cos(kal->theta); tt = st/ct;

    float A[4] = {tt*(q*cp-r*sp), (r*cp+q*sp)*(tt*tt+1.0f), 
                -(r*cp+q*sp), 0.0f};

    float Ptmp[4] = {T*(kal->Q[0] + 2.0f*A[0]*kal->P[0] + A[1]kal->P[1] + A[1]->kal[2]),
                    T*(A[0]*kal->P[1] + A[2]*kal->P[0] + A[1]*kal->P[3] + A[3]*kal->P[1]),
                    T*(A[0]*kal->P[2] + A[2]*kal->P[0] + A[1]*kal->P[3] + A[3]*kal->P[2]),
                    T*(kal->Q[1] + A[2]*kal->P[1] + A[2]*kal->P[2] + 2.0f*A[3*kal->P[3]])};

    kal->P[0] = kal->P[0] + Ptmp[0];    kal->P[1] = kal->P[1] + Ptmp[1];
    kal->P[2] = kal->P[2] + Ptmp[2];    kal->P[3] = kal->P[3] + Ptmp[3];

}

void KalmanRollPitch_Update(KalmanRollPitch *kal, float *acc_mesures) {
    float ax = acc_mesures[0];
    float ay = acc_mesures[1];
    float az = acc_mesures[2];

    float sp = sin(kal->phi);   float cp = cos(kal->phi);
    float st = sin(kal->theta); float ct = cos(kal->theta);

    float h[3] = {g*st, -g*ct*sp, -g*ct*cp};

    float C[6] = {0.0f, g*ct, -g*cp*ct, g*sp*st, g*sp*ct, g*cp*st};

    float G[9] = {C[1]*C[1]*kal->P[3] + kal->R[0], C[2]*C[1]*kal->P[2] + C[3]*C[1]*kal->P[3], C[5]*C[1]*kal->P[3] + C[4]*C[1]*kal->P[2], 
                C[1]*(C[2]*kal->P[1] + C[3]*kal->P[3]), C[2]*(C[2]*kal->P[0] + C[3]*kal->P[2]) + C[3]*(C[2]*kal->P[1] + C[3]*kal->P[3]) + kal->R[1], C[5]*(C[2]*kal->P[1] + C[3]*kal->P[3]) + C[4]*(C[2]*kal->P[0] + C[3]*kal->P[2]),
                C[1]*(C[5]*kal->P[3] + C[4]*kal->P[1]), C[2]*(C[5]*kal->P[2] + C[4]*kal->P[0]) + C[3]*(C[5]*kal->P[3] + C[4]*kal->P[1]), C[5]*(C[5]*kal->P[3] + C[4]*kal->P[1]) + C[4]*(C[5]*kal->P[2] + C[4]*kal->P[0]) + kal->R[2]};

    float GdetInv = 1.0f/(G[0]*G[4]*G[8] - G[0]*G[5]*G[7] - G[1]*G[3]*G[8] + G[1]*G[5]*G[6] + G[2]*G[3]*G[7] - G[2]*G[4]*G[6]);

    float Ginv[9] = {GdetInv*(G[4]*G[8] - G[5]*G[7]), GdetInv*(-G[1]*G[8] + G[2]*G[7]), GdetInv*(G[1]*G[5] - G[2]*G[4]),
                    GdetInv*(-G[3]*G[8] + G[5]*G[6]), GdetInv*(G[0]*G[8] - G[2]*G[6]), GdetInv*(-G[0]*G[5] + G[2]*G[3]),
                    GdetInv*(G[3]*G[7] - G[4]*G[6]), GdetInv*(-G[0]*G[7] + G[1]*G[6]), GdetInv*(G[0]*G[4] - G[1]*G[3])};

    float K[6] = {Ginv[0]*C[1]*kal->P[1] + Ginv[3]*(C[2]*kal->P[0] + C[3]*kal->P[1]) + Ginv[6]*(C[4]*kal->P[0] + C[5]*kal->P[1]), Ginv[1]*C[1]*kal->P[1] + Ginv[4]*(C[2]*kal->P[0] + C[3]*kal->P[1]) + Ginv[7]*(C[4]*kal->P[0] + C[5]*kal->P[1]), Ginv[2]*C[1]*kal->P[1] + Ginv[5]*(C[2]*kal->P[0] + C[3]*kal->P[1]) + Ginv[8]*(C[4]*kal->P[0] + C[5]*kal->P[1]),
                Ginv[0]*C[1]*kal->P[3] + Ginv[3]*(C[2]*kal->P[2] + C[3]*kal->P[3]) + Ginv[6]*(C[4]*kal->P[2] + C[5]*kal->P[3]), Ginv[1]*C[1]*kal->P[3] + Ginv[4]*(C[2]*kal->P[2] + C[3]*kal->P[3]) + Ginv[7]*(C[4]*kal->P[2] + C[5]*kal->P[3]), Ginv[2]*C[1]*kal->P[3] + Ginv[5]*(C[2]*kal->P[2] + C[3]*kal->P[3]) + Ginv[8]*(C[4]*kal->P[2] + C[5]*kal->P[3])};

    float Ptmp[4];
    Ptmp[0] = kal->P[0]*(-K[1]*C[2] - K[2]*C[4] + 1) + kal->P[2]*(-K[0]*C[1] - K[1]*C[3] - K[2]*C[5]);  Ptmp[1] = kal->P[1]*(-K[1]*C[2] - K[2]*C[4] + 1) + kal->P[3]*(-K[0]*C[1] - K[1]*C[3] - K[2]*C[5]);
    Ptmp[2] = kal->P[0]*(-K[4]*C[2] - K[5]*C[4]) + kal->P[2]*(-K[3]*C[1] - K[4]*C[3] - K[5]*C[5] + 1);  Ptmp[3] = kal->P[1]*(-K[4]*C[2] - K[5]*C[4]) + kal->P[3]*(-K[3]*C[1] - K[4]*C[3] - K[5]*C[5] + 1);

    kal->P[0] = kal->P[0] + Ptmp[0];    kal->P[1] = kal->P[1] + Ptmp[1];
    kal->P[2] = kal->P[2] + Ptmp[2];    kal->P[3] = kal->P[3] + Ptmp[3];

    kal->phi = kal->phi+K[0]*(ax-h[0])+K[1]*(ay-h[1])+K[2]*(az-h[2]);
    kal->theta = kal->theta+K[3]*(ax-h[0])+K[4]*(ay-h[1])+K[5]*(az-h[2]);
}