#include <stdio.h>
#include "KalmanRollPitch.h"

#define ALPHAGYR 0.01f
#define ALPHAACC 0.1f

#define KALMAN_P_INIT 0.1f
#define KALMAN_Q 0.001f
#define KALMAN_R 0.011f

#define KALMAN_PREDICT_PERIOD_MS 10
#define KALMAN_UPDATE_PERIOD_MS 100


int main() {
    float gyrPrev[3] = {0.0f, 0.0f, 0.0f};
    float accPrev[3] = {0.0f, 0.0f, 0.0f};

    while (1) {
        float acc[3];
        float gyro[3];
        float accNew[3];
        float gyroNew[3];
        pogobot_imu_read(acc, gyro);

        // premier filtre
        accNew[0] = ALPHAACC*accPrev[0] + (1.0f-ALPHAACC)*acc[0];
        accNew[1] = ALPHAACC*accPrev[1] + (1.0f-ALPHAACC)*acc[1];
        accNew[2] = ALPHAACC*accPrev[2] + (1.0f-ALPHAACC)*acc[2];

        gyroNew[0] = ALPHAGYR*gyrPrev[0] + (1.0f-ALPHAGYR)*gyro[0];
        gyroNew[1] = ALPHAGYR*gyrPrev[1] + (1.0f-ALPHAGYR)*gyro[1];
        gyroNew[2] = ALPHAGYR*gyrPrev[2] + (1.0f-ALPHAGYR)*gyro[2];

        KalmanRollPitch ekf;

        KalmanRollPitch_Predict(&ekf, gyroNew, 0.001f*KALMAN_PREDICT_PERIOD_MS);
        KalmanRollPitch_Update(&ekf, accNew);
    }

    return 1;
}