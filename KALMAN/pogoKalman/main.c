
//#include "pogobot.h"
#include "kalman.h"

#define ALPHAGYR 0.01f
#define ALPHAACC 0.1f

#define KALMAN_P_INIT 0.1f
#define KALMAN_Q 0.001f
#define KALMAN_R 0.011f

#define KALMAN_PREDICT_PERIOD_MS 10
#define KALMAN_UPDATE_PERIOD_MS 100

#define MAX_LENGTH 50



int main(int argc, char* argv[]) {
    float gyrPrev[3] = {0.0f, 0.0f, 0.0f};
    float accPrev[3] = {0.0f, 0.0f, 0.0f};
    KalmanRollPitch ekf;
    KalmanRollPitch *EKFptr = &ekf;
    float Q[2] = {KALMAN_Q, KALMAN_Q};
    float R[3] = {KALMAN_R, KALMAN_R, KALMAN_R}; // pas très sure de tout ce bazar
    KalmanRollPitch_Init(EKFptr, KALMAN_P_INIT, Q, R);


    int i=0;
    while (i<1000) {
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

        
        //KalmanRollPitch_Predict(EKFptr, gyroNew, 0.001f*KALMAN_PREDICT_PERIOD_MS);
        //KalmanRollPitch_Update(EKFptr, accNew);


        printf("---------------- Avant filtre ----------------\n");
        printf("Acc = (");
        print_f_list(acc, 3, 100);
        printf(")\n");
        printf("Gyro = (");
        print_f_list(gyro, 3, 100);
        printf(")\n");

        printf("---------------- Après filtre ----------------\n");
        printf("Acc = (");
        print_f_list(accNew, 3, 100);
        printf(")");
        printf("Gyro = (");
        print_f_list(gyroNew, 3, 100);
        printf(")");

        i++;
    }


    return 1;
}