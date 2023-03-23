#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "KalmanRollPitch.h"

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

    printf("%s\n", argv[1]);
    FILE* inputFile = fopen( argv[1], "r" );
    if ( inputFile == NULL ) {
        printf( "Cannot open file %s\n", argv[0] );
        exit( -1 );
    }
    
    char *buffer = (char *) malloc( MAX_LENGTH*sizeof(char) );
    while ( ! feof( inputFile ) ) {
        fgets( buffer, MAX_LENGTH, inputFile );
        if ( ferror( inputFile ) ) {
            fprintf( stderr, "Reading error" );
            break;
        }  

        //on recupere les donnees imu des fichiers 
        buffer[strlen(buffer)-1] = '\0';     
        char type[5];
        int x, y, z;
        sscanf(buffer, "%s X %d Y %d  Z %d\n", type, &x, &y, &z);

        // analyse du texte
        float acc[3];
        float gyro[3];
        if (strcmp(buffer, "Acc") == 0){
            acc[0] = x;
            acc[1] = y;
            acc[2] = z;
        } else {
            gyro[0] = x;
            gyro[1] = y;
            gyro[2] = z;
        }

        float accNew[3];
        float gyroNew[3];
        //pogobot_imu_read(acc, gyro);

        // premier filtre
        accNew[0] = ALPHAACC*accPrev[0] + (1.0f-ALPHAACC)*acc[0];
        accNew[1] = ALPHAACC*accPrev[1] + (1.0f-ALPHAACC)*acc[1];
        accNew[2] = ALPHAACC*accPrev[2] + (1.0f-ALPHAACC)*acc[2];

        gyroNew[0] = ALPHAGYR*gyrPrev[0] + (1.0f-ALPHAGYR)*gyro[0];
        gyroNew[1] = ALPHAGYR*gyrPrev[1] + (1.0f-ALPHAGYR)*gyro[1];
        gyroNew[2] = ALPHAGYR*gyrPrev[2] + (1.0f-ALPHAGYR)*gyro[2];

        KalmanRollPitch ekf;
        float Q[2] = {KALMAN_Q, KALMAN_Q};
        float R[3] = {KALMAN_R, KALMAN_R, KALMAN_R}; // pas très sure de tout ce bazar
        KalmanRollPitch_Init(&ekf, KALMAN_P_INIT, Q, R);
        KalmanRollPitch_Predict(&ekf, gyroNew, 0.001f*KALMAN_PREDICT_PERIOD_MS);
        KalmanRollPitch_Update(&ekf, accNew);

    }

    free( buffer );
    fclose( inputFile );


    return 1;
}