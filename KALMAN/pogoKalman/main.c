
//#include "pogobot.h"
#include "kalman.h"

int main(int argc, char* argv[]) {
    float acc[3];
    float gyro[3];
    float accNew[3];
    float gyroNew[3];

    // kalman arguments
    float obs_vector_z_k[6];
    float state_estimate_k_minus_1[1][6];
    float P_k_minus_1[6][6];
    float A_k_minus_1[6][6];
    float process_noise_v_k_minus_1[1][6];
    float Q_k[6][6];
    float R_k[6][6];
    float H_k[6][6];
    float sensor_noise_w_k[1][6];
    init_ekf(
        state_estimate_k_minus_1,       // [1][6] 6x1
        P_k_minus_1,                    // [6][6] 6x6
        A_k_minus_1,                    // [6][6] 6x6
        process_noise_v_k_minus_1,      // [1][6] 6x1
        Q_k,                            // [6][6] 6x6
        R_k,                            // [6][6] 6x6
        H_k,                            // [6][6] 6x6
        sensor_noise_w_k                // [1][6] 6x1
    );
    // kalman results
    float state_estimate_k[1][6];
    float P_k[6][6];


    int i=0;
    while (i<1000) {
        pogobot_imu_read(acc, gyro);
        combine_arrays(obs_vector_z_k, acc, gyro, 3, 3);
        
        extendedKalmanFilter(
            z_k_observation_vector,         // [1][6] 6x1
            state_estimate_k_minus_1,       // [1][6] 6x1
            P_k_minus_1,                    // [6][6] 6x6
            A_k_minus_1,                    // [6][6] 6x6
            process_noise_v_k_minus_1,      // [1][6] 6x1
            Q_k,                            // [6][6] 6x6
            R_k,                            // [6][6] 6x6
            H_k,                            // [6][6] 6x6
            sensor_noise_w_k,               // [1][6] 6x1
            // returns:
            state_estimate_k,               // [1][6] 6x1
            P_k                             // [6][6] 6x6
        );

        split_array(state_estimate_k, accNew, gyroNew, 3, 3);

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