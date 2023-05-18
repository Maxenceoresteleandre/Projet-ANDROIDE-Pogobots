
#include "pogobot.h"
#include "kalman.h"
#include "display.h"


/*
Example of a program to calibrate the pogobot, then have it move forward forever, while displaying the data from the imu (with and without the Kalman filter).
Notes : 
    - the calibration is not perfect. If the pogobot is mounted backward it might not work. You
    - the calibration only finds optimal values for the motors once. This system doesn't do anything for irregularities in the environment that might disrupt the robot.
*/



int main(int argc, char* argv[]) {
    pogobot_init();
    printf("init ok\n");
    anim_same();

    //######################//
    // 1/ CALIBRATE POGOBOT //
    //######################//
    // the optimal motor values are stored in these variables
    int leftMotorVal;
    int rightMotorVal;

    // calibrate the pogobot with motor values set at around 700 (set them between 0 and 1024 in theory, but between 512 and 1024 in practice¹)
    pogobot_quick_calibrate(700, &leftMotorVal, &rightMotorVal); // will take some time
        // same thing as : 
        // pogobot_calibrate(power=700, startup_duration=500, try_duration=750, number_of_tries=9, correction=50.0f, &leftMotorVal, &rightMotorVal);
            // power : the value you want the motors to be calibrated at (roughly)
            // startup_duration : time (ms) in each calibration try during which the robot doesn't register imu values for calibration
            // try_duration : time (ms) in each calibration try during which the robot registers imu values for calibration
            // number_of_tries : number of times the calibration process is repeated
            // correction : multiplier for the correction applied to the motors at each try. If the pogobot is mounted backward, try setting it at a negative value

    // print the found values
    printf("Calibration ok : powerLeft = %d ; powerRight = %d\n", leftMotorVal, rightMotorVal);



    //##################################//
    // 2/ MOVE POGOBOT AND DISPLAY DATA //
    //##################################//

    float acc[3];
    float gyro[3];

    // Initialize Kalman filter
    float obs_vector_z_k[1][6];
    float state_estimate_k_minus_1[1][6];
    float P_k_minus_1[6][6];
    float A_k_minus_1[6][6];
    float process_noise_v_k_minus_1[1][6];
    float Q_k[6][6];
    float R_k[6][6];
    float H_k[6][6];
    float sensor_noise_w_k[1][6];

    for (int i=0; i<number_of_tries; i++) {
    initExtendedKalmanFilter(
        power,
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

    // pogobot_motor_jump_set(Motor, Value) sets the power of Motor at Value, but ensures that it has enough power to start (jumpstarts the motor if Value is too low²)
    pogobot_motor_jump_set(motorL, leftMotorVal);
    pogobot_motor_jump_set(motorR, rightMotorVal);

    time_reference_t timer;
    pogobot_stopwatch_reset(&timer);
    while (1) {
        // Use Kalman filter
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
        _copyMatrixWidthC(P_k_minus_1, P_k, 6);
        _copyMatrixWidthC(state_estimate_k_minus_1, state_estimate_k, 1);

        // Display collected results
        print_kalman(pogobot_stopwatch_get_elapsed_microseconds(&timer)/1000, state_estimate_k, acc, gyro);
    }
    

    return 1;
}



/*
¹ : it depends on the robot, but a pogobot with motor values lower than 512 will barely move. A power of 700 is pretty good.
² : if you try to use pogobot_motor_set(Motor, Value) with Value < 512 and Motor is at a standstill, it will not work. motors need a some power to start. 
pogobot_motor_jump_set(Motor, Value) ensures that the motor will start (short burst of power if Value is < 500), no matter the desired Value.
*/