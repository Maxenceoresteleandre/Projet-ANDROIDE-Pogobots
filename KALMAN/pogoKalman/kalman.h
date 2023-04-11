


// CORRECTION is the amount of correction done at each step of the calibration
// if your Pogo is mounted backwards, give it a negative value
#define CORRECTION 50.0f
#define GRAVITY 9.81

// don't touch these
#define C 6
#define PSEUDO_INVERSE_MAX_ITER 30

// multiply two matrices
// mat1 : 6x6 or 1x6 (rows x colums)
// mat2 : 6x6
void _multMatrixWidthC(
    float matRes[][C], // the matrix in which is saved the result
    float mat1[][C],       
    float mat2[][C], 
    int r1);            // number of rows of the first matrix (1 or 6)

// add two matrices
// must have the same size (or runtime error)
void _addMatrixWidthC(
    float matRes[][C], 
    float mat1[][C], 
    float mat2[][C], 
    int r);

// subtract mat2 from mat1 (matRes = mat1 - mat2)
// must have the same size (or runtime error)
void _subtractMatrixWidthC(
    float matRes[][C], 
    float mat1[][C], 
    float mat2[][C], 
    int r);

// transpose a [C][C] matrix mat into the [C][C] matrix matRes
// only works on [C][C] matrices lol
void _transposeMatrixCbyC(float matRes[][C], float mat[][C]);

// pseudo invert a [C][C] matrix
// doesn't check for squareness nor non_singulareness cuz i'm still lazyyyyy
void _pseudoInverseMatrixCbyC(float IM[][C], float mat[][C]);

// makes matRes (CxC matrix) the idendity matrix
void _identityMatrixCbyC(float matRes[][C]);

// copy mat to matRes
void _copyMatrixWidthC(float matRes[][C], float mat[][C], int r);








// combine arr1[len1] and arr2[len2] into res[len1+len2] (res = arr1 + arr2 in python)
void combine_arrays(float res[], float arr1[], float arr2[], int len1, int len2);

// split base[len1+len2] into arr1[len1] and arr2[len2] by putting the len1 first elements into arr1 and len1+1 to len1+len2 into arr2
void split_array(float base[], float arr1[], float arr2[], int len1, int len2);

// prints to the console the IMU values with and without the Kalman filter
void print_kalman(int i, float state_estimate_k[][6], float acc[], float gyro[3]);







// calibrate the pogobot's motor values with the Kalman filter; durations in milliseconds
void pogobot_calibrate(int power, int startup_duration, int try_duration, int number_of_tries, int* leftMotorVal, int* rightMotorVal);

// calibrate with fewer parameters, duration in milliseconds
void pogobot_quick_calibrate(int power, int* leftMotorVal, int* rightMotorVal);

// code for the extended Kalman Filter algorithm 
// reproduction from the kalman.py file
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
    );

//void init_kalman();
void initExtendedKalmanFilter(
    int power,
    float state_estimate_k_minus_1[][C],       // [1][6] 6x1
    float P_k_minus_1[][C],                    // [6][6] 6x6
    float A_k_minus_1[][C],                    // [6][6] 6x6
    float process_noise_v_k_minus_1[][C],      // [1][6] 6x1
    float Q_k[][C],                            // [6][6] 6x6
    float R_k[][C],                            // [6][6] 6x6
    float H_k[][C],                            // [6][6] 6x6
    float sensor_noise_w_k[][C]                // [1][6] 6x1
    );


