

// don't use any of these functions outside of kalman lol
// they are designed for speed cuz pogos have smol brain
// so they only work with specific arguments (mostly 6x6 or 6x1 matrices)
// and also im lazyyyyy
// so don't try to use anything in a pogoprogram beyond:
//      - kalman
//      - print_float
// the rest probably won't work or will blow up in your face or will 
// cause bugs and then pogo will be sad :(
// or ded
// ded pogo
// and then programer will be sad :/




// Edit MACROs here, according to your Matrix Dimensions for
// mat1[r1][C] and mat2[r2][C]
// r1 can anything but r2 must be equal to C
#define C 6
#define R2 6
#define PSEUDO_INVERSE_MAX_ITER 30
#define GRAVITY 9.81

// multiply two matrices
// mat1 : 6x6 or 1x6 (rows x colums)
// mat2 : 6x6
void multMat(
    float matRes[][C], // the matrix in which is saved the result
    float mat1[][C],       
    float mat2[][C], 
    int r1);            // number of rows of the first matrix (1 or 6)


// adds two matrices
// must have the same size (or runtime error)
void addMat(
    float matRes[][C], 
    float mat1[][C], 
    float mat2[][C], 
    int r);


// subtracts mat2 from mat1 (matRes = mat1 - mat2)
// must have the same size (or runtime error)
void subtractMat(
    float matRes[][C], 
    float mat1[][C], 
    float mat2[][C], 
    int r);


// transpose a [C][C] matrix mat into the [C][C] matrix matRes
// only works on [C][C] matrices lol
void transpose(float matRes[][C], float mat[][C]);


// pseudo invert a [C][C] matrix
// doesn't check for squareness nor non_singulareness cuz i'm still lazyyyyy
void pseudo_inverse(float IM[][C], float mat[][C]);


// makes matRes (CxC matrix) the idendity matrix
void identity_matrix(float matRes[][C]);


// copy mat to matRes
void copy_matrix(float matRes[][C], float mat[][C], int r)


void print_f_matrix(float mat[][6], int rows);


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
void init_ekf(
    float state_estimate_k_minus_1[][C],       // [1][6] 6x1
    float P_k_minus_1[][C],                    // [6][6] 6x6
    float A_k_minus_1[][C],                    // [6][6] 6x6
    float process_noise_v_k_minus_1[][C],      // [1][6] 6x1
    float Q_k[][C],                            // [6][6] 6x6
    float R_k[][C],                            // [6][6] 6x6
    float H_k[][C],                            // [6][6] 6x6
    float sensor_noise_w_k[][C]                // [1][6] 6x1
    );

void print_float(float i, int precision);

void print_f_list(float* list, int len, int precision);
