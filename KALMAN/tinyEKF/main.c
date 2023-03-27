#include "tiny_ekf.h"

int main() {

    ekf_t ekf;
    ekf_t *ekfptr = &ekf;

    ekf_init(ekfptr, Nsta, Mobs);
    
    
    // Process model is f(x) = x
    fx[0] = this->x[0];
    fx[1] = this->x[1];

    // So process model Jacobian is identity matrix
    F[0][0] = 1;
    F[1][1] = 1;

    // Measurement function simplifies the relationship between state and sensor readings for convenience.
    // A more realistic measurement function would distinguish between state value and measured value; e.g.:
    //   hx[0] = pow(this->x[0], 1.03);
    //   hx[1] = 1.005 * this->x[1];
    //   hx[2] = .9987 * this->x[1] + .001;
    hx[0] = this->x[0]; // Barometric pressure from previous state
    hx[1] = this->x[1]; // Baro temperature from previous state
    hx[2] = this->x[1]; // LM35 temperature from previous state

    // Jacobian of measurement function
    H[0][0] = 1;        // Barometric pressure from previous state
    H[1][1] = 1 ;       // Baro temperature from previous state
    H[2][1] = 1 ;       // LM35 temperature from previous state

    return 0;
}