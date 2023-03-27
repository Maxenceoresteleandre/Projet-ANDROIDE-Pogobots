/*
 * 
 * ADAPTE DE:
 *
 * TinyEKF: Extended Kalman Filter for embedded processors.
 *
 * Copyright (C) 2015 Simon D. Levy
 *
 * MIT License
 */


//#include "pogobot.h"
#include <stdio.h>

#define Nsta 6
#define Mobs 6
// Nsta : nb de valeurs d'états, Mobs : nb de valeurs observées => supposons m = n, i.e ax, ay, az, gx, gy, gz 


typedef struct {

    double x[Nsta];    /* state vector */

    double P[Nsta*Nsta];  /* prediction error covariance */
    double Q[Nsta*Nsta];  /* process noise covariance */
    double R[Mobs*Mobs];  /* measurement error covariance */

    double G[Nsta*Mobs];  /* Kalman gain; a.k.a. K */

    double F[Nsta*Nsta];  /* Jacobian of process model */
    double H[Mobs*Nsta];  /* Jacobian of measurement model */

    double Ht[Nsta*Mobs]; /* transpose of measurement Jacobian */
    double Ft[Nsta*Nsta]; /* transpose of process Jacobian */
    double Pp[Nsta*Nsta]; /* P, post-prediction, pre-update */

    double fx[Nsta];  /* output of user defined f() state-transition function */
    double hx[Mobs];  /* output of user defined h() measurement function */

    /* temporary storage */
    double tmp0[Nsta*Nsta];
    double tmp1[Nsta*Mobs];
    double tmp2[Mobs*Nsta];
    double tmp3[Mobs*Mobs];
    double tmp4[Mobs*Mobs];
    double tmp5[Mobs]; 

} ekf_t;


void ekf_init(ekf_t* ekf, int n, int m);

/**
  * Runs one step of EKF prediction and update. Your code should first build a model, setting
  * the contents of <tt>ekf.fx</tt>, <tt>ekf.F</tt>, <tt>ekf.hx</tt>, and <tt>ekf.H</tt> to appropriate values.
  * @param ekf pointer to structure EKF 
  * @param z array of measurement (observation) values
  * @return 0 on success, 1 on failure caused by non-positive-definite matrix.
  */
int ekf_step(ekf_t* ekf, double * z, int n, int m);