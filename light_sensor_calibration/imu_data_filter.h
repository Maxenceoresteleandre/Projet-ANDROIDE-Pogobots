/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
**/

#include "pogobot.h"

#define CALIBRATION_LOOP_ITERATIONS 3000

// cute animations
void set_all_leds(int r, int g, int b);
void anim_blink(int r, int g, int b, int number_of_blinks);
void anim_same(void);
void blink_int(int number);

// print functions
void print_float(float i, int precision);
int dec(float i, int precision);
void print_f_list(float* list, int len, int precision);

// calibrate functions
void calibrate_constant_bias(float* acc_bias, float* gyro_bias);
void correct_bias(float* data, float* bias);
