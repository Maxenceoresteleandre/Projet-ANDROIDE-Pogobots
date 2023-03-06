/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
**/

#include "imu_data_filter.h"

void set_all_leds(int r, int g, int b) {
  for (int led=0; led<=4; led++){
    pogobot_led_setColors(r, g, b, led);
  }
  return;
}

void anim_same(void) {
  for (int i=0; i<3; i++) {
    set_all_leds(255, 0, 0);
    msleep(150);
    set_all_leds(0, 255, 0);
    msleep(150);
    set_all_leds(0, 0, 255);
    msleep(150);
  }
  set_all_leds(255, 255, 255);
  msleep(500);
  set_all_leds(0, 0, 0);
  return;
}

void anim_blink(int r, int g, int b, int number_of_blinks) {
  for (int i=0; i<number_of_blinks; i++) {
    set_all_leds(r, g, b);
    msleep(250);
    set_all_leds(0, 0, 0);
    msleep(250);
  }
  return;
}

void blink_int(int number) {
  int num = number;
  int n1000 = num / 1000;
  num = num % 1000;
  int n100 = num / 100;
  num = num % 100;
  int n10 = num / 10;
  num = num % 10;
  int n1 = (num);
  //printf("n1000 = %d\nn100 = %d\nn10 = %d\nn1 = %d\n", n1000, n100, n10, n1);
  anim_blink(255, 0, 255, n1000);
  msleep(800);
  anim_blink(150, 0, 150, n100);
  msleep(800);
  anim_blink(0, 150, 150, n10);
  msleep(800);
  anim_blink(0, 0, 150, n1);
  msleep(800);
  set_all_leds(255, 0, 255);
  msleep(3000);
  set_all_leds(0, 0, 0);
}

void calibrate_constant_bias(float* acc_bias, float* gyro_bias) {
  anim_same();
  pogobot_motor_set(motorL, motorStop);
  pogobot_motor_set(motorR, motorStop);
  float acc[3];
  float gyro[3];
  for (int i=0; i<3; i++) {
    acc_bias[i]  = 0.0;
    gyro_bias[i] = 0.0;
  }
  for (int i=0; i<CALIBRATION_LOOP_ITERATIONS; i++) {
    pogobot_imu_read(acc, gyro);
    for (int j=0; j<3; j++) {
      acc_bias[j]  += acc[j];
      gyro_bias[j] += gyro[j];
    }
  }
  for (int j=0; j<3; j++) {
    acc_bias[j] = acc_bias[j] / (float)CALIBRATION_LOOP_ITERATIONS;
    gyro_bias[j] = gyro_bias[j] / (float)CALIBRATION_LOOP_ITERATIONS;
  }
  acc_bias[2] = acc_bias[2] + (float)9.81; // gravité lol
  anim_blink(0, 255, 0, 4);
  return;
}

void correct_bias(float* data, float* bias) {
  for (int i=0; i<3; i++) {
    data[i] = data[i] - bias[i];
  }
  return;
}


void print_f_list(float* list, int len, int precision) {
  for (int i=0; i<len; i++) {
    printf(", ");
    print_float(list[i], precision);
  }
}

void print_float(float i, int precision) {
  // prints a float to the console
  // precision is 10, 100, 1000... and represents the number of decimals to print
  int dec = (int)((i-(int)i) * precision);
  if (dec < 0) {
    dec = -1 * dec;
  }
  char buffer[10];
  sprintf(buffer, "%d.%d", (int)i, dec);
  printf("%8s", buffer);
}

int dec(float i, int precision) {
  int d = (int)((i-(int)i) * precision);
  if (d < 0) {
    d = -1 * d;
  }
  return d;
}
