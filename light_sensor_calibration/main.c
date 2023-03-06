/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
**/

#include "imu_data_filter.h"


#define FIND_MAX_DURATION 7500 // the time given for the pogo to find its maximum amount of light (in milliseconds)
#define MOTOR_POWER 768 // the power of motorRight. MotorLeft will calibrate itself based on motorRight
#define FLOAT_PRECISION 100


void print_light_values(void);
int calibrate_pogo(void);
void go_toward_the_light(int light_val);



void print_light_values(void) {
  char b1[6];
  char b2[6];
  char b3[6];
  sprintf(b1, "%d", pogobot_photosensors_read(0));
  sprintf(b2, "%d", pogobot_photosensors_read(1));
  sprintf(b3, "%d", pogobot_photosensors_read(2));
  printf("photosensors [rear, right, left] = [%-5s, %-5s, %-5s]\n", b1, b2, b3);
  return;
}

void go_toward_the_light(int light_val) {
  pogobot_motor_set(motorL, 768);
  pogobot_motor_set(motorR, 768);
  msleep(100);
  pogobot_motor_set(motorL, 250);
  pogobot_motor_set(motorR, 400);
  pogobot_led_setColors(255, 255, 255, 1);
  while (pogobot_photosensors_read(1) < light_val - 5) {}
  pogobot_motor_set(motorL, motorStop);
  pogobot_motor_set(motorR, motorStop);
  anim_blink(255, 0, 0, 3);
}

int calibrate_pogo(void) {
  time_reference_t timer;
  pogobot_stopwatch_reset(&timer);

  int max_light = 0;
  float time = ((float)pogobot_stopwatch_get_elapsed_microseconds(&timer))/1000.0;
  pogobot_motor_set(motorL, MOTOR_POWER / 2);
  pogobot_motor_set(motorR, MOTOR_POWER);
  pogobot_led_setColors(255, 255, 255, 1);
  while (time < FIND_MAX_DURATION) {
    time = ((float)pogobot_stopwatch_get_elapsed_microseconds(&timer))/1000.0;
    int current_light = pogobot_photosensors_read(1);
    if (current_light > max_light) {
      max_light = current_light;
      // printf("new max light = %d\n", max_light);
    }
  }
  pogobot_motor_set(motorL, motorStop);
  pogobot_motor_set(motorR, motorStop);
  if (max_light < 60) {
    anim_blink(255, 0, 0, 5);
  } else {
    anim_blink(0, 255, 0, 5);
  }
  // printf("final max light = %d\n", max_light);
  blink_int(max_light);


  // TEST TEST TEST TEST TEST TEST
  while (true) {
    set_all_leds(0, 0, 0);
    pogobot_motor_set(motorL, MOTOR_POWER / 2);
    pogobot_motor_set(motorR, MOTOR_POWER);
    msleep(2500);
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
    pogobot_led_setColors(255, 255, 255, 1);
    msleep(2000);
    go_toward_the_light(max_light);
    set_all_leds(0, 0, 0);
    msleep(2000);
    pogobot_led_setColors(255, 255, 255, 1);
    msleep(500);
    blink_int(pogobot_photosensors_read(1));
  }
  // END TEST END TEST END TEST END

  int not_calibrated = 1;
  int motorLval = MOTOR_POWER / 2;
  int previous_min_light = 0;
  int to_add = MOTOR_POWER / 10;
  go_toward_the_light(max_light);
  while (not_calibrated) {
    pogobot_led_setColors(255, 255, 255, 1);
    pogobot_stopwatch_reset(&timer);
    time = ((float)pogobot_stopwatch_get_elapsed_microseconds(&timer))/1000.0;
    int min_light = pogobot_photosensors_read(1);
    pogobot_motor_set(motorL, motorLval);
    pogobot_motor_set(motorR, MOTOR_POWER);
    while (time < 3000) {
      int current_light = pogobot_photosensors_read(1);
      if (current_light < min_light) {
        min_light = current_light;
      }
      time = ((float)pogobot_stopwatch_get_elapsed_microseconds(&timer))/1000.0;
    }
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
    if (min_light > max_light - 10) {
      not_calibrated = 0;
    } else {
      go_toward_the_light(max_light);
      if (previous_min_light < min_light) {
        to_add = to_add * 2;
      } else {
        to_add = (to_add / 2) * -1;
      }
      previous_min_light = min_light;
      motorLval = motorLval + to_add;
    }
  }
  anim_blink(150, 150, 150, 5);
  msleep(500);
  blink_int(motorLval);
  return motorLval;
}

int main(void) {

    pogobot_init();
    printf("init ok\n\n\n");
    anim_blink(150, 150, 0, 3);
    
    msleep(5000);
    calibrate_pogo();
    msleep(2000);
    pogobot_motor_set(motorL, 768);
    pogobot_motor_set(motorR, 768);


    set_all_leds(150, 150, 0);
}
