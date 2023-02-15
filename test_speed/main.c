/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
**/

#include "pogobot.h"


#define NB_OF_TESTS 5
#define LOOP_END 200
#define LEN_AVG_LIST 9
const int POWERS[] = { 1023, 0, 512, 716, 1023 };


void print_float(float i);
int dec(float i);
void calculate_speed(float acc[3], float prev_speed[3], float delta, float speed[3]);
void update_speed(float prev_speed[3], float speed[3]);
void print_f_list(float* list, int len);



void print_f_list(float* list, int len) {
  for (int i=0; i<len; i++) {
    printf(", ");
    print_float(list[i]);
  }
}

void calculate_speed(float acc[3], float prev_speed[3], float delta, float speed[3]) {
  for (int i=0; i<3; i++) {
    speed[i] = prev_speed[i] + acc[i] * delta;
  }
}

void update_speed(float prev_speed[3], float speed[3]) {
  for (int i=0; i<3; i++) {
    prev_speed[i] = speed[i];
  }
}

void print_float(float i) {
  //prints a float to the console
  int dec = (int)((i-(int)i) * 1000);
  if (dec < 0) {
    dec = -1 * dec;
  }
  char buffer[10];
  sprintf(buffer, "%d.%d", (int)i, dec);
  printf("%8s", buffer);
}

int dec(float i) {
  int d = (int)((i-(int)i) * 1000);
  if (d < 0) {
    d = -1 * d;
  }
  return d;
}

int main(void) {

    pogobot_init();
    printf("init ok\n\n\n");
    printf("========================\nTest : Speed\n========================\n");

    

    float avg[NB_OF_TESTS][LEN_AVG_LIST];
    for (int i=0; i<NB_OF_TESTS; i++) {
      for (int j=0; j<LEN_AVG_LIST; j++) {
        avg[i][j] = 0.0;
      }
    }

    printf("Mpow|     accX,     accY,     accZ,    gyroX,    gyroY,    gyroZ,   speedX,   speedY,   speedZ\n");
    msleep(500);
    float acc[3];
    float gyro[3];
    float prev_speed[3];
    float speed[3];

    msleep(7500);

    time_reference_t timer;
    pogobot_stopwatch_reset(&timer);

    for (int pow=0; pow<NB_OF_TESTS; pow++) {
      int motorPower = POWERS[pow];
      int lum = (int)(255/1023.0) * motorPower;
      for (int led=0; led<=4; led++){
        pogobot_led_setColors(lum, lum, lum, led);
      }

      for (int j=0; j<3; j++){
        acc[j]  = 0.0;
        gyro[j] = 0.0;
        prev_speed[j] = 0.0;
        speed[j] = 0.0;
      }

      pogobot_motor_set(motorL, motorPower);
      pogobot_motor_set(motorR, motorPower);

      for (int i=0; i<LOOP_END; i++) {
        pogobot_imu_read(acc, gyro);


        int time = pogobot_stopwatch_get_elapsed_microseconds(&timer);
        //printf("\ntime = %ld\n\n", time);
        float delta = (float)time / 1000.0;
        calculate_speed(acc, prev_speed, delta, speed);
        pogobot_stopwatch_reset(&timer);

        for (int j=0; j<3; j++){
          avg[pow][j]   = avg[pow][j]   + (acc[j]   / ((float)LOOP_END));
          avg[pow][j+3] = avg[pow][j+3] + (gyro[j]  / ((float)LOOP_END));
          avg[pow][j+6] = avg[pow][j+6] + (speed[j] / ((float)LOOP_END));
        }
        
        char buffer[10];
        sprintf(buffer, "%d", motorPower);
        printf("%4s", buffer);
        print_f_list(acc, 3);
        print_f_list(gyro, 3);
        print_f_list(speed, 3);
        printf("\n");
        msleep(10);  //MAYBE THIS FUCKS UP EVERYTHING WITH THE IMU READS IDK
      }
      
      pogobot_motor_set(motorL, motorStop);
      pogobot_motor_set(motorR, motorStop);
      pogobot_led_setColor(0, 0, 0);
      msleep(1500);
    }

    printf("\n========================\n");
    printf("Averages : \n");
    printf("Mpow|     accX,     accY,     accZ,    gyroX,    gyroY,    gyroZ,   speedX,   speedY,   speedZ\n");
    for (int i=0; i<NB_OF_TESTS; i++) {
      char buffer[10];
      sprintf(buffer, "%d", POWERS[i]);
      printf("%4s", buffer);
      for (int j=0; j<LEN_AVG_LIST; j++) {
        float a = avg[i][j];
        printf(", ");
        print_float(a);
      }
      printf("\n");
    }


    printf("\n\n========================\nEnd of speed test\n");
}
