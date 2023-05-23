/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.

 Displays various IMU data. 
**/

#include "imu_data_filter.h"


#define FLOAT_PRECISION 100
#define VARIANCE_ON 0 // 0=acceleration, 1=gyro, 2=speed // la variance requiert la sauvegarde de tout l'array donc economies ? ptet pas necessaire mais precaution
#define LOOP_ITERATIONS 400     // le nombre d'itérations de tests
#define DELAY_BETW_ITERATIONS -1 // valeur negative = pas de délai
#define NB_OF_TESTS 1    // la longueur déclarée de l'array POWERS : 1 c'est bien comme ça ça fait juste le 1er test
const int POWERS[] = { 512, 0, 512, 768, 1024 };   // les puissances avec lesquelles les moteurs sont testés
#define LEN_AVG_LIST 9


void calculate_speed(float acc[3], float prev_speed[3], float delta, float speed[3]);
void update_speed(float prev_speed[3], float speed[3]);



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

int main(void) {

    pogobot_init();
    printf("init ok\n\n\n");
    printf("========================\nTest : Speed\n========================\n");

    

    float sum[NB_OF_TESTS][LEN_AVG_LIST];
    float avg[NB_OF_TESTS][LEN_AVG_LIST];
    int has_variance = 1;
    if (VARIANCE_ON < 0) {
      has_variance = 0;
    }
    float vrnc_data[3][LOOP_ITERATIONS * has_variance];
    for (int i=0; i<NB_OF_TESTS; i++) {
      for (int j=0; j<LEN_AVG_LIST; j++) {
        avg[i][j] = 0.0;
      }
    }

    msleep(500);
    float acc[3];
    float gyro[3];
    float acc_bias[3];
    float gyro_bias[3];
    float prev_speed[3];
    float speed[3];

    calibrate_constant_bias(acc_bias, gyro_bias);

    msleep(500);
    printf("----time, Mpow,     accX,     accY,     accZ,    gyroX,    gyroY,    gyroZ\n");

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

      for (int i=0; i<LOOP_ITERATIONS; i++) {
        pogobot_imu_read(acc, gyro);
        correct_bias(acc, acc_bias);
        correct_bias(gyro, gyro_bias);


        int time = pogobot_stopwatch_get_elapsed_microseconds(&timer);
        //printf("\ntime = %ld\n\n", time);
        float delta = (float)time / 1000.0;
        calculate_speed(acc, prev_speed, delta, speed);
        //pogobot_stopwatch_reset(&timer);

        for (int j=0; j<3; j++){
          avg[pow][j]   = avg[pow][j]   + acc[j]   ;
          avg[pow][j+3] = avg[pow][j+3] + gyro[j]  ;
          avg[pow][j+6] = avg[pow][j+6] + speed[j] ;
          
          if (VARIANCE_ON == 0) {
            vrnc_data[j][i]    = acc[j];
          } else if (VARIANCE_ON == 1) {
            vrnc_data[j][i]    = gyro[j];
          } else if (VARIANCE_ON == 2) {
            vrnc_data[j][i]    = speed[j];
          }
        }

        char motor_pow_txt[10];
        sprintf(motor_pow_txt, "%d", motorPower);
        print_float(delta, FLOAT_PRECISION);
        printf(",%5s", motor_pow_txt);
        print_f_list(acc, 3, FLOAT_PRECISION);
        print_f_list(gyro, 3, FLOAT_PRECISION);
        //print_f_list(speed, 3);
        printf("\n");
        if (DELAY_BETW_ITERATIONS > 0) {
          msleep(DELAY_BETW_ITERATIONS); //MAYBE THIS FUCKS UP EVERYTHING WITH THE IMU READS IDK
        }
      }
      
      pogobot_motor_set(motorL, motorStop);
      pogobot_motor_set(motorR, motorStop);
      pogobot_led_setColor(0, 0, 0);
      msleep(500);
    }

    anim_blink(0, 255, 0, 4);
    printf("\nTests done over %d iterations\n", LOOP_ITERATIONS);
    printf("========================\n");
    printf("Biases : \n");
    printf("Mpow|     accX,     accY,     accZ,    gyroX,    gyroY,    gyroZ\n    ");
    print_f_list(acc_bias, 3, FLOAT_PRECISION);
    print_f_list(gyro_bias, 3, FLOAT_PRECISION);
    printf("\nAverages : \n");
    printf("Mpow|     accX,     accY,     accZ,    gyroX,    gyroY,    gyroZ,   speedX,   speedY,   speedZ\n");
    for (int i=0; i<NB_OF_TESTS; i++) {
      char buffer[10];
      sprintf(buffer, "%d", POWERS[i]);
      printf("%4s", buffer);
      for (int j=0; j<LEN_AVG_LIST; j++) {
        sum[i][j] = avg[i][j];
        avg[i][j] = avg[i][j] / ((float)LOOP_ITERATIONS);
        printf(", ");
        print_float(avg[i][j], FLOAT_PRECISION);
      }
      printf("\n");
    }
    for (int i=0; i<NB_OF_TESTS; i++) {
      printf("sums");
      for (int j=0; j<LEN_AVG_LIST; j++) {
        printf(", ");
        print_float(sum[i][j], FLOAT_PRECISION);
      }
      printf("\n");
    }
    if (VARIANCE_ON >= 0) {
      int vrnc_array_offset;
      if (VARIANCE_ON == 0) {
        vrnc_array_offset = 0;
        printf("vrnc");
      } else if (VARIANCE_ON == 1) {
        vrnc_array_offset = 3;
        printf("variance:                         ");
      } else if (VARIANCE_ON == 2) {
        vrnc_array_offset = 6;
        printf("variance:                                                       ");
      }
      for (int item=vrnc_array_offset; item<vrnc_array_offset+3; item++) {
        float variance = 0.0;
        for (int i=0; i<LOOP_ITERATIONS; i++) {
          float s = (vrnc_data[item-vrnc_array_offset][i] - avg[0][item]);
          variance = variance + s * s;
        }
        variance = variance / ((float)LOOP_ITERATIONS);
        printf(", ");
        print_float(variance, FLOAT_PRECISION);
      }
    }

    printf("\n");
    


    printf("\n\n========================\nEnd of imu test\n");
    anim_blink(0, 0, 255, 10);
}
