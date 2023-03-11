/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
**/

#include "pogobot.h"


int main(void) {

    pogobot_init();
    printf("init ok\n");

    while (1)
    {
      pogobot_led_setColor(0,0,255);
      pogobot_motor_set(motorL, motorFull);
      pogobot_motor_set(motorR, motorStop);
      msleep(3000);

      printf("CA MARCHE HAHAHAHHAHAHHA!!! \n");

      pogobot_led_setColor(255,0,0);
      pogobot_motor_set(motorL, motorStop);
      pogobot_motor_set(motorR, motorFull);
      msleep(3000);

      pogobot_led_setColor(0,255,0);
      pogobot_motor_set(motorL, motorFull);
      pogobot_motor_set(motorR, motorFull);
      msleep(3000);
      
      pogobot_motor_set(motorL, motorStop);
      pogobot_motor_set(motorR, motorStop);
      pogobot_led_setColor(0, 0, 0);
      msleep(500);
      printf("\nTesting lights and light sensors :\n\n");
      printf("  light sensors  || lights\n");
      printf(" rear|right|left \n");
      char l_names[5][20] = {
        "head",
        "belly front",
        "belly right",
        "belly back",
        "belly left"
      };
      for (int led=0; led<=4; led++) {
        for (int i=0; i<=100; i++) {
          pogobot_led_setColors(i, i, i, led);
          msleep(50);
        }
        pogobot_led_setColors(255, 255, 255, led);
        msleep(500);
        char b1[6];
        char b2[6];
        char b3[6];
        sprintf(b1, "%d", pogobot_photosensors_read(0));
        sprintf(b2, "%d", pogobot_photosensors_read(1));
        sprintf(b3, "%d", pogobot_photosensors_read(2));
        msleep(100);
        pogobot_led_setColors(0, 0, 0, led);
        printf("%-5s|%-5s|%-5s||%s\n", b1, b2, b3, l_names[led]);
      }
      printf("=====|=====|=====\n\n");
      msleep(1000);
      printf("end loop\n");
    }

}