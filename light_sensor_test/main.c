/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.

Prints the values of the different sensors, with different leds lit up.
Better results in relative darkness.

**/

#include "pogobot.h"


int main(void) {

    pogobot_init();
    printf("init ok\n");

    printf("\nTesting lights and light sensors :\n\n");
    printf("  light sensors  || lights\n");
    printf(" rear|right|left \n");

    while (1)
    {
      char l_names[5][20] = {
        "head",
        "belly front",
        "belly right",
        "belly back",
        "belly left"
      };
      for (int led=0; led<=4; led++) {
        pogobot_led_setColors(255, 255, 255, led);
        char b1[6];
        char b2[6];
        char b3[6];
        msleep(500);
        sprintf(b1, "%d", pogobot_photosensors_read(0));
        sprintf(b2, "%d", pogobot_photosensors_read(1));
        sprintf(b3, "%d", pogobot_photosensors_read(2));
        printf("%-5s|%-5s|%-5s||%s\n", b1, b2, b3, l_names[led]);
        msleep(500);
        pogobot_led_setColors(0, 0, 0, led);
        msleep(2000);
      }
      printf("=====|=====|=====\n");
      msleep(5000);
    }

}
