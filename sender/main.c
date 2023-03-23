/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
**/


/** \file
Pogobot demo source code D2.1, sender side.

This file implements the sender side of demo "D2.1 Testing
communication: introverted listener".

It shows how short a code can be thanks to the API design.

It exercises the following features: RGB LED, low-level infrared
transmission API.

Details:

Robot A continuously emits a specific message in each direction
- each message contains:
  - name of robot (“robot A”)
  - name of IR emitter (“front”, “back”, “left”, “right”)
  - 26 letters (alphabet in lower case)
- RBG LED is always white/black

 */

/* clang-format-ok */

#include <stdio.h>
#include <string.h>
#include "pogobot.h"

#define robot_name "robot A"
#define alphabet "abcdefghijklmnopqrstuvwyz"

// typedef struct rgb8_t
// {
//     uint8_t r;
//     uint8_t g;
//     uint8_t b;
// } rgb8_t;

// typedef struct my_message_t
// {
//     char name[8];
//     rgb8_t color;
// } my_message_t;

// // #FF0000, #00FF00, #0000FF, #FF00FF, #FFFF00, #00FFFF
// #define NB_COLOR 6
// rgb8_t my_color = { .r = 0xFF, .g = 0x00, .b = 0x00 }
// my_message_t message = {}

int main(void) {

    pogobot_init();

    printf("init ok\n");

    //pogobot_led_setColor( 25, 25, 25 );

    // set to max
    pogobot_infrared_set_power(pogobot_infrared_emitter_power_twoThird);

    while(1){  
        pogobot_led_setColor( 0, 0, 255 );
        msleep( 5000 );

        printf( "Taille: %d ",message_length_bytes );
        printf( "%s\n", message);


        pogobot_infrared_sendMessageAllDirection( 0x1234, message,
                                                message_length_bytes);
        pogobot_infrared_update();   
        pogobot_led_setColor(255,0,0);
        msleep( 5000 );
        }

    msleep( 500 );

}
