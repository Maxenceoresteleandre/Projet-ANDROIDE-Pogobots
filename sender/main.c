/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
**/

#include <stdio.h>
#include <string.h>
#include "pogobot.h"

#define message_length_bytes 375

int main(void) {
    pogobot_init();

    printf("init ok\n");

    // pogobot_infrared_emitter_power_max - 3
    // pogobot_infrared_emitter_power_twoThird - 2
    // pogobot_infrared_emitter_power_oneThird - 1
    // pogobot_infrared_emitter_power_null - 0
    pogobot_infrared_set_power(pogobot_infrared_emitter_power_twoThird);

    unsigned char message[message_length_bytes];
    int i;
    for (i=0;i<message_length_bytes;i++){
        message[i] = 'a';
    }
    message[i-1] = '\0';

    while(1){
        // printf( "Taille: %d ",message_length_bytes);
        // printf( "%s\n", message);

        pogobot_led_setColor( 255, 255, 255 );
        msleep( 500 );
        // 0 success
        bool b = pogobot_infrared_sendMessageAllDirection( 0x1234, message,message_length_bytes);
        if (b == 0){
            printf("SENDING %d...\n",message_length_bytes);
            pogobot_infrared_update();   
            pogobot_led_setColor( 255, 0, 0 );
        }
        msleep( 500 );
    }
    msleep( 500 );

}
