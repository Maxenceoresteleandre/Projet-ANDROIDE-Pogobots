/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
**/

/* 
Border detection (Pogobots change color (Blue) if they have neighbors on all sides).
Green - 3 sides
Red - 2 sides
Yellow - 1 side
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pogobot.h"


#define message_length_bytes 100
#define F 30
#define FREQEMISSION 100

int main(void) {

    pogobot_init();

    printf("init ok\n");
    srand(pogobot_helper_getRandSeed());
    // pogobot_infrared_emitter_power_max - 3
    // pogobot_infrared_emitter_power_twoThird - 2
    // pogobot_infrared_emitter_power_oneThird - 1
    // pogobot_infrared_emitter_power_null - 0
    pogobot_infrared_set_power( pogobot_infrared_emitter_power_max );

    unsigned char message[message_length_bytes];
    int i;
    for (i=0;i<message_length_bytes;i++){
        message[i] = 'a';
    }
    message[i-1] = '\0';

    time_reference_t t0;
    uint32_t t1;

    int iter = 0;
    int nbMsgRecus = 0;
    while (iter<100) {
        pogobot_stopwatch_reset(&t0);
        pogobot_infrared_update();

        /* read reception fifo buffer */
        if ( pogobot_infrared_message_available() ) {
            printf("msg!!");
            while ( pogobot_infrared_message_available() ) {
                message_t mr;
                pogobot_infrared_recover_next_message( &mr );

                nbMsgRecus++;
            }
        }
        
        if (rand()%100<FREQEMISSION){  
            pogobot_infrared_sendMessageAllDirection( 0x1234, message, message_length_bytes);
        }

        pogobot_infrared_clear_message_queue();
        pogobot_infrared_update();

        iter++;
    
        t1=pogobot_stopwatch_get_elapsed_microseconds(&t0);
        msleep( (1000000/F - t1)/1000 );
    }

    printf("%d %d\n", FREQEMISSION, nbMsgRecus);

}
