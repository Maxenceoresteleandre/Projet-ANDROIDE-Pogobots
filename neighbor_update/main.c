/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
**/

/* Border detection (Pogobots change color (Blue) if they don't have neighbors on at least one side). */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pogobot.h"


#define message_length_bytes 100
#define F 30

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

    // time_reference_t timers[4];
    // for (int i=0; i<4; i++) {
    //     pogobot_stopwatch_reset(&timers[i]);
    // }

    time_reference_t t0;
    uint32_t t1;

    while (1)
    {
        int sender_id[4] = {-1,-1,-1,-1};
        pogobot_stopwatch_reset(&t0);
        pogobot_infrared_update();
        /* read reception fifo buffer */
        if ( pogobot_infrared_message_available() )
        {
            while ( pogobot_infrared_message_available() )
            {
                message_t mr;
                pogobot_infrared_recover_next_message( &mr );

                //printf("len : %d,%s\n", strlen(mr.payload), mr.payload);
                int s_id = mr.header._sender_id;
                int i=0;
                for (i=0; i<4; i++){
                    if (sender_id[i]==-1){
                        sender_id[i]=s_id;
                        break;
                    }
                    if (s_id == sender_id[i]){
                        // pogobot_stopwatch_reset(&timers[i]);
                        break;
                    }
                }  

                
            }
        }

        printf("%d,%d,%d,%d\n",sender_id[0],sender_id[1],sender_id[2],sender_id[3]);
        int nb_voisins = 0;
        for (int i=0; i<4; i++){
            if (sender_id[i]!=-1) {
                nb_voisins += 1;
            }
        }
        // switch (nb_voisins){
        //     case 0:
        //         pogobot_led_setColor( 0, 0, 0 );
        //         break;
        //     case 1:
        //         pogobot_led_setColor( 255, 255, 0 );
        //         break;
        //     case 2:
        //         pogobot_led_setColor( 255, 0, 0 );
        //         break;
        //     case 3:
        //         pogobot_led_setColor( 0, 255, 0 );
        //         break;
        //     case 4:
        //         pogobot_led_setColor( 0, 0, 255 );
        //         break;
        // }
        
        if (nb_voisins==4){
            pogobot_led_setColor( 0, 0, 255 );
        }
        else{
            pogobot_led_setColor( 0, 0, 0 );
        }
        if (rand()%100<50){  
            pogobot_infrared_sendMessageAllDirection( 0x1234, message,message_length_bytes);
        }
        //printf( "front : %d \nright : %d \nback : %d \nleft : %d \n",
                //neighbour_ir_front, neighbour_ir_right, neighbour_ir_back, neighbour_ir_left);

        pogobot_infrared_clear_message_queue();
        pogobot_infrared_update();
    
        t1=pogobot_stopwatch_get_elapsed_microseconds(&t0);
        msleep( (1000000/F - t1)/1000 );
    }

}
