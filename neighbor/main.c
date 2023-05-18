/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
**/

/* 
Border detection (Pogobots change color (Blue) if they have neighbors on all sides). 
*/

#include <stdio.h>
#include <string.h>
#include "pogobot.h"

#define message_length_bytes 100

int main(void) {

    pogobot_init();

    printf("init ok\n");

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
    
    time_reference_t timer_front;
    time_reference_t timer_right;
    time_reference_t timer_back;
    time_reference_t timer_left;

    while (1)
    {
        pogobot_infrared_update();
        /* read reception fifo buffer */
        if ( pogobot_infrared_message_available() )
        {
            while ( pogobot_infrared_message_available() )
            {
                printf( "You've got a new message ! \n" );
                message_t mr;
                pogobot_infrared_recover_next_message( &mr );

                //printf("len : %d,%s\n", strlen(mr.payload), mr.payload);
                int ir_id = mr.header._receiver_ir_index;
                printf( "ir_id:%d\n",ir_id );

                if ( ir_id == 0 ){
                    pogobot_stopwatch_reset(&timer_front);
                }
                else if ( ir_id == 1 ){
                    pogobot_stopwatch_reset(&timer_right);
                }
                else if ( ir_id == 2 ){
                    pogobot_stopwatch_reset(&timer_back);
                }
                else if ( ir_id == 3 ){
                    pogobot_stopwatch_reset(&timer_left);
                }
                
                pogobot_led_setColor( 0, 255, 0 );
                
            }
        }

        if ((pogobot_stopwatch_get_elapsed_microseconds(&timer_front) / 1000 < 500) &&
            (pogobot_stopwatch_get_elapsed_microseconds(&timer_right) / 1000 < 500) &&
            (pogobot_stopwatch_get_elapsed_microseconds(&timer_back) / 1000 < 500) &&
            (pogobot_stopwatch_get_elapsed_microseconds(&timer_left) / 1000 < 500)){
            
            pogobot_led_setColor( 0, 0, 255 );

        }

        //printf( "front : %d \nright : %d \nback : %d \nleft : %d \n",
                //neighbour_ir_front, neighbour_ir_right, neighbour_ir_back, neighbour_ir_left);

        pogobot_infrared_clear_message_queue();
        pogobot_infrared_update();
    
        msleep( 50 );

        // 0 success
        bool b = pogobot_infrared_sendMessageAllDirection( 0x1234, message,message_length_bytes);
        if ( b == 0 ){
            printf( "SENDING %d...\n",message_length_bytes );
            // pogobot_infrared_update();   
            pogobot_led_setColor( 255, 0, 0 );
        }
        msleep( 50 );
    }

}
