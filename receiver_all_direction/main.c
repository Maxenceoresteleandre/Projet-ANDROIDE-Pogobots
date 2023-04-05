/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
**/

/* clang-format-ok */

#include <stdio.h>
#include <string.h>
#include "pogobot.h"


int main(void) {

    pogobot_init();

    printf("init ok\n");

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
                printf("len : %d,%s\n",strlen(mr.payload),mr.payload);

                
                pogobot_led_setColor( 0, 255, 0 );
                msleep( 1000 );
                pogobot_led_setColor( 0, 0, 0 );
                
            }
        }
        else
        {
            pogobot_led_setColor( 255, 255, 255 );
        }
    }

}
