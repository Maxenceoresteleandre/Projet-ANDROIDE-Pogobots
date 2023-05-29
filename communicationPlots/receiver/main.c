/**
 * POGOBOT
 *
 * Copyright © 2022 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
**/

/**
 * Ce code correspond à l'algo suivant:
 * Tant que le nombre d'expériences n'est pas atteint:
 * - Réinitialiser le tick
 * - Dépiler ses messages
 * - Regarder l'id de l'expéditeur:
 *      - si il est nouveau, on incrémente le nombre de voisins identifiés autour de nous
 * - Envoyer un message avec FREQEMISSION % de chance
 * - Dormir jusqu'à la fin du tick
 * - Afficher sur la console le nombre de voisins perçus et le nombre de messages reçus
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pogobot.h"


#define message_length_bytes 100
// faire varier les paramètres suivants, par pas de 10:
#define FTICK 30               
#define FREQEMISSION 50     // en fonction de la fréquence d'émission -> de 0 à 100%
#define TEMPSSONDE 1e6      // en fonction du temps de sonde -> de 0 à 1 seconde

int main(void) {

    pogobot_init();

    printf("init ok\n");
    srand(pogobot_helper_getRandSeed());
    // pogobot_infrared_emitter_power_max - 3
    // pogobot_infrared_emitter_power_twoThird - 2
    // pogobot_infrared_emitter_power_oneThird - 1
    // pogobot_infrared_emitter_power_null - 0
    pogobot_infrared_set_power( pogobot_infrared_emitter_power_twoThird );

    unsigned char message[message_length_bytes];
    int i;
    for (i=0;i<message_length_bytes;i++){
        message[i] = 'a';
    }
    message[i-1] = '\0';

    time_reference_t t0;    // tick
    uint32_t t1;

    int iter = 0;   // répéter l'expérience
    while (iter < 10){
        
        int nbMsgRecus = 0;
        int neighborsID[4] = {-1, -1, -1, -1}; // pour déterminer le nombre de voisins perçus
        time_reference_t tStartExp; // temps d'expérience (plutôt que de fonctionner en itérations)
        uint32_t tEndExp;
        pogobot_stopwatch_reset(&tStartExp);
        tEndExp = pogobot_stopwatch_get_elapsed_microseconds(&tStartExp);
        while (tEndExp < TEMPSSONDE) { 
            pogobot_led_setColor( 0, 0, 0 );
            pogobot_stopwatch_reset(&t0);
            pogobot_infrared_update();

            /* read reception fifo buffer */
            if ( pogobot_infrared_message_available() ) {
                printf("msg!!");
                while ( pogobot_infrared_message_available() ) {
                    message_t mr;
                    pogobot_infrared_recover_next_message( &mr );

                    int s_id = mr.header._sender_id;
                    int i=0;
                    for (i=0; i<4; i++){
                        if (neighborsID[i]==-1){
                            neighborsID[i]=s_id;
                            break;
                        }
                        if (neighborsID[i] == s_id){
                            break;
                        }
                    }  

                    nbMsgRecus++;
                }
            }
            
            if (rand()%100<FREQEMISSION){  
                pogobot_infrared_sendMessageAllDirection( 0x1234, message, message_length_bytes);
            }

            pogobot_infrared_clear_message_queue(); // clean pour le temps de sonde
            pogobot_infrared_update();
        
            t1=pogobot_stopwatch_get_elapsed_microseconds(&t0); // si on a dépassé le tick, on ignore la phase sleep
            if ((FTICK != 0) && (t1 < 1000000/FTICK)) {
                msleep( (1000000/FTICK - t1)/1000 );
            }

            tEndExp = pogobot_stopwatch_get_elapsed_microseconds(&tStartExp);
        }

        int nbVoisins = 0;
        for (int i=0;i<4;i++){
            if (neighborsID[i] != -1){
                nbVoisins++;
            }
        }
        printf("Nombre de messages reçus: %d, nombre de voisins perçus: %d\n", nbMsgRecus, nbVoisins);

        iter++;

    }

}
