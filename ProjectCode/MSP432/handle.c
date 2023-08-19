#include <msp432p401r.h>
#include <lib_PAE.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <init.h>
#include <config.h>


void TA0_0_IRQHandler(void)
{

    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIE;  //Conviene inhabilitar la interrupción al principio
    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIFG; //Clear interrupt flag

    //Si boosterpack no funciona, use este
    //Recuerde activar el código inferior en config_RGB_LEDS
    //este es sirve para controlar la luminosidad de la placa LED1 P1.0:
    /*
    if (cnt == CNT_MAX){
        //Encendemos el LED Rojo
        if(limitador != 0){ //Si el limitador es igual 0 es para mantener las luces apagadas
            P1OUT |= LED_V_BIT;
        }
        cnt = 0;
    }else if (cnt >= limitador){ //Mayor que limitador es significa apagar las luces
        //Apagamos el LED Rojo
        P1OUT &= ~LED_V_BIT;
    }else if (cnt < limitador){ //Menor que limitador es significa encendemos las luces
        //Encendemos el LED Rojo
        P1OUT |= LED_V_BIT;
    }
    */

    //este es sirve para controlar la luminosidad de la boosterpack RGB:
    ///*
    if (cnt == CNT_MAX){
        //Encendemos el LED rojo
        if(limitador != 0){ //Si el limitador es igual 0 es para mantener las luces apagadas
            P2OUT |= BOOSTERPACK_LED_RGB_R;
        }
        cnt = 0;
    }else if (cnt >= limitador){ //Mayor que limitador es significa apagar las luces
        //Apagamos el LED rojo
        P2OUT &= ~(BOOSTERPACK_LED_RGB_R);
    }else if (cnt < limitador){ //Menor que limitador es significa encendemos las luces
        //Encendemos el LED rojo
        P2OUT |= BOOSTERPACK_LED_RGB_R;
    }
    //*/

    cnt++;

    TA0CCTL0 |= TIMER_A_CCTLN_CCIE; //Se debe habilitar la interrupción antes de salir
}



void TA1_0_IRQHandler(void)
{

    TA1CCTL0 &= ~TIMER_A_CCTLN_CCIE;  //Conviene inhabilitar la interrupción al principio
    TA1CCTL0 &= ~TIMER_A_CCTLN_CCIFG; //Clear interrupt flag

    TIMEPS++;

    if (INDEX == -1 || TIMEPS == color_sequence[INDEX].time){ //statu inicial o después de completar el último tiempo de luz
        INDEX++;

        if (INDEX == color_sequence_Size){ //Evitar overflow del color sequence
            INDEX = 0;
        }

        TIMEPS = 0;


        //Si boosterpack no funciona, use este
        //este es sirve para controlar la luminosidad de la placa LED1 P1.0:
        //Recuerde activar el código inferior en config_RGB_LEDS
        /*
        if (color_sequence[INDEX].r == true){   //Encendemos el RGB Rojo
            P2OUT |= LED_RGB_R;
        }else{                                  //Apagamos el RGB Rojo
            P2OUT &= ~(LED_RGB_R);
        }

        if (color_sequence[INDEX].g == true){   //Encendemos el RGB amarillo
            P2OUT |= LED_RGB_G;
        }else{                                  //Apagamos el RGB amarillo
            P2OUT &= ~(LED_RGB_G);
        }

        if (color_sequence[INDEX].b == true){   //Encendemos el RGB azul
            P2OUT |= LED_RGB_B;
        }else{                                  //Apagamos el RGB azul
            P2OUT &= ~(LED_RGB_B);
        }
        */



        //este es sirve para controlar la luminosidad de la boosterpack RGB:
        ///*
        if (color_sequence[INDEX].r == true){   //Encendemos el RGB Rojo
            P2OUT |= BOOSTERPACK_LED_RGB_R;
        }else{                                  //Apagamos el RGB Rojo
            P2OUT &= ~(BOOSTERPACK_LED_RGB_R);
        }

        if (color_sequence[INDEX].g == true){   //Encendemos el RGB amarillo
            P2OUT |= BOOSTERPACK_LED_RGB_G;
        }else{                                  //Apagamos el RGB amarillo
            P2OUT &= ~(BOOSTERPACK_LED_RGB_G);
        }

        if (color_sequence[INDEX].b == true){   //Encendemos el RGB azul
            P5OUT |= BOOSTERPACK_LED_RGB_B;
        }else{                                  //Apagamos el RGB azul
            P5OUT &= ~(BOOSTERPACK_LED_RGB_B);
        }
        //*/
    }

    TA1CCTL0 |= TIMER_A_CCTLN_CCIE; //Se debe habilitar la interrupción antes de salir
}

//ISR para las interrupciones del puerto 3
void PORT3_IRQHandler(void)
{
    uint8_t flag = P3IV; //guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P3IE &= ~BIT5;       //interrupciones del boton S2 en port 3 desactivadas

    if (flag == 0x0C)
    {
        statu = BUTTON_S2;
    }

    P3IE |= BIT5;       //interrupciones en port 3 reactivadas
}

//ISR para las interrupciones del puerto 4
void PORT4_IRQHandler(void)
{
    uint8_t flag = P4IV;            //guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P4IE &= ~(BIT7 | BIT5 | BIT1 ); //interrupciones Joystick en port 4 desactivadas

    switch (flag)
    {
    case JLEFT_INT:
        statu = JOYSTICK_RIGHT;
        step += change;      //Joystic dreta: la quantitat step és veu incrementada en 5.
        break;
    case JRIGHT_INT:
        statu = JOYSTICK_LEFT;
        step -= change;      //Joystick esquerra: la quantitat step és veu decrementada en 5.
        break;
    case JCENTER_INT:
        statu = JOYSTICK_CENTER;
        break;
    default:
        break;
    }

    P4IE |= (BIT7 | BIT5 | BIT1 ); //interrupciones en port 4 reactivadas
}

//ISR para las interrupciones del puerto 5
void PORT5_IRQHandler(void)
{
    uint8_t flag = P5IV;            //guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P5IE &= ~(BIT4 | BIT5 | BIT1 ); //interrupciones Joystick y S1 en port 5 desactivadas

    switch (flag)
    {
    case SW1_INT:
        statu = BUTTON_S1;
        break;
    case JDOWN_INT:
        statu = JOYSTICK_UP;
        limitador += step;      //Joystick amunt: increment d’aquest valor en una quantitat step
        break;
    case JUP_INT:
        statu = JOYSTICK_DOWN;
        limitador -= step;      //Joystick avall: decrement d’aquest valor en una quantitat step
        break;
    default:
        break;
    }

    P5IE |= (BIT4 | BIT5 | BIT1 ); //interrupciones en port 5 reactivadas
}
