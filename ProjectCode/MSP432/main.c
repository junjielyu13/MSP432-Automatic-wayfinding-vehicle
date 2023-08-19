#include "lib_PAE.h"
#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include <config.h>
#include <init.h>
#include <handler.h>
#include <rxReturn.h>
#include <uart.h>


void main(void)
{

    WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer

    //Inicializaciones:
    init_ucs_24MHz();
    init_timers();              //Configuramos los timers
    init_UART();                //Configuramos la UART
    init_botons();              //Configuramos los botons
    init_interrupciones();      //Configuramos las interrupciones.
    init_RGB_LEDS();

    stop();

    do
    {
        switch (statu)
        {
        case BUTTON_S1:
            pushLeft(speed);
            break;
        case BUTTON_S2:
            pushRight(speed);
            break;
        case JOYSTICK_UP:
            push(speed);
            break;
        case JOYSTICK_DOWN:
            back(speed);
            break;
        case JOYSTICK_RIGHT:
            right(speed);
            break;
        case JOYSTICK_LEFT:
            left(speed);
            break;
        case JOYSTICK_CENTER:
            stop();
            break;
        default:
            break;
        }
    }
    while (1);
}
