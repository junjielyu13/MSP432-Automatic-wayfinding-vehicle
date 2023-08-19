#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include "lib_PAE.h"

#include <rxReturn.h>
#include <uart.h>

#define LED_V_BIT BIT0
#define LED_RGB_R BIT0
#define LED_RGB_G BIT1
#define LED_RGB_B BIT2
#define BOOSTERPACK_LED_RGB_R BIT6
#define BOOSTERPACK_LED_RGB_G BIT4
#define BOOSTERPACK_LED_RGB_B BIT6

#define SW1_POS 1
#define SW2_POS 4
#define SW1_INT 0x04
#define SW2_INT 0x0A
#define JUP_INT 0x0C
#define JDOWN_INT 0x0A
#define JLEFT_INT 0x0C
#define JRIGHT_INT 0x10
#define JCENTER_INT 0x04
#define SW1_BIT BIT(SW1_POS)
#define SW2_BIT BIT(SW2_POS)

typedef struct
{
    bool r, g, b;
    uint8_t time;
} color_t;

color_t color_sequence[] = { { .r = true, .g = false, .b = false, .time = 2 }, // Vermell 2 segons
                             { .r = true, .g = true, .b = false, .time = 1 },  // Groc 1 segons
                             { .r = false, .g = true, .b = false, .time = 3 },  // Verd 3 segons
                             { .r = false, .g = false, .b = true, .time = 2 },  // Blau 2 segons
                             { .r = true, .g = true, .b = true, .time = 1 },  // Blanc 1 segons
        };

int8_t color_sequence_Size = sizeof(color_sequence) / sizeof(color_sequence[0]);


void config_RGB_LEDS(void)
{
    //port.pin dels LEDs RGB als recursos del boosterpack:
    //LEDs RGB = P2.6, P2.4, P5.6
    P2SEL0 &= ~(BIT4 + BIT6 );    //P2.4, P2.6, son GPIOs
    P2SEL1 &= ~(BIT4 + BIT6 );    //P2.4, P2.6, son GPIOs
    P2DIR |= (BIT4 + BIT6 );      //Els LEDs son sortides
    P2OUT &= ~(BIT4 + BIT6 );     //El seu estat inicial sera apagat

    P5SEL0 &= ~(BIT6 );    //P5.6 son GPIOs
    P5SEL1 &= ~(BIT6 );    //P5.6 son GPIOs
    P5DIR |= (BIT6 );      //Els LEDs son sortides
    P5OUT &= ~(BIT6 );     //El seu estat inicial sera apagat

    //borrar tot a baix
    //**********************************************************************************************************
    //port.pin dels LEDs RGB als recursos de la placa msp432
    //
    //Si no se puede usar els RGB del boosterpack, use els RGB de la placa!!!!
    //
    //LED Rojo de placa sota
    /*
    P1SEL0 &= ~(BIT0 );    //P2.0, P2.1, P2.2 son GPIOs
    P1SEL1 &= ~(BIT0 );    //P2.0, P2.1, P2.2 son GPIOs
    P1DIR |= (BIT0 );      //Els LEDs son sortides
    P1OUT &= ~(BIT0 );     //El seu estat inicial sera apagat
    // /LED RGB de placa sota
    P2SEL0 &= ~(BIT0 + BIT1 + BIT2 );   // Los LEDs RGB son GPIOs
    P2SEL1 &= ~(BIT0 + BIT1 + BIT2 );   // Los LEDs RGB son GPIOs
    P2DIR |= (BIT0 + BIT1 + BIT2 );      // Un LED es una salida
    P2OUT &= ~(BIT0 + BIT1 + BIT2 ); // El estado inicial de los LEDs RGB es apagado
    */
    //***********************************************************************************************************

}

#define BUTTON_S1       1
#define BUTTON_S2       2
#define JOYSTICK_LEFT   3
#define JOYSTICK_RIGHT  4
#define JOYSTICK_UP     5
#define JOYSTICK_DOWN   6
#define JOYSTICK_CENTER 7
#define speed           500

uint8_t estado = 7;

void init_timers(void)
{

    //Divider = 1; CLK source is ACLK; clear the counter; MODE is up
    TA0CTL |= TASSEL_1 + TACLR;
    TA0CCTL0 &= ~(CAP + CCIFG);
    TA0CCTL0 |= CCIE;           //Interrupciones activadas en CCR0
    TA0CCR0 = 33;               //1ms

    //Divider = 1; CLK source is ACLK; clear the counter; MODE is up
    TA1CTL |= TASSEL_1 + MC_1 + TACLR;
    TA1CCTL0 &= ~(CAP + CCIFG);
    TA1CCTL0 |= CCIE;                   //Interrupciones activadas en CCR0
    TA1CCR0 = 33;                       //1000ms = 1sec

}


void init_interrupciones(void)
{
    // Configuracion al estilo MSP430 "clasico":
    // --> Enable Port 1 interrupt on the NVIC.
    // Segun el Datasheet (Tabla "6-39. NVIC Interrupts", apartado "6.7.2 Device-Level User Interrupts"),
    // la interrupcion del puerto 1 es la User ISR numero 35.
    // Segun el Technical Reference Manual, apartado "2.4.3 NVIC Registers",
    // hay 2 registros de habilitacion ISER0 y ISER1, cada uno para 32 interrupciones (0..31, y 32..63, resp.),
    // accesibles mediante la estructura NVIC->ISER[x], con x = 0 o x = 1.
    // Asimismo, hay 2 registros para deshabilitarlas: ICERx, y dos registros para limpiarlas: ICPRx.

    //Int. port 3 = 37 corresponde al bit 5 del segundo registro ISER1:
    NVIC->ICPR[1] |= BIT5; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= BIT5; //y habilito las interrupciones del puerto
    //Int. port 4 = 38 corresponde al bit 6 del segundo registro ISERx:
    NVIC->ICPR[1] |= BIT6; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= BIT6; //y habilito las interrupciones del puerto
    //Int. port 5 = 39 corresponde al bit 7 del segundo registro ISERx:
    NVIC->ICPR[1] |= BIT7; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= BIT7; //y habilito las interrupciones del puerto

    // Timer A0
    NVIC->ICPR[0] |= BIT8;  //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[0] |= BIT8;  //y habilito las interrupciones del puerto
    NVIC->ICPR[0] |= BIT9;  //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[0] |= BIT9;  //y habilito las interrupciones del puerto

    // Timer A1
    NVIC->ICPR[0] |= BITA;  //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[0] |= BITA;  //y habilito las interrupciones del puerto
    NVIC->ICPR[0] |= BITB; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[0] |= BITB; //y habilito las interrupciones del puerto

    __enable_interrupt(); //Habilitamos las interrupciones a nivel global del micro.
}




//Configuramos botones
void init_botons(void)
{
    //P3.5 Polsador S2
    P3SEL0 &= ~BIT5;                    //Els polsadors son GPIOs
    P3SEL1 &= ~BIT5;                    //Els polsadors son GPIOs
    P3DIR &= ~(BIT5 );                  //Un polsador es una entrada
    P3OUT |= (BIT5 );                   //Donat que l'altra costat es GND, volem una pull-up
    P3IE |= (BIT5 );                    //Interrupcions activades
    P3IES &= ~(BIT5 );                  //amb transicio L->H
    P3IFG = 0;                          //Netegem les interrupcions anteriors

    //P4.1 Joystick Centre
    //P4.5 Joystick Esquerra
    //P4.7 Joystick Dreta
    P4SEL0 &= ~(BIT1 + BIT5 + BIT7 );   //Els polsadors son GPIOs
    P4SEL1 &= ~(BIT1 + BIT5 + BIT7 );   //Els polsadors son GPIOs
    P4DIR &= ~(BIT1 + BIT5 + BIT7 );    //Un polsador es una entrada
    P4REN |= (BIT1 + BIT5 + BIT7 );     //Pull-up/pull-down pel pulsador
    P4OUT |= (BIT1 + BIT5 + BIT7 );     //Donat que l'altra costat es GND, volem una pull-up
    P4IE |= (BIT1 + BIT5 + BIT7 );      //Interrupcions activades
    P4IES &= ~(BIT1 + BIT5 + BIT7 );    //amb transicio L->H
    P4IFG = 0;                          //Netegem les interrupcions anteriors

    //P5.1 Polsador S1
    //P5.4 Joystick Amunt
    //P5.5 Joystick Aval
    P5SEL0 &= ~(BIT1 + BIT4 + BIT5 );   //Els polsadors son GPIOs
    P5SEL1 &= ~(BIT1 +BIT4 + BIT5 );    //Els polsadors son GPIOs
    P5DIR &= ~(BIT1 + BIT4 + BIT5 );    //Un polsador es una entrada
    P5OUT |= (BIT1 + BIT4 + BIT5 );     //Donat que l'altra costat es GND, volem una pull-up
    P5IE |= (BIT1 + BIT4 + BIT5 );      //Interrupcions activades
    P5IES &= ~(BIT1 + BIT4 + BIT5 );    //amb transicio L->H
    P5IFG = 0;                          //Netegem les interrupcions anteriors

}


void main(void)
{

    WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer

    //Inicializaciones:
    init_ucs_24MHz();
    init_timers();              //Configuramos los timers
    init_UART();                //Configuramos la UART
    init_botons();              //Configuramos los botons
    init_interrupciones();      //Configuramos las interrupciones.
    config_RGB_LEDS();

    stop();

    do
    {
        switch (estado)
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


volatile uint8_t limitador = 50;
volatile uint8_t step = 10;
#define CNT_MAX 100
volatile int8_t pwm_duty = 50;

void TA0_0_IRQHandler(void)
{
    static uint8_t cnt = 0;

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

int8_t INDEX = -1;  //estat inicial
uint8_t TIMEPS = 0; //temps inicial

void TA1_0_IRQHandler(void){

    TA1CCTL0 &= ~TIMER_A_CCTLN_CCIE;  //Conviene inhabilitar la interrupción al principio
    TA1CCTL0 &= ~TIMER_A_CCTLN_CCIFG; //Clear interrupt flag

    TIMEPS++;

    if (INDEX == -1 || TIMEPS == color_sequence[INDEX].time){ //Estado inicial o después de completar el último tiempo de luz
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
        estado = BUTTON_S2;
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
    case 0x0C:
        estado = JOYSTICK_RIGHT;
        step += 5;      //Joystic dreta: la quantitat step és veu incrementada en 5.
        break;
    case 0x10:
        estado = JOYSTICK_LEFT;
        step -= 5;      //Joystick esquerra: la quantitat step és veu decrementada en 5.
        break;
    case 0x04:
        estado = JOYSTICK_CENTER;
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
    case 0x04:
        estado = BUTTON_S1;
        break;
    case 0x0A:
        estado = JOYSTICK_UP;
        limitador += step;      //Joystick amunt: increment d’aquest valor en una quantitat step
        break;
    case 0x0C:
        estado = JOYSTICK_DOWN;
        limitador -= step;      //Joystick avall: decrement d’aquest valor en una quantitat step
        break;
    default:
        break;
    }

    P5IE |= (BIT4 | BIT5 | BIT1 ); //interrupciones en port 5 reactivadas
}

