#include <msp432p401r.h>
#include <lib_PAE.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>



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


void init_RGB_LEDS(void)
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


void init_UART(void)
{
    UCA2CTLW0 |= UCSWRST;           //Fem un reset de la USCI, desactiva la USCI
    UCA2CTLW0 |= UCSSEL__SMCLK;     //UCSYNC=0 mode asé”Ÿçµ¥cron
                                    //UCMODEx=0 seleccionem mode UART
                                    //UCSPB=0 nomes 1 stop bit
                                    //UC7BIT=0 8 bits de dades
                                    //UCMSB=0 bit de menys pes primer
                                    //UCPAR=x ja que no es fa servir bit de paritat
                                    //UCPEN=0 sense bit de paritat
                                    //Triem SMCLK (24MHz) com a font del clock BRCLK

    UCA2MCTLW = UCOS16;             // Necessitem sobre-mostreig => bit 0 = UCOS16 = 1
    UCA2BRW = 3;                    //Prescaler de BRCLK fixat a 13. Com SMCLK va a24MHz,
                                    //volem un baud rate de 115200kb/s i fem sobre-mostreig de 16
                                    //el rellotge de la UART ha de ser de ~1.85MHz (24MHz/13).

    //Configurem els pins de la UART
    P3SEL0 |= BIT2 | BIT3;          //I/O funci¨®: P1.3 = UART0TX, P1.2 = UART0RX
    P3SEL1 &= ~ (BIT2 | BIT3);
    P3SEL0 &= ~BIT0;
    P3SEL1 &= ~BIT0;
    P3DIR |= BIT0;
    P3OUT &= ~BIT0;

    UCA2CTLW0 &= ~UCSWRST;          //Reactivem la l¨ªnia de comunicacions s¨¨rie
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG; // Clear eUSCI RX interrupt flag

}
