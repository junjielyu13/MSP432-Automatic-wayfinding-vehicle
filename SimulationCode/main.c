#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include <rxReturn.h>
#include <uart.h>
#include <lib_PAE.h>

#define BUTTON_S1       1
#define BUTTON_S2       2
#define JOYSTICK_LEFT   3
#define JOYSTICK_RIGHT  4
#define JOYSTICK_UP     5
#define JOYSTICK_DOWN   6
#define JOYSTICK_CENTER 7

#define LEFT_SENSOR     0x1A
#define CENTER_SENSOR   0x1B
#define RIGHT_SENSOR    0x1C
#define SENSOR_ID       100
#define DISTANCE        200

uint8_t estado = 0;
uint8_t estado_anterior = 8;
volatile unsigned int speed = 600;

int activado = 0;

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
    //interrupcuiones en el puerto 5
    NVIC->ICPR[1] |= (BIT7);
    NVIC->ISER[1] |= (BIT7);

    //interrupciones en el puerto 4
    NVIC->ICPR[1] |= (BIT6);
    NVIC->ISER[1] |= (BIT6);

    //interrupciones en el puerto 3
    //Int. puerto 3 = 37 corresponde al bit 5 del segundo registro ISER1
    NVIC->ICPR[1] |= (BIT5); //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto
    NVIC->ISER[1] |= (BIT5); //y habilito las interrupciones de los puertos

    //timer A0
    //Int. timer A0 corresponde al bit 8 del primer registro ISER0
    NVIC->ICPR[0] |= 1 << 8; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para el timer
    NVIC->ISER[0] |= 1 << 8; //y habilito las interrupciones del timer

    //timer A1
    //Int. timer A1 corresponde al bit 10 del primer registro ISER0
    NVIC->ICPR[0] |= 1 << 10; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para el timer
    NVIC->ISER[0] |= 1 << 10; //y habilito las interrupciones del timer

    //eUSCI_A0
    //Int. eUSCI_A0 corresponde al bit 16 del primer registro ISER0
    NVIC->ICPR[0] |= 1 << EUSCIA0_IRQn; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente
    NVIC->ISER[0] |= 1 << EUSCIA0_IRQn; //y habilito las interrupciones
   
    __enable_interrupt(); //Habilitamos las interrupciones a nivel global del micro.
}

//Configuramos botones
void init_botons(void)
{
    //P3.5 Polsador S2
    P3SEL0 &= ~BIT5;                    //Els polsadors son GPIOs
    P3SEL1 &= ~BIT5;                    //Els polsadors son GPIOs
    P3DIR &= ~(BIT5 );                  //Un polsador es una entrada
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
    P5IE |= (BIT1 + BIT4 + BIT5 );      //Interrupcions activades
    P5IES &= ~(BIT1 + BIT4 + BIT5 );    //amb transicio L->H
    P5IFG = 0;                          //Netegem les interrupcions anteriors

}

uint8_t parent = 0;
void main(void)
{

    WDTCTL = WDTPW + WDTHOLD; // Paramos el watchdog timer

    //Inicializaciones:
    init_ucs_24MHz();
    init_timers();              //Configuramos los timers
    init_UART();                //Configuramos la UART
    init_botons();              //Configuramos los botons
    init_interrupciones();      //Configuramos las interrupciones.


    int distR, distC, distL;
    init_sensor(SENSOR_ID, DISTANCE);


    do
    {

        distC = readSensor(SENSOR_ID, CENTER_SENSOR);
        distR = readSensor(SENSOR_ID, RIGHT_SENSOR);
        distL = readSensor(SENSOR_ID, LEFT_SENSOR);

        if (activado)
        {


            if(parent == 0){
                push(speed);
                if(distC >= 80){
                    parent = 1;
                }
            }

            if(parent && distC >= 50){
                while( distC != 0 && (distL <= 125) ){

                    right(speed);

                    distC = readSensor(SENSOR_ID, CENTER_SENSOR);
                    distR = readSensor(SENSOR_ID, RIGHT_SENSOR);
                    distL = readSensor(SENSOR_ID, LEFT_SENSOR);

                }
            }else if(parent && distL <= 25 && distC <= 25 && distR <= 25){
                while(distC <= 0 && distL <= 15){

                    left(speed);

                    distC = readSensor(SENSOR_ID, CENTER_SENSOR);
                    distR = readSensor(SENSOR_ID, RIGHT_SENSOR);
                    distL = readSensor(SENSOR_ID, LEFT_SENSOR);
                }

            }else if( parent && distL >= 125 ){
                while( distC <= 10 && distL >= 125  ){

                    right(speed);

                    distC = readSensor(SENSOR_ID, CENTER_SENSOR);
                    distR = readSensor(SENSOR_ID, RIGHT_SENSOR);
                    distL = readSensor(SENSOR_ID, LEFT_SENSOR);
                }

            }else if(parent){
                push(speed);
            }

        }

        if (estado_anterior != estado)
        {
            estado_anterior = estado;

            switch (estado)
            {
            case BUTTON_S1:
                activado = 1;
                break;
            case BUTTON_S2:
                activado = 0;
                stop();
                break;
            case JOYSTICK_LEFT:
                break;
            case JOYSTICK_RIGHT:
                break;
            case JOYSTICK_UP:
                break;
            case JOYSTICK_DOWN:
                break;
            case JOYSTICK_CENTER:
                break;
            default:
                break;
            }
        }


    }
    while (1);
}

void TA0_0_IRQHandler(void)
{
    TA0CCTL0 &= ~CCIE;
    timer_ms_count++;
    TA0CCTL0 &= ~CCIFG;
    TA0CCTL0 |= CCIE;
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
        break;
    case 0x10:
        estado = JOYSTICK_LEFT;
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
        break;
    case 0x0C:
        estado = JOYSTICK_DOWN;
        break;
    default:
        break;
    }

    P5IE |= (BIT4 | BIT5 | BIT1 ); //interrupciones en port 5 reactivadas
}
