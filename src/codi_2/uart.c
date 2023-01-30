#include <msp432p401r.h>
#include <lib_PAE.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <rxReturn.h>


#define false       0
#define true        1
#define TXD2_READY (UCA2IFG & UCTXIFG)
#define WRITE_DATA  0x03
#define READ_DATA   0x02
#define LEFT        0
#define RIGHT       1
#define LEFT_MOTOR  3
#define RIGHT_MOTOR 2
#define SPEED_LOW   0x20
#define SPEED_HIGH  0x21

uint32_t current_time = 0;
byte Byte_Recibido;
byte DatoLeido_UART;

// Función time_out es True si el tiempo del contador (current_time) supera el tiempo que hemos pasado como parámetro (time)
bool time_out(int time){
    return (current_time>=time);
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

/* funcions per canviar el sentit de les comunicacions */
void sentit_dades_Rx(void)
{ //Configuraci¨® del Half Duplex dels motors: Recepci¨®
    P3OUT &= ~BIT0; //El pin P3.0 (DIRECTION_PORT) el posem a 0 (Rx)
}

void sentit_dades_Tx(void)
{ //Configuraci¨® del Half Duplex dels motors: Transmissi¨®
    P3OUT |= BIT0; //El pin P3.0 (DIRECTION_PORT) el posem a 1 (Tx)
}

/* funci¨® TxUACx(byte): envia un byte de dades per la UART 0 */
void TxUAC2(byte bTxdData)
{
    while(!TXD2_READY); // Espera a que estigui preparat el buffer de transmissi¨®
    UCA2TXBUF = bTxdData;
}

//TxPacket() 3 par¨¤metres: ID del Dynamixel, Mida dels par¨¤metres, Instruction byte. torna la mida del "Return packet"
byte TxPacket(byte bID, byte bParameterLength, byte bInstruction, byte bParameters[16])
{
    byte bCount,bCheckSum,bPacketLength;
    byte TxBuffer[32];

    if(bParameters[0] <= 5){ // Comprovacié”Ÿï¿½ per seguretat
        return 0;
    }
    sentit_dades_Tx();              //El pin P3.0 (DIRECTION_PORT) el posem a 1 (Transmetre)
    TxBuffer[0] = 0xff;             //Primers 2 bytes que indiquen inici de trama FF, FF.
    TxBuffer[1] = 0xff;
    TxBuffer[2] = bID;                                  //ID del mé”Ÿçµ›ul al que volem enviar el missatge
    TxBuffer[3] = bParameterLength+2;                   //Length(Parameter,Instruction,Checksum)
    TxBuffer[4] = bInstruction;                         //Instruccié”Ÿï¿½ que enviem al Mé”Ÿçµ›ul
    for(bCount = 0; bCount < bParameterLength; bCount++) //Comencem a generar la trama que hem dé”Ÿçµœnviar
    {
        TxBuffer[bCount+5] = bParameters[bCount];
    }
    bCheckSum = 0;
    bPacketLength = bParameterLength+4+2;
    for(bCount = 2; bCount < bPacketLength-1; bCount++) //Cé”Ÿçµ£cul del checksum
    {
        bCheckSum += TxBuffer[bCount];
    }
    TxBuffer[bCount] = ~bCheckSum;                    //Escriu el Checksum (complement a 1)
    for(bCount = 0; bCount < bPacketLength; bCount++) //Aquest bucle é”Ÿçµª el que envia la trama al Mé”Ÿçµ›ul Robot
    {
        TxUAC2(TxBuffer[bCount]);
    }
    while( (UCA2STATW&UCBUSY));                         //Espera fins que sé”ŸçµŸa transmé”Ÿçµª el é”Ÿçµ£tim byte
    sentit_dades_Rx();                                  //Posem la lé”Ÿçµ¥ia de dades en Rx perqué”Ÿï¿½ el mé”Ÿçµ›ul Dynamixel envia resposta
    return bPacketLength;
}


struct RxReturn RxPacket(void)
{
    struct RxReturn respuesta;
    byte bCount, bLenght, bCheckSum = 0;
    BOOL Rx_time_out = 0;

    sentit_dades_Rx(); //Ponemos la linea half duplex en Rx

    for(bCount = 0; bCount < 4; bCount++)
    {
        current_time=0;

        Byte_Recibido=0; //No_se_ha_recibido_Byte();

        while (!Byte_Recibido) // No se ha recibido el dato?
        {
            Rx_time_out=TimeOut(20);  // tiempo en decenas de microsegundos
            if (Rx_time_out)break;
        }

        if (Rx_time_out)break;  //sale del for si ha habido Timeout

        //Si no, es que todo ha ido bien, y leemos un dato:
        respuesta.StatusPacket[bCount] = DatoLeido_UART; //Get_Byte_Leido_UART();
    }

    // Continua llegint la resta de bytes del Status Packet
    if (!Rx_time_out){

        bLenght = respuesta.StatusPacket[3]+4;

        for(bCount = 4; bCount < bLenght; bCount++)
        {
            current_time=0;

            Byte_Recibido=0; //No_se_ha_recibido_Byte();

            while (!Byte_Recibido) // No se ha recibido el dato?
            {
                Rx_time_out = TimeOut(20);  // Tiempo de espera en milisegundos
                if (Rx_time_out)break;
            }

            if (Rx_time_out)break; //sale del for si ha habido Timeout

            //Si no, es que todo ha ido bien, y leemos un dato:
            respuesta.StatusPacket[bCount] = DatoLeido_UART;
        }
    }

    return respuesta;
}

//interrupcion de recepcion en la UART A0
void EUSCIA2_IRQHandler(void){

    UCA2IE &= ~UCRXIE; //Interrupciones desactivadas en RX
    DatoLeido_UART = UCA2RXBUF;
    Byte_Recibido = 1;
    UCA2IE |= UCRXIE;  //Interrupciones reactivadas en RX
}

void TA1_0_IRQHandler(void) {

    TA1CCTL0 &= ~CCIE;
    current_time++;
    TA1CCTL0 &= ~CCIFG;
    TA1CCTL0 |= CCIE;
}



void actionMotor(byte ID, bool rotation, unsigned int speed)
{
    struct RxReturn returnPacket;
    byte bParameters[16], speed_rotation, speed_origin;
    speed_origin = speed;

    if(speed<1024){
        if(rotation){
            speed_rotation = (speed >> 8)+4;
        }else{
            speed_rotation = speed >> 8;
        }

        bParameters[0] = SPEED_LOW;
        bParameters[1] = speed_origin;
        bParameters[2] = speed_rotation;

        TxPacket(ID, 3, WRITE_DATA, bParameters);
        returnPacket = RxPacket();
    }
}

void stop(void)
{
    actionMotor(RIGHT_MOTOR, 0, 0);
    actionMotor(LEFT_MOTOR, 0, 0);
}

void left(unsigned int speed){
    if(speed < 1024){
        actionMotor(RIGHT_MOTOR, RIGHT, speed);
        actionMotor(LEFT_MOTOR, RIGHT, 0);
    }
}


void pushLeft(unsigned int speed)
{
    if(speed < 1024){
        actionMotor(RIGHT_MOTOR, RIGHT, speed*2);
        actionMotor(LEFT_MOTOR, LEFT, speed);
    }
}

void right(unsigned int speed)
{
    if(speed < 1024){
        actionMotor(RIGHT_MOTOR, LEFT, 0);
        actionMotor(LEFT_MOTOR, LEFT, speed);
    }
}


void pushRight(unsigned int speed)
{
    if(speed < 1024){
        actionMotor(RIGHT_MOTOR, RIGHT, speed);
        actionMotor(LEFT_MOTOR, LEFT, speed*2);
    }
}

void push(unsigned int speed)
{
    if(speed < 1024){
        actionMotor(RIGHT_MOTOR, RIGHT, speed);
        actionMotor(LEFT_MOTOR, LEFT, speed);
    }
}

void back(unsigned int speed)
{
    if(speed < 1024){
        actionMotor(RIGHT_MOTOR, LEFT, speed);
        actionMotor(LEFT_MOTOR, RIGHT, speed);
    }
}
