#include <msp432p401r.h>
#include <rxReturn.h>
#include <stdint.h>
#include <stdio.h>

#define TXD2_READY (UCA2IFG & UCTXIFG)
#define LEFT 0
#define RIGHT 1
#define LEFT_MOTOR  3
#define RIGHT_MOTOR 2

#define WRITE_DATA 0x03
#define READ_DATA 0x02

#define SPEED_LOW   0x20
#define SPEED_HIGH  0x21

#define false 0
#define true 1

uint32_t timer_ms_count;
uint32_t current_time = 0;
byte received_data;
byte read_data_UART;

void delay_t(uint32_t temps_ms)
{
    timer_ms_count = 0;
    TA0CTL |= MC_1;
    do {

    } while (timer_ms_count <= temps_ms);
    TA0CTL &= ~MC_1;
}

bool time_out(int time){
    return (current_time>=time);
}

/* funcions per canviar el sentit de les comunicacions */
void sentit_dades_Rx(void)
{ //Configuració del Half Duplex dels motors: Recepció
    P3OUT &= ~BIT0; //El pin P3.0 (DIRECTION_PORT) el posem a 0 (Rx)
}

void sentit_dades_Tx(void)
{ //Configuració del Half Duplex dels motors: Transmissió
    P3OUT |= BIT0; //El pin P3.0 (DIRECTION_PORT) el posem a 1 (Tx)
}

/* funció TxUACx(byte): envia un byte de dades per la UART 0 */
void TxUAC2(byte bTxdData)
{
    while(!TXD2_READY);  // Espera a que estigui preparat el buffer de transmissió
    UCA2TXBUF = bTxdData;
}

void init_UART(void)
{
    UCA2CTLW0 |= UCSWRST;           //Fem un reset de la USCI, desactiva la USCI
    UCA2CTLW0 |= UCSSEL__SMCLK;     //UCSYNC=0 mode asíncron
                                    //UCMODEx=0 seleccionem mode UART
                                    //UCSPB=0 nomes 1 stop bit
                                    //UC7BIT=0 8 bits de dades
                                    //UCMSB=0 bit de menys pes primer
                                    //UCPAR=x ja que no es fa servir bit de paritat
                                    //UCPEN=0 sense bit de paritat
                                    //Triem SMCLK (16MHz) com a font del clock BRCLK
    UCA2MCTLW = UCOS16;             // Necessitem sobre-mostreig => bit 0 = UCOS16 = 1
    UCA2BRW = 3;                    //Prescaler de BRCLK fixat a 3. Com SMCLK va a24MHz,
                                    //volem un baud rate de 500kb/s i fem sobre-mostreig de 16
                                    //el rellotge de la UART ha de ser de 8MHz (24MHz/3).

    //Configurem els pins de la UART
    P3SEL0 |= BIT2 | BIT3;          //I/O funció: P1.3 = UART0TX, P1.2 = UART0RX
    P3SEL1 &= ~ (BIT2 | BIT3);
    P3SEL0 &= ~BIT0;
    P3SEL1 &= ~BIT0;
    P3DIR |= BIT0;
    P3OUT &= ~BIT0;
    UCA2CTLW0 &= ~UCSWRST; //Reactivem la línia de comunicacions sèrie
    UCA2IE |= UCRXIE;
}


//TxPacket() 3 paràmetres: ID del Dynamixel, Mida dels paràmetres, Instruction byte. torna la mida del "Return packet"
byte TxPacket(byte bID, byte bParameterLength, byte bInstruction, byte bParameters[16])
{
    byte bCount,bCheckSum,bPacketLength;
    byte TxBuffer[32];
    if(bParameters[0] <= 5){ // Comprovació per seguretat
        return 0;
    }
    sentit_dades_Tx();  //El pin P3.0 (DIRECTION_PORT) el posem a 1 (Transmetre)
    TxBuffer[0] = 0xff; //Primers 2 bytes que indiquen inici de trama FF, FF.
    TxBuffer[1] = 0xff;
    TxBuffer[2] = bID; //ID del mòdul al que volem enviar el missatge
    TxBuffer[3] = bParameterLength+2; //Length(Parameter,Instruction,Checksum)
    TxBuffer[4] = bInstruction; //Instrucció que enviem al Mòdul
    for(bCount = 0; bCount < bParameterLength; bCount++) //Comencem a generar la trama que hem d’enviar
    {
        TxBuffer[bCount+5] = bParameters[bCount];
    }
    bCheckSum = 0;
    bPacketLength = bParameterLength+4+2;
    for(bCount = 2; bCount < bPacketLength-1; bCount++) //Càlcul del checksum
    {
        bCheckSum += TxBuffer[bCount];
    }
    TxBuffer[bCount] = ~bCheckSum; //Escriu el Checksum (complement a 1)
    for(bCount = 0; bCount < bPacketLength; bCount++) //Aquest bucle és el que envia la trama al Mòdul Robot
    {
        TxUAC2(TxBuffer[bCount]);
    }
    while( (UCA2STATW&UCBUSY)); //Espera fins que s’ha transmès el últim byte
    sentit_dades_Rx(); //Posem la línia de dades en Rx perquè el mòdul Dynamixel envia resposta
    return bPacketLength;
}


struct RxReturn RxPacket(void)
{
    struct RxReturn respuesta;
    byte bCount, bLenght, bCheckSum = 0;
    bool Rx_time_out = false;
    sentit_dades_Rx(); //Ponemos la linea half duplex en Rx
    for(bCount = 0; bCount < 4; bCount++)
    {
        current_time=0;
        received_data=0; //No_se_ha_recibido_Byte();
        while (!received_data)
        {
            Rx_time_out = time_out(20);  // tiempo en decenas de microsegundos
            if (Rx_time_out)break;
        }
        if (Rx_time_out)break; //sale del for si ha habido Timeout
        respuesta.StatusPacket[bCount] = read_data_UART; //Get_Byte_Leido_UART();
    }
    if (!Rx_time_out){
        bLenght = respuesta.StatusPacket[3]+4;
        for(bCount = 4; bCount < bLenght; bCount++)
        {
            current_time=0;
            received_data=0; //No_se_ha_recibido_Byte();
            while (!received_data)
            {
                Rx_time_out = time_out(20);   // tiempo en decenas de microsegundos
                if (Rx_time_out)break;
            }
            if (Rx_time_out)break; //sale del for si ha habido Timeout
            respuesta.StatusPacket[bCount] = read_data_UART;
        }
    }

    if(!Rx_time_out){
        for(bCount = 2; bCount < bLenght-1; bCount++) //checksum
        {
            bCheckSum += respuesta.StatusPacket[bCount];
        }
        bCheckSum = ~bCheckSum; //Escriu el Checksum (complement a 1)
        if(respuesta.StatusPacket[bLenght-1] != bCheckSum){
            respuesta.error = true;
        }
    }
    return respuesta;
}

//interrupcion de recepcion en la UART A0
void EUSCIA2_IRQHandler(void){

    UCA2IE &= ~UCRXIE; //Interrupciones desactivadas en RX
    read_data_UART = UCA2RXBUF;
    received_data = 1;
    UCA2IE |= UCRXIE; //Interrupciones reactivadas en RX
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


void left(unsigned int speed)
{
    actionMotor(RIGHT_MOTOR, RIGHT, speed);
    actionMotor(LEFT_MOTOR, LEFT, speed/2);
}


void right(unsigned int speed)
{
    actionMotor(RIGHT_MOTOR, LEFT, 0);
    actionMotor(LEFT_MOTOR, LEFT, speed);
}

void push(unsigned int speed)
{

    actionMotor(RIGHT_MOTOR, RIGHT, speed);
    actionMotor(LEFT_MOTOR, LEFT, speed);

}

void back(unsigned int speed)
{
    actionMotor(RIGHT_MOTOR, LEFT, speed);
    actionMotor(LEFT_MOTOR, RIGHT, speed);

}

void init_sensor(byte ID,unsigned int dist)
{

    struct RxReturn returnPacket;
    byte bParameters[16];
    bParameters[0] = 0x14;
    bParameters[1] = dist;

    TxPacket(ID, 2, WRITE_DATA, bParameters);
    returnPacket = RxPacket();

}

int readSensor(byte ID, byte sensor)
{

    struct RxReturn returnPacket;
    byte bParameters[16];
    bParameters[0] = sensor;
    bParameters[1] = 1;

    TxPacket(ID, 2, READ_DATA, bParameters);
    returnPacket = RxPacket();

    return returnPacket.StatusPacket[5];
}
