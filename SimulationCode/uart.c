#include <msp432p401r.h>
#include <rxReturn.h>
#include <stdint.h>
#include <stdio.h>

#define TXD0_READY (UCA0IFG & UCTXIFG)
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
#define Si 1

uint32_t timer_ms_count;
uint32_t current_time = 0;
byte received_data;
byte read_data_UART;
byte Byte_Recibido;
uint16_t DatoLeido_UART;

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
void TxUAC0(uint8_t bTxdData)
{
    while(!TXD0_READY);     //espera a que esté preparado el buffer de transmisión
    UCA0TXBUF = bTxdData;   //ponemos el dato en el buffer de transmisión
}

void init_UART(void)
{
    UCA0CTLW0 |= UCSWRST;                   //hacemos un reset de la USCI, desactiva la USCI
    UCA0CTLW0 |= UCSSEL__SMCLK;             //seleccionamos la fuente de reloj SMCLK
    UCA0MCTLW = UCOS16;                     //oversampling habilitado
    UCA0BRW = 3;                            //baud rate = 500000b/s
    P1SEL0 |= BIT2 | BIT3;                  //configuramos los pines de la UART
    P1SEL1 &= ~ (BIT2 | BIT3);              //P1.3 = UART0TX, P1.2 = UART0RX
    UCA0CTLW0 &= ~UCSWRST;                  //reactivamos la línea de comunicaciones serie
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    //limpiamos el flag de interrupciones de la UART
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        //habilitamos a nivel de dispositivo las interrupciones de USCI_A0 RX
                                            //cuando tengamos la recepción
    P3SEL0 &= ~BIT0;                        //DIRECTION PORT es un GPIO
    P3SEL1 &= ~BIT0;                        //DIRECTION PORT es un GPIO
    P3DIR |= BIT0;                          //DIRECTION PORT es un salida
    P3OUT &= ~BIT0;                         //DIRECTION PORT en un principio indica que se van a recibir bytes
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
        TxUAC0(TxBuffer[bCount]);                           //pone el dato en el buffer de transmisión
        //espera a que esté preparado el buffer de transmisión
    }
    while((UCA0STATW & UCBUSY));         
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
void EUSCIA0_IRQHandler(){
    UCA0IE &= ~UCRXIE;                                      //interrupciones desactivadas en Rx
    EUSCI_A0->IFG &=~ EUSCI_A_IFG_RXIFG;                    //limpiamos el flag de interrupciones
    DatoLeido_UART = UCA0RXBUF;                             //leemos el dato recibido del buffer de recepción
    Byte_Recibido=Si;                                       //indicamos que se ha recibido el byte
    UCA0IE |= UCRXIE;                                       //interrupciones reactivadas en Rx
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
