#ifndef UART_H_
#define UART_H_

void init_UART(void);
struct RxReturn RxPacket(void);
byte TxPacket(byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16]);
void sentit_dades_Rx(void);
void sentit_dades_Tx(void);
BOOL time_out(int time);
void TxUAC2(byte bTxdData);
void EUSCIA2_IRQHandler(void);
void TA1_0_IRQHandler(void);
void actionMotor(byte ID, bool rotation, unsigned int speed);
void left(unsigned int speed);
void pushLeft(unsigned int speed);
void right(unsigned int speed);
void pushRight(unsigned int speed);
void stop(void);
void push(unsigned int speed);
void back(unsigned int speed);

#endif /* UART_H_ */
