#ifndef UART_H_
#define UART_H_


extern uint32_t timer_ms_count;

void delay_t(uint32_t temps_ms);
void init_UART(void);
void wheelMode(void);
struct RxReturn RxPacket(void);
byte TxPacket(byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16]);
void sentit_dades_Rx(void);
void sentit_dades_Tx(void);
bool time_out(int time);
void TxUAC2(byte bTxdData);

void EUSCIA2_IRQHandler(void);
void TA1_0_IRQHandler(void);

void actionMotor(byte ID, bool rotation, unsigned int speed);

void left(unsigned int speed);
void right(unsigned int speed);

void stop(void);
void push(unsigned int speed);
void back(unsigned int speed);

int readSensor(byte ID, byte sensor);
void init_sensor(byte ID,unsigned int dist);

#endif /* UART_H_ */
