#ifndef RXRETURN_H_
#define RXRETURN_H_

typedef uint8_t byte;
typedef int bool;

typedef struct RxReturn{
    byte StatusPacket[16];
    byte time_out;
    bool error;
} RxReturn;


#endif /* RXRETURN_H_ */
