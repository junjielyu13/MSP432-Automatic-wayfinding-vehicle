#ifndef RXRETURN_H_
#define RXRETURN_H_

typedef uint8_t byte;
typedef int BOOL;

typedef struct RxReturn{
    byte StatusPacket[16];
    byte time_out;
    BOOL error;
} RxReturn;

#endif /* RXRETURN_H_ */
