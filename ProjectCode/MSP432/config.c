#include <msp432p401r.h>
#include <lib_PAE.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>



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


#define LED_V_BIT BIT0
#define LED_RGB_R BIT0
#define LED_RGB_G BIT1
#define LED_RGB_B BIT2
#define BOOSTERPACK_LED_RGB_R BIT6
#define BOOSTERPACK_LED_RGB_G BIT4
#define BOOSTERPACK_LED_RGB_B BIT6


#define CNT_MAX 100



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



#define BUTTON_S1       1
#define BUTTON_S2       2
#define JOYSTICK_LEFT   3
#define JOYSTICK_RIGHT  4
#define JOYSTICK_UP     5
#define JOYSTICK_DOWN   6
#define JOYSTICK_CENTER 7
#define speed           500



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
uint8_t statu = 7;
int8_t INDEX = -1;  //estat inicial
uint8_t TIMEPS = 0; //temps inicial
static uint8_t cnt = 0;
volatile uint8_t limitador = 50;
volatile uint8_t step = 10;
volatile uint8_t change = 5;


