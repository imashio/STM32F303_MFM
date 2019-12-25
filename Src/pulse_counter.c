#include "tim.h"


USART_TypeDef* huart2; // debug

// core frequency [kHz]
#define F_CPU           72000000

// TIM prescaler
#define Prescaler       360

// Number of pulese pulse/rpm (Tacho)
#define NCNT            65536

// Number of pulese pulse/rpm (Tacho)
#define Npulse_tacho    2

volatile int16_t        rpm;

volatile unsigned char  pulse_ovfl = 0;

void tacho_overflow(){
    if( pulse_ovfl > 1 ){
        rpm = 0;
    }else{
        pulse_ovfl++;
    }
}


//unsigned int tacho(){
void tacho(){
    static unsigned int  CNT0;
    static unsigned int  CNT1;
    
    float  freq;

    if( pulse_ovfl > 1 ){
        rpm = 0;
        pulse_ovfl = 0;
    }else{

        CNT0 = CNT1;
        CNT1 = TIM3->CNT;

//        HAL_UART_Transmit_printf(&huart2, "CNT1=%d, CNT0=%d, pulse_ovfl=%d - ", CNT1, CNT0, pulse_ovfl); // debug

        freq = (float)F_CPU / (float)Prescaler / (float)(CNT1-CNT0 + NCNT*pulse_ovfl);

        rpm = freq * (float)Npulse_tacho * 60.0;

//        HAL_UART_Transmit_printf(&huart2, "%d[rpm]]\n", rpm); // debug

        pulse_ovfl = 0;
    }

}