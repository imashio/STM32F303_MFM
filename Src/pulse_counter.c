#include "tim.h"

// USART_TypeDef* huart2; // debug

// COMMON setting ----------------------------------------------------------------

// core frequency [kHz]
#define F_CPU           72000000

// Timer prescaler
#define Prescaler       360         // Timer prescaler must be set '360-1'

// Number of timer counter (Tacho, Speed)
#define NCNT            65536       // Timer counter must be set '65536-1'

// Speed meter setting ----------------------------------------------------------------

// Number of pulese pulse/rpm (Speed)
#define Npulse_speed    4

// Number of pulese rpm (Speed)
#define Fbase_speed    637


// Tacho meter setting ----------------------------------------------------------------

// Number of pulese pulse/rpm (Tacho)
#define Npulse_tacho    2


// SPEED meter ----------------------------------------------------------------
// Pin : PA4 (GPIO_IN)
//       There is No Interrupt for PA4

volatile uint16_t        speed;

volatile unsigned char  speed_pulse_ovfl = 0;

unsigned int det_speed_pulse(){
    // detect speed pulse negative edge
    static unsigned int  STATE0;
    static unsigned int  STATE1;

    STATE1 = STATE0;
    STATE0 = HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_4 );

    return ~STATE0 & STATE1;

}

void speed_overflow(){
    if( speed_pulse_ovfl > 1 ){
        speed = 0;
    }else{
        speed_pulse_ovfl++;
    }
}

void speed_meter(){
    static unsigned int  CNT0;
    static unsigned int  CNT1;
    
    float  freq;

    if( speed_pulse_ovfl > 1 ){
        speed = 0;
        speed_pulse_ovfl = 0;
    }else{

        CNT0 = CNT1;
        CNT1 = TIM6->CNT;

        freq = (float)F_CPU / (float)Prescaler / (float)(CNT1-CNT0 + NCNT*speed_pulse_ovfl);

        speed = freq / (float)(Npulse_speed * Fbase_speed) * 3600.0;

        speed_pulse_ovfl = 0;
    }

}


// TACHO meter ----------------------------------------------------------------
// Pin : PB4 (GPIO_EXTI)

volatile uint16_t        rpm;

volatile unsigned char  tacho_pulse_ovfl = 0;

void tacho_overflow(){
    if( tacho_pulse_ovfl > 1 ){
        rpm = 0;
    }else{
        tacho_pulse_ovfl++;
    }
}

void tacho_meter(){
    static unsigned int  CNT0;
    static unsigned int  CNT1;
    
    float  freq;

    if( tacho_pulse_ovfl > 1 ){
        rpm = 0;
        tacho_pulse_ovfl = 0;
    }else{

        CNT0 = CNT1;
        CNT1 = TIM3->CNT;

        freq = (float)F_CPU / (float)Prescaler / (float)(CNT1-CNT0 + NCNT*tacho_pulse_ovfl);

        rpm = freq / (float)Npulse_tacho * 60.0;

        if( rpm < 7000 ){
            tacho_pulse_ovfl++;
        }

        tacho_pulse_ovfl = 0;
    }

}

