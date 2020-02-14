
extern volatile int16_t        speed;

extern volatile unsigned char  speed_pulse_ovfl;

extern volatile int16_t        rpm;

extern volatile unsigned char  tacho_pulse_ovfl;

unsigned int det_speed_pulse();

void speed_meter();

void tacho_meter();

