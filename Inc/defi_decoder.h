#ifndef DEFI_H

#define DEFI_H

#define N_DEFI_MEAS_TYPE  7  // number of Defi measurement type 
#define N_DEFI_BYTE       5  // number of Defi data per 1 packet
#define N_DEFI_PACKET     8 // number of Defi data packet

  // 0 : Receiver ID
  // 1 : Control
  // 2 : Angle data (100 digit)
  // 3 : Angle data ( 10 digit)
  // 4 : Angle data (  1 digit)

extern volatile unsigned char DEFI_ID[];

// value = eq_grad * dec_nrm + eq_intercept
// Gradient-term of decoding equation
extern volatile unsigned int DEFI_eq_grad[];

// Intercept-term of decoding equation
extern volatile int     DEFI_eq_intercept[];

extern volatile int     DEFI_digit[];

extern volatile int     DEFI_frac[];

extern volatile int     DEFI_sign[];

extern unsigned char    DEFI_FRAME[N_DEFI_BYTE];

extern volatile unsigned char   UART_data_update;
extern volatile unsigned char   UART_data_index;
extern volatile unsigned char   UART_RxData[N_DEFI_BYTE*N_DEFI_PACKET];

extern volatile unsigned char    DEFI_proc_data_index;

extern volatile int16_t DEFI_value[];

void defi_init();

void defi_decoder();

#endif
