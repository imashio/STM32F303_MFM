#ifndef DEFI_H

#define DEFI_H

#define N_DEFI_MEAS_TYPE 7 // number of Defi measurement type 
#define N_DEFI_FRAME     5 // number of Defi data per 1 frame

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

extern unsigned char    DEFI_FRAME[N_DEFI_FRAME];

/*
extern unsigned char    DEFI_id_index;
extern unsigned char	  DEFI_low4bits[N_DEFI_FRAME];		// Extracted lower 4 bits from byte data
extern unsigned int     DEFI_maxv;		                  // maximum decimal angle data value from 'Defi Link Unit II'
extern unsigned int     DEFI_dec_ang;                   // Angle data (decimal)
extern float            DEFI_dec_nrm;				            // Normalized Angle data (decimal)
extern float		        DEFI_value;					            // Decoded value
*/

extern volatile unsigned char   UART1_Data;
extern volatile unsigned char   UART1_data_index;
extern volatile unsigned char   UART1_RxData[N_DEFI_FRAME*2];


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);

int defi_decode_value(unsigned char *);

#endif
