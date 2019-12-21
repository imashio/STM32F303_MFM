// Defi Link Unit II
// seiral daisy chain decoder

#include "usart.h"
#include "defi_decoder.h"

volatile unsigned char DEFI_ID[] = {
  0x01,	// Turbo
  0x02,	// Tacho
  0x03,	// Oil pres.
  0x04,	// Fuel pres.
  0x05,	// Ext. Temp.
  0x07,	// Oil Temp.
  0x0f	// Water Temp.
};

// value = eq_grad * dec_nrm + eq_intercept
// Gradient-term of decoding equation
volatile unsigned int DEFI_eq_grad[] = {
  300,	// Turbo
  9000,	// Tacho
  1000,	// Oil pres.
  600,	// Fuel pres.
  900,	// Ext. Temp.
  100,	// Oil Temp.
  100	// Water Temp.
};

// Intercept-term of decoding equation
volatile int DEFI_eq_intercept[] = {
  -100,	// Turbo
  0,	// Tacho
  0,	// Oil pres.
  0,	// Fuel pres.
  200,	// Ext. Temp.
  50,	// Oil Temp.
  20	// Water Temp.
};

volatile int DEFI_digit[] = {
  3,	// Turbo
  4,	// Tacho
  3,	// Oil pres.
  3,	// Fuel pres.
  4,	// Ext. Temp.
  3,	// Oil Temp.
  3	// Water Temp.
};

volatile int DEFI_frac[] = {
  2,	// Turbo
  0,	// Tacho
  2,	// Oil pres.
  2,	// Fuel pres.
  0,	// Ext. Temp.
  0,	// Oil Temp.
  0	// Water Temp.
};

/*
volatile int16_t DEFI_value[] = {
  1,	// Turbo
  0,	// Tacho
  0,	// Oil pres.
  0,	// Fuel pres.
  0,	// Ext. Temp.
  0,	// Oil Temp.
  0	// Water Temp.
};
*/

//unsigned char   UART1_RxData[N_DEFI_FRAME*2];
volatile unsigned char    UART1_Data;
volatile unsigned char    UART1_data_index;
volatile unsigned char    UART1_RxData[N_DEFI_FRAME*2];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
    
    HAL_UART_Receive_IT(&huart1, &UART1_Data, 1);

    UART1_RxData[UART1_data_index] = UART1_Data;
    if( UART1_data_index < N_DEFI_FRAME*2-1 ){
       UART1_data_index++;
    }else{
        UART1_data_index = 0;
    }

}

//// int defi_decode_value(unsigned char *UART1_RxData, unsigned char UART1_data_index){
int defi_decode_value(unsigned char *UART1_RxData){
    unsigned int    m,n;
    unsigned char   DEFI_valid_frame = 0;               // valid frame indicator

    float		    DEFI_value;					            // Decoded value
    unsigned char   DEFI_FRAME[N_DEFI_FRAME];
    unsigned char   DEFI_id_index;
    unsigned char	DEFI_low4bits[N_DEFI_FRAME];		// Extracted lower 4 bits from byte data
    unsigned int    DEFI_maxv = 2352;		            // maximum decimal angle data value from 'Defi Link Unit II'
    unsigned int    DEFI_dec_ang;                   // Angle data (decimal)
    float           DEFI_dec_nrm;				            // Normalized Angle data (decimal)


    // Defi data from UART1 data recognition
    for(n=0;n<N_DEFI_FRAME*2-1;n++){
        // find Receiver ID
        if( ( UART1_RxData[n] & 0xF0 ) == 0x00 ){

            DEFI_FRAME[0] =  UART1_RxData[n];    // Reciver ID
            DEFI_FRAME[1] =  UART1_RxData[n+1];  // Control
            DEFI_FRAME[2] =  UART1_RxData[n+2];  // Angle data (100)
            DEFI_FRAME[3] =  UART1_RxData[n+3];  // Angle data ( 10)
            DEFI_FRAME[4] =  UART1_RxData[n+4];  // Angle data (  1)

            // check Defi data ID
            for(DEFI_id_index=0;DEFI_id_index<N_DEFI_MEAS_TYPE;DEFI_id_index++){
                if( DEFI_FRAME[0] == DEFI_ID[DEFI_id_index] ){
                    break;
                }
            }
            if( DEFI_id_index >= N_DEFI_MEAS_TYPE ){
                DEFI_valid_frame = 0;
            }

            // Judge data validity
            for( n = 2; n < N_DEFI_FRAME; n++ ){
                if( (   ( (DEFI_FRAME[n] >= '0') & (DEFI_FRAME[n] <= '9') )
                      | ( (DEFI_FRAME[n] >= 'A') & (DEFI_FRAME[n] <= 'F') ) ) ){
                    DEFI_valid_frame = 1;
                }else{
                    DEFI_valid_frame = 0;
                    break;
                }
            }
            break;
        }
    }

    // decode ASCII data to ISO value
    if ( DEFI_valid_frame == 1 ) {
        // Convert char to angle-dec
        DEFI_dec_ang = 0;
        m = 2; // bit shift number (4*m-bit left shift)
        for( n = 2; n < N_DEFI_FRAME; n++){ // data[0-1] is control data
            if  ( (DEFI_FRAME[n] & 0xf0) == 0x30 ){ // char is between '0' to '9'
                DEFI_low4bits[n] = (unsigned int)(DEFI_FRAME[n] & 0x0f);
            }else if ( (DEFI_FRAME[n] & 0xf0) == 0x40 ){ // char is between 'A' to 'F'
                DEFI_low4bits[n] = (unsigned int)(DEFI_FRAME[n] & 0x0f) + 9;
            }else{
                break;
            }
            DEFI_dec_ang = DEFI_dec_ang + (DEFI_low4bits[n]<<(4*m));
            m--;
        }
        // end of Convert char to angle-dec
    
        // Change angle-dec to normlized-dec
        DEFI_dec_nrm = (float)DEFI_dec_ang / DEFI_maxv;
        // end of Change angle-dec to normlized-dec
        
        // Change dec to ISO
        DEFI_value = DEFI_dec_nrm * DEFI_eq_grad[DEFI_id_index] + DEFI_eq_intercept[DEFI_id_index];
        // end of change

        return DEFI_value;

    }else{
        // Error
        return ~0;
    }


}