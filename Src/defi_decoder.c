#include "main.h"

// Defi Link Unit II
// seiral daisy chain decoder

#include "usart.h"
#include "defi_decoder.h"

volatile unsigned char    UART_data_index;
volatile unsigned char    UART_RxData[N_DEFI_BYTE*N_DEFI_PACKET];

volatile int DEFI_debug;
volatile int DEFI_debug2;

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
  11000,	// Tacho
  100,	// Oil pres.
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
  1,	// Oil pres.
  2,	// Fuel pres.
  0,	// Ext. Temp.
  0,	// Oil Temp.
  0	    // Water Temp.
};


volatile int16_t DEFI_value[] = {
  0,	// Turbo
  0,	// Tacho
  0,	// Oil pres.
  0,	// Fuel pres.
  0,	// Ext. Temp.
  0,	// Oil Temp.
  0	    // Water Temp.
};

#define Tacho_NLUT 16
#define Tacho_StepLUT 50 // Angle step
volatile unsigned int Ang_LUT[] = {
    25,     // 1
    75,     // 2
    125,    // 3
    175,	// 4
    225,	// 5
    275,	// 6
    325,    // 7
    375,	// 8
    425,	// 9
    475,	// 10
    525,	// 11
    575,	// 12
    625,    // 13
    675,    // 14
    725,    // 15
    775,    // 16
    2719    // 17
};
volatile unsigned int Tacho_LUT[] = {
    0,      // 1
    0,      // 2
    0,      // 3
    700,	// 4
    735,	// 5
    770,	// 6
    805,    // 7
    835,	// 8
    971,	// 9
    1449,	// 10
    1947,	// 11
    2358,	// 12
    2749,   // 13
    3023,   // 14
    3244,   // 15
    3502,   // 16
    13000   // 17
};

volatile unsigned char    DEFI_proc_data_index;

static UART_HandleTypeDef* pHuart;

void defi_init(){
    memset(UART_RxData, 0, sizeof(UART_RxData));
}

void defi_decoder(){
    unsigned int    ite;
    unsigned int    m, n;
    unsigned int    UART_write_index;
    unsigned int    UART_proc_end_index;

    unsigned char   DEFI_valid_frame = 0;               // valid frame indicator
    
    unsigned char   DEFI_FRAME[N_DEFI_BYTE];
    unsigned char   DEFI_id_index;
    unsigned char	DEFI_low4bits[N_DEFI_BYTE];		// Extracted lower 4 bits from byte data
    float           DEFI_maxv = 2352.0;		        // maximum decimal angle data value from 'Defi Link Unit II'
    unsigned int    DEFI_dec_ang;                   // Angle data (decimal)
    float           DEFI_dec_nrm;		            // Normalized Angle data (decimal)

    ite = 0;

    pHuart = &huart1;
    UART_write_index = (sizeof(UART_RxData) - pHuart->hdmarx->Instance->CNDTR) % sizeof(UART_RxData);

    if( (UART_write_index - N_DEFI_BYTE) < 0 ){
        UART_proc_end_index = (UART_write_index - N_DEFI_BYTE) + sizeof(UART_RxData);
    }else{
        UART_proc_end_index = (UART_write_index - N_DEFI_BYTE);
    }
/*
    // DEBUG
    HAL_UART_Transmit_printf(&huart2, "DEFI_proc:%d, UART:%d\n", DEFI_proc_data_index,UART_write_index); // DEBUG
    // DEBUG
*/
    // Defi data from UART data recognition
    while( (DEFI_proc_data_index <= (UART_write_index - N_DEFI_BYTE)) | (DEFI_proc_data_index > UART_write_index) ){

        // find Receiver ID
        if( ( UART_RxData[DEFI_proc_data_index] & 0xF0 ) == 0x00 ){

            DEFI_FRAME[0] =  UART_RxData[DEFI_proc_data_index+0];    // Reciver ID
            DEFI_FRAME[1] =  UART_RxData[DEFI_proc_data_index+1];  // Control
            DEFI_FRAME[2] =  UART_RxData[DEFI_proc_data_index+2];  // Angle data (100)
            DEFI_FRAME[3] =  UART_RxData[DEFI_proc_data_index+3];  // Angle data ( 10)
            DEFI_FRAME[4] =  UART_RxData[DEFI_proc_data_index+4];  // Angle data (  1)

            // check Defi data ID
            for(DEFI_id_index=0;DEFI_id_index<N_DEFI_MEAS_TYPE;DEFI_id_index++){
                if( DEFI_FRAME[0] == DEFI_ID[DEFI_id_index] ){
                    break;
                }
            }

            if( DEFI_id_index < N_DEFI_MEAS_TYPE ){
                // Judge data validity
                for( n = 2; n < N_DEFI_BYTE; n++ ){
                    if( (   ( (DEFI_FRAME[n] >= '0') & (DEFI_FRAME[n] <= '9') )
                        | ( (DEFI_FRAME[n] >= 'A') & (DEFI_FRAME[n] <= 'F') ) ) ){
                        DEFI_valid_frame = 1;
                    }else{
                        DEFI_valid_frame = 0;
                        break;
                    }
                }
            }

            // decode ASCII data to ISO value
            if ( DEFI_valid_frame ) {
                // Convert char to angle-dec
                DEFI_dec_ang = 0;
                m = 2; // bit shift number (4*m-bit left shift)
                for( n = 2; n < N_DEFI_BYTE; n++){ // data[0-1] is control data
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
            
                // Change dec to ISO
                if( DEFI_ID[DEFI_id_index] == 0x02 ){ 

                    /*
                    // Look up table
                    float DIV;
                    float FRAC;
                    unsigned int LUT_index;

                    DIV = ((float)DEFI_dec_ang+(float)Tacho_StepLUT/2) / (float)Tacho_StepLUT;
                    LUT_index = (unsigned int)DIV;
                    if( LUT_index > Tacho_NLUT ) LUT_index = Tacho_NLUT;
                    FRAC = DEFI_dec_ang - Ang_LUT[LUT_index];

                    DEFI_value[DEFI_id_index] = (Tacho_LUT[LUT_index+1]-Tacho_LUT[LUT_index])/(Ang_LUT[LUT_index+1]-Ang_LUT[LUT_index])*FRAC + Tacho_LUT[LUT_index];
                    */

                    if( DEFI_dec_ang <= (int)(119.3549/4.7708) ){
                        DEFI_value[DEFI_id_index] = 0;
                    }else{
                        DEFI_value[DEFI_id_index] = (int)( DEFI_dec_ang * 4.7708 - 119.3549 );
                    }

                    /*
                    if( DEFI_dec_nrm <= 50 ){
                        DEFI_value[DEFI_id_index] = 0;
                    }else if( (DEFI_dec_nrm >  50)&&(DEFI_dec_nrm <= 420) ){
                        DEFI_value[DEFI_id_index] =  * 0.4360 + 634.6;
                    }else if( (DEFI_dec_nrm > 420)&&(DEFI_dec_nrm <= 700) ){
                        DEFI_value[DEFI_id_index] = DEFI_dec_nrm*DEFI_dec_nrm * (-0.0113) + DEFI_dec_nrm * 21.0445 - 6027.0;
                    }else{
                        DEFI_value[DEFI_id_index] = DEFI_dec_nrm*DEFI_dec_nrm * (-0.0003) + DEFI_dec_nrm * 5.5913 - 605.48;
                    }
                    */
                    // debug 
                    DEFI_debug = DEFI_dec_ang;
                    DEFI_debug2 = DEFI_value[DEFI_id_index];
                    // debug
                    
                }else{
                    // Change angle-dec to normlized-dec
                    DEFI_dec_nrm = (float)DEFI_dec_ang / DEFI_maxv;
                    // end of Change angle-dec to normlized-dec
                    
                    DEFI_value[DEFI_id_index] = DEFI_dec_nrm * (float)DEFI_eq_grad[DEFI_id_index] + DEFI_eq_intercept[DEFI_id_index];
                }
                // end of change

            }

        }
        
 /*       
        // DEBUG
        //if( (DEFI_id_index == 0) && (( DEFI_value[DEFI_id_index] > 400 )|( DEFI_value[DEFI_id_index] < -400 ) ) ){ // DEBUG
        if( (DEFI_id_index == 1)  ){ // DEBUG
            HAL_UART_Transmit_printf(&huart2, "UART:%d   id:%d %x %x %x %x %x %d\n", UART_data_index, DEFI_id_index, DEFI_FRAME[0],DEFI_FRAME[1],DEFI_FRAME[2],DEFI_FRAME[3],DEFI_FRAME[4],DEFI_value[DEFI_id_index]); // DEBUG
        }
        // DEBUG
*/
/*
        if( DEFI_id_index == 0x02 ){ // DEBUG
            for( n = 0; n < N_DEFI_BYTE*N_DEFI_PACKET; n++){
                HAL_UART_Transmit_printf(&huart2, "[%d], %x\n", n, UART_RxData[n]); // DEBUG
            }
        }
*/

        if( DEFI_proc_data_index < N_DEFI_BYTE*N_DEFI_PACKET-1 ){
            DEFI_proc_data_index++;
        }else{
            DEFI_proc_data_index = 0;
        }

        if( ite > 100 ){ // Time-out
            break;
        }else{
            ite++;
        }
        
    }

}
