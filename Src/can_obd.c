#include "can.h"
#include "can_obd.h"

CAN_TxHeaderTypeDef     CAN_TxHeader;
CAN_RxHeaderTypeDef     CAN_RxHeader;
volatile uint8_t        CAN_TxData[8];
volatile uint8_t        CAN_RxData[8];
volatile uint32_t       CAN_TxMailbox;

volatile uint8_t        CAN_Received; // CAN data received flag
////  Received data and set 'CAN_Received' to 1 in 'stm32**xx_it.c'
// 
//        void CAN_RX0_IRQHandler(void)
//        {
//            /* USER CODE BEGIN CAN_RX0_IRQn 0 */
//            
//            /* USER CODE END CAN_RX0_IRQn 0 */
//            HAL_CAN_IRQHandler(&hcan);
//            /* USER CODE BEGIN CAN_RX0_IRQn 1 */
//            HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN_RxData);
//            CAN_Received = 1;
//            /* USER CODE END CAN_RX0_IRQn 1 */
//        }
//


void CAN_OBD_Init(){

    // CAN filter configuration
    CAN_FilterTypeDef CAN_FilterConfig;
    CAN_FilterConfig.FilterBank = 0;
    CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
    //  CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfig.FilterIdHigh = 0x7DF<<5;
    CAN_FilterConfig.FilterIdLow = 0x7DF<<5;
    CAN_FilterConfig.FilterMaskIdHigh = 0x000<<5 | 0x08;
    CAN_FilterConfig.FilterMaskIdLow = 0x000<<5 | 0x08;
//    CAN_FilterConfig.FilterMaskIdHigh = 0x7FF<<5 | 0x08;
//    CAN_FilterConfig.FilterMaskIdLow = 0x7FF<<5 | 0x08;
    CAN_FilterConfig.FilterFIFOAssignment = 0;
    CAN_FilterConfig.FilterActivation = ENABLE;
    CAN_FilterConfig.SlaveStartFilterBank = 0;

    // CAN filter configuration
    if (HAL_CAN_ConfigFilter(&hcan, &CAN_FilterConfig) != HAL_OK){
        Error_Handler();
    }

    // CAN Tx Header setup
    CAN_TxHeader.StdId        = 0x7E8; // ECU ID
    CAN_TxHeader.RTR          = CAN_RTR_DATA;
    CAN_TxHeader.IDE          = CAN_ID_STD;
    CAN_TxHeader.DLC          = 8;
    CAN_TxHeader.TransmitGlobalTime = DISABLE;

    // CAN Rx Header setup
    CAN_RxHeader.StdId        = 0x7DF; // Any ID
    CAN_RxHeader.ExtId        = 0;
    CAN_RxHeader.IDE          = CAN_ID_STD;
    CAN_RxHeader.RTR          = 0;
    CAN_RxHeader.DLC          = 8;
    CAN_RxHeader.Timestamp    = 0;
    CAN_RxHeader.FilterMatchIndex = 0;

    // CAN start
    HAL_CAN_Start(&hcan);
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){ // Activate Interrupt Callback
        Error_Handler();
    }

}

void CAN_OBD_Response(uint8_t MAP, uint16_t rpm, uint8_t Speed, uint8_t Throttle, uint8_t CoolantTemp, uint8_t OilTemp, uint16_t FuelPress, uint8_t IntakeAirTemp){
//void CAN_OBD_Response(uint8_t MAP, uint16_t rpm, uint8_t SPEED, uint8_t THROTTLE, uint8_t COOLANT_TEMP, uint8_t OIL_TEMP, uint16_t FUELPRESS){

    uint8_t	RPM_A;
    uint8_t RPM_B;

    uint8_t	MAP_ABS;

    uint8_t FUEL_RAIL_PRESS_A;
    uint8_t FUEL_RAIL_PRESS_B;

    if( CAN_Received ){

        // Data detection process
        // Mode 01h PID 00h / PIDs supported (01h-1Fh)
        if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x00 ){
            CAN_TxData[0] = 0x06; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x00; // PID

            CAN_TxData[3] = 0x08; // Engine Coolant Temperature 05h
//            CAN_TxData[4] = 0x78; // Fuel Pressure 0Ah, MAP 0Bh, Engine RPM 0Ch, Speed 0Dh
            CAN_TxData[4] = 0x7A; // Fuel Pressure 0Ah, MAP 0Bh, Engine RPM 0Ch, Speed 0Dh, IntakeAirTemp 0Fh
            CAN_TxData[5] = 0x80; // Throttle position 11h
            CAN_TxData[6] = 0x01; // PID 20h
            CAN_TxData[7] = 0x00;
        
        // Mode 01h PID 20h / PIDs supported (21h-3Fh)
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x20 ){
            CAN_TxData[0] = 0x06; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x20; // PID

//            CAN_TxData[3] = 0x00;
            CAN_TxData[3] = 0x40; // Fuel Rail Pressure (relative to manifold vacuum) 22h
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x01; // PID 40h
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 40h / PIDs supported (41h-5Fh)
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x40 ){
            CAN_TxData[0] = 0x06; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x40; // PID

            CAN_TxData[3] = 0x00;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x11; // Engine Oil temprature 59h, PID 60h
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 60h / PIDs supported (61h-7Fh)
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x60 ){
            CAN_TxData[0] = 0x06; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x60; // PID

            CAN_TxData[3] = 0x00;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x01; // PID 80h
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 80h / PIDs supported (81h-9Fh)
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x80 ){
            CAN_TxData[0] = 0x06; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x80; // PID

            CAN_TxData[3] = 0x00;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        // Mode 09h PID 00h / Service 9 supported PIDs 
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x09 && CAN_RxData[2] == 0x00 ){
/*
            CAN_TxData[0] = 0x06; // Data Length [byte]
            CAN_TxData[1] = 0x49; // Mode
            CAN_TxData[2] = 0x00; // PID
*/
            CAN_TxData[0] = 0x04; // Data Length [byte]
            CAN_TxData[1] = 0x00;
            CAN_TxData[2] = 0x00;

            CAN_TxData[3] = 0x00;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 05h / Engine coolant temperature
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x05 ){

            CAN_TxData[0] = 0x03; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x05; // PID

            CAN_TxData[3] = CoolantTemp + 40;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 0Ah / Fuel pressure (gauge pressure)
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x0A ){

            CAN_TxData[0] = 0x03; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x0A; // PID

            CAN_TxData[3] = FuelPress/3;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 0Bh / MAP Intake manifold absolute pressure
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x0B ){

            if( MAP <= -101 ){
                MAP_ABS = 0;
            }else{
                MAP_ABS = MAP + 101; // Atmospheric pressure 101.325kPa
            }
            
            CAN_TxData[0] = 0x03; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x0B; // PID

            CAN_TxData[3] = MAP_ABS;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 0Ch / Engine RPM
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x0C ){

            RPM_A = ((unsigned long int)rpm * 4) >> 8;
            RPM_B = ((unsigned long int)rpm * 4) - 256*(unsigned long int)RPM_A;

            CAN_TxData[0] = 0x04; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x0C; // PID

            CAN_TxData[3] = RPM_A;
            CAN_TxData[4] = RPM_B;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 0Dh / Speed
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x0D ){
            CAN_TxData[0] = 0x03; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x0D; // PID

            CAN_TxData[3] = Speed;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 0Fh / Intake air temperature
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x0F ){
            CAN_TxData[0] = 0x03; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x0F; // PID

            CAN_TxData[3] = IntakeAirTemp + 40;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 11h / Throttle Position
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x11 ){
            CAN_TxData[0] = 0x03; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x11; // PID

            CAN_TxData[3] = Throttle;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 22h / Fuel Rail Pressure 0.079(256A+B)
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x22 ){

            FUEL_RAIL_PRESS_A = ((uint16_t)( (1.0/0.079)*FuelPress ) >> 8 ) & 0xff;
            FUEL_RAIL_PRESS_B = ((uint16_t)( (1.0/0.079)*FuelPress )      ) & 0xff;

            CAN_TxData[0] = 0x04; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x22; // PID

            CAN_TxData[3] = FUEL_RAIL_PRESS_A;
            CAN_TxData[4] = FUEL_RAIL_PRESS_B;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 42h / Control module voltage
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x42 ){
            CAN_TxData[0] = 0x04; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x42; // PID

            CAN_TxData[3] = 29;
            CAN_TxData[4] = 246;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        // Mode 01h PID 5Ch / Engine oil temperature
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x5C ){
            CAN_TxData[0] = 0x03; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x5C; // PID

            CAN_TxData[3] = OilTemp+40;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        // Mode 01h not supported PID
        }else if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 ){
            CAN_TxData[0] = 0x04; // Data Length [byte]
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = CAN_RxData[2]; // PID
            CAN_TxData[3] = 0xAA;
            CAN_TxData[4] = 0xAA;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

		// Mode 22h PID 1410h / Injector Fuel Pulse Width
		}else if( CAN_RxData[0] == 0x03 && CAN_RxData[1] == 0x22 && CAN_RxData[2] == 0x14 && CAN_RxData[3] == 0x10 ){
			CAN_TxData[0] = 0x05; // Data Length [byte]
			CAN_TxData[1] = 0x62; // Mode
			CAN_TxData[2] = 0x14; // PID upper 8-bit
			CAN_TxData[3] = 0x10; // PID lower 8-bit
			CAN_TxData[4] = 0x00; // DATA A
			CAN_TxData[5] = 0x00; // DATA B
			CAN_TxData[6] = 0x00;
			CAN_TxData[7] = 0x00;

		// Mode 22h PID 402Bh / Unknown
		}else if( CAN_RxData[0] == 0x03 && CAN_RxData[1] == 0x22 && CAN_RxData[2] == 0x40 && CAN_RxData[3] == 0x2B ){
			CAN_TxData[0] = 0x04; // Data Length [byte]
			CAN_TxData[1] = 0x62; // Mode
			CAN_TxData[2] = 0x40; // PID upper 8-bit
			CAN_TxData[3] = 0x2B; // PID lower 8-bit
			CAN_TxData[4] = 0x7F; // DATA A
			CAN_TxData[5] = 0x00;
			CAN_TxData[6] = 0x00;
			CAN_TxData[7] = 0x00;

		// Mode 22h PID 4030h / Unknown
		}else if( CAN_RxData[0] == 0x03 && CAN_RxData[1] == 0x22 && CAN_RxData[2] == 0x40 && CAN_RxData[3] == 0x30 ){
			CAN_TxData[0] = 0x04; // Data Length [byte]
			CAN_TxData[1] = 0x62; // Mode
			CAN_TxData[2] = 0x40; // PID upper 8-bit
			CAN_TxData[3] = 0x30; // PID lower 8-bit
			CAN_TxData[4] = 0x7F; // DATA A
			CAN_TxData[5] = 0x00;
			CAN_TxData[6] = 0x00;
			CAN_TxData[7] = 0x00;

		// Mode 22h not supported PID
		}else if( CAN_RxData[0] == 0x03 && CAN_RxData[1] == 0x22 ){
			CAN_TxData[0] = 0x04; // Data Length [byte]
			CAN_TxData[1] = 0x62; // Mode
			CAN_TxData[2] = CAN_RxData[2]; // PID upper 8-bit
			CAN_TxData[3] = CAN_RxData[3]; // PID lower 8-bit
			CAN_TxData[4] = 0x00;
			CAN_TxData[5] = 0x00;
			CAN_TxData[6] = 0x00;
			CAN_TxData[7] = 0x00;

		// Mode 22h not supported PID & invalid data
		}else if( CAN_RxData[1] == 0x22 ){
			CAN_TxData[0] = 0x04; // Data Length [byte]
			CAN_TxData[1] = 0x62; // Mode
			CAN_TxData[2] = CAN_RxData[2]; // PID upper 8-bit
			CAN_TxData[3] = CAN_RxData[3]; // PID lower 8-bit
			CAN_TxData[4] = 0x00;
			CAN_TxData[5] = 0x00;
			CAN_TxData[6] = 0x00;
			CAN_TxData[7] = 0x00;


        // error
        }else{
			CAN_TxData[0] = 0x04; // Data Length [byte]
			CAN_TxData[1] = CAN_RxData[1]; // Mode
			CAN_TxData[2] = CAN_RxData[2]; // PID upper 8-bit
			CAN_TxData[3] = CAN_RxData[3]; // PID lower 8-bit
			CAN_TxData[4] = 0x00;
			CAN_TxData[5] = 0x00;
			CAN_TxData[6] = 0x00;
			CAN_TxData[7] = 0x00;

        }

        HAL_CAN_AddTxMessage(&hcan, &CAN_TxHeader, CAN_TxData, &CAN_TxMailbox);
        CAN_Received = 0;
    }

}
