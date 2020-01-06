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
    CAN_FilterConfig.FilterMaskIdHigh = 0x7FF<<5;
    CAN_FilterConfig.FilterMaskIdLow = 0x7FF<<5;
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

void CAN_OBD_Response(uint8_t MAP, uint16_t rpm, uint8_t SPEED, uint8_t THROTTLE){

    uint8_t	RPM_A;
    uint8_t RPM_B;

    RPM_A = ((unsigned long int)rpm * 4) >> 8;
    RPM_B = ((unsigned long int)rpm * 4) - 256*(unsigned long int)RPM_A;

    if( CAN_Received ){

        // Data detection process
        // Mode 01h PID 00h / PIDs supported
        if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x00 ){
            CAN_TxData[0] = 0x06; // Data Length
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x00; // PID

            CAN_TxData[3] = 0x00;
            CAN_TxData[4] = 0x38; // MAP & Engine RPM & Speed
    //        CAN_TxData[4] = 0x18; // Engine RPM & Speed
            CAN_TxData[5] = 0x80; // Throttle position
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        }

        // Mode 09h PID 00h / PIDs supported
        if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x09 && CAN_RxData[2] == 0x00 ){
            CAN_TxData[0] = 0x06; // Data Length
            CAN_TxData[1] = 0x49; // Mode
            CAN_TxData[2] = 0x00; // PID

            CAN_TxData[3] = 0x00;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        }

        // Mode 01h PID 0Bh / Intake manifold absolute pressure
        if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x0B ){
            CAN_TxData[0] = 0x03; // Data Length
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x0B; // PID

            CAN_TxData[3] = MAP;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        }

        // Mode 01h PID 0Ch / Engine RPM
        if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x0C ){
            CAN_TxData[0] = 0x04; // Data Length
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x0C; // PID

            CAN_TxData[3] = RPM_A;
            CAN_TxData[4] = RPM_B;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        }

        // Mode 01h PID 0Dh / Speed
        if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x0D ){
            CAN_TxData[0] = 0x03; // Data Length
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x0D; // PID

            CAN_TxData[3] = SPEED;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        }

        // Mode 01h PID 11h / Throttle Position
        if( CAN_RxData[0] == 0x02 && CAN_RxData[1] == 0x01 && CAN_RxData[2] == 0x11 ){
            CAN_TxData[0] = 0x03; // Data Length
            CAN_TxData[1] = 0x41; // Mode
            CAN_TxData[2] = 0x11; // PID

            CAN_TxData[3] = THROTTLE;
            CAN_TxData[4] = 0x00;
            CAN_TxData[5] = 0x00;
            CAN_TxData[6] = 0x00;
            CAN_TxData[7] = 0x00;

        }

        HAL_CAN_AddTxMessage(&hcan, &CAN_TxHeader, CAN_TxData, &CAN_TxMailbox);
        CAN_Received = 0;
    }

}
