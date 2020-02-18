extern CAN_TxHeaderTypeDef      CAN_TxHeader;
extern CAN_RxHeaderTypeDef      CAN_RxHeader;
extern volatile uint8_t         CAN_TxData[8];
extern volatile uint8_t         CAN_RxData[8];
extern volatile uint32_t        CAN_TxMailbox;

extern volatile uint8_t         CAN_Received; // CAN data received flag


void CAN_OBD_Init();

void CAN_OBD_Response(uint8_t MAP, uint16_t rpm, uint8_t SPEED, uint8_t THROTTLE, uint8_t COOLANT_TEMP, uint8_t OIL_TEMP, uint16_t FUELPRESS);
