extern CAN_TxHeaderTypeDef      CAN_TxHeader;
extern CAN_RxHeaderTypeDef      CAN_RxHeader;
extern volatile uint8_t         CAN_TxData[8];
extern volatile uint8_t         CAN_RxData[8];
extern volatile uint32_t        CAN_TxMailbox;

extern volatile uint8_t         CAN_Received; // CAN data received flag


void CAN_OBD_Init();

void CAN_OBD_Response(uint8_t MAP, uint16_t rpm, uint8_t Speed, uint8_t Throttle, uint8_t CoolantTemp, uint8_t OilTemp, uint16_t FuelPress, uint8_t IntakeAirTemp);
