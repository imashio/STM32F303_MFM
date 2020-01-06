#define ADXL0_ADDR 0x1D
// ADXL345     I2C
// SDO/ALT   Address
//    H        0x1D
//    L        0x53

void ADXL345_RegWrite(uint8_t slv_addr, uint8_t addr, uint8_t data);

int8_t ADXL345_RegRead_1byte(uint8_t slv_addr, uint8_t addr);

uint8_t Gsens_ADXL345_Init(uint8_t ch);
