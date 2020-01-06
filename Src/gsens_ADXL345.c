//#include "main.h"
#include "i2c.h"
#include "gsens_ADXL345.h"

#define ADXL0_ADDR 0x1D
// ADXL345     I2C
// SDO/ALT   Address
//    H        0x1D
//    L        0x53

void ADXL345_RegWrite(uint8_t slv_addr, uint8_t addr, uint8_t data){
  uint8_t i2c_buf[2];
  i2c_buf[0] = addr;
  i2c_buf[1] = data;  
  HAL_I2C_Master_Transmit(&hi2c1, slv_addr << 1, &i2c_buf, 2, 10);
}

int8_t ADXL345_RegRead_1byte(uint8_t slv_addr, uint8_t addr){
  uint8_t data;
  HAL_I2C_Master_Transmit(&hi2c1, slv_addr << 1, &addr, 1, 10);
  HAL_I2C_Master_Receive(&hi2c1, slv_addr << 1, &data, 1, 10);
  return data;
}

uint8_t Gsens_ADXL345_Init(uint8_t ch){
  uint8_t     i2c_data;
  uint8_t     Gsens_EN;


  // DEIVID
  i2c_data = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x00);
  if( i2c_data == 0xE5 ){
    Gsens_EN = 1;
  }else{
    Gsens_EN = 0;
  }

  // POWER_CTL
  ADXL345_RegWrite(ADXL0_ADDR, 0x2D, 0x08);
  // bit 3    Measure   1'b1
//  i2c_data = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x2D); // Check

  // DATA_FORMAT
  ADXL345_RegWrite(ADXL0_ADDR, 0x31, 0x05);
  // bit 2    Justify   1'b1
  // bit 1:0  Range     2'b01
//  i2c_data = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x31); // Check "POWER_CTL"

  return Gsens_EN;

}

/*
int8_t Gsens_ADXL345_Init(char axis, uint8_t ch){
  int8_t  G_code;
  int16_t G_norm;

  // Justify = 1
  Gsens_X1 = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x33); // Check "DATAX1" (MSB side)
  Gsens_Y1 = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x35); // Check "DATAY1" (MSB side)
  Gsens_Z1 = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x37); // Check "DATAZ1" (MSB side)
  // Acceration 1G = 100
  Gsens_X = 400 * (int8_t)Gsens_X1 / 128; // unsigned->signed & scaling
  Gsens_Y = 400 * (int8_t)Gsens_Y1 / 128; // unsigned->signed & scaling
  Gsens_Z = 400 * (int8_t)Gsens_Z1 / 128; // unsigned->signed & scaling

  retutn G_norm;
}
*/