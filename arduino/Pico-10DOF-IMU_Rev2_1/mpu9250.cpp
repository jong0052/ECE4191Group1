#include "mpu9250.h"
#include <stdbool.h>
#include <Wire.h>

// Display Code 
#include <Arduino_GFX_Library.h>
// Display 
//#define TFT_BL 13
//Arduino_DataBus *bus = new Arduino_RPiPicoSPI(8 /* DC */, 9 /* CS */, 10 /* SCK */, 11 /* MOSI */, UINT8_MAX /* MISO */, spi1 /* spi */);
//Arduino_GFX *gfx = new Arduino_ST7789(bus, 12 /* RST */, 3 /* rotation */, true /* IPS */, 135, 240, 52, 30, 60, 40);

int16_t magn[3];
int16_t accel[3], gyro[3];
unsigned char BUF[10];
MPU9250_TypeDef MPU9250_Offset={0};
MPU9250_TypeDef_Off MPU9250_Magn_Offset={0};

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Initializes MPU9250
  * @param  None
  * @retval None
  */
void MPU9250_Init(void)
{
  I2C_WriteOneByte(GYRO_ADDRESS,PWR_MGMT_1, 0x00);
  I2C_WriteOneByte(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
  I2C_WriteOneByte(GYRO_ADDRESS,CONFIG, 0x06);
  I2C_WriteOneByte(GYRO_ADDRESS,GYRO_CONFIG, 0x10);
  I2C_WriteOneByte(GYRO_ADDRESS,ACCEL_CONFIG, 0x01);
  
  delay(10);
  MPU9250_InitGyrOffset();
}

/**
  * @brief Get accelerometer datas
  * @param  None
  * @retval None
  */
void MPU9250_READ_ACCEL(void)
{ 
   uint8_t i;
   int16_t InBuffer[3] = {0}; 
   static int32_t OutBuffer[3] = {0};
   static MPU9250_AvgTypeDef MPU9250_Filter[3];

   BUF[0]=I2C_ReadOneByte(ACCEL_ADDRESS,ACCEL_XOUT_L); 
   BUF[1]=I2C_ReadOneByte(ACCEL_ADDRESS,ACCEL_XOUT_H);
   InBuffer[0]= (BUF[1]<<8)|BUF[0];

   BUF[2]=I2C_ReadOneByte(ACCEL_ADDRESS,ACCEL_YOUT_L);
   BUF[3]=I2C_ReadOneByte(ACCEL_ADDRESS,ACCEL_YOUT_H);
   InBuffer[1]= (BUF[3]<<8)|BUF[2];
             
   BUF[4]=I2C_ReadOneByte(ACCEL_ADDRESS,ACCEL_ZOUT_L);
   BUF[5]=I2C_ReadOneByte(ACCEL_ADDRESS,ACCEL_ZOUT_H);
   InBuffer[2]= (BUF[5]<<8)|BUF[4];            
   
   for(i = 0; i < 3; i ++)  
   {
      MPU9250_CalAvgValue(&MPU9250_Filter[i].Index, MPU9250_Filter[i].AvgBuffer, InBuffer[i], OutBuffer + i);
   }
   accel[0] = *(OutBuffer + 0);
   accel[1] = *(OutBuffer + 1);
   accel[2] = *(OutBuffer + 2); 
}

/**
  * @brief Get gyroscopes datas
  * @param  None
  * @retval None
  */
void MPU9250_READ_GYRO(void)
{ 
   uint8_t i;
   int16_t InBuffer[3] = {0}; 
   static int32_t OutBuffer[3] = {0};
   static MPU9250_AvgTypeDef MPU9250_Filter[3];

   BUF[0]=I2C_ReadOneByte(GYRO_ADDRESS,GYRO_XOUT_L); 
   BUF[1]=I2C_ReadOneByte(GYRO_ADDRESS,GYRO_XOUT_H);
   InBuffer[0]= (BUF[1]<<8)|BUF[0];
   
   BUF[2]=I2C_ReadOneByte(GYRO_ADDRESS,GYRO_YOUT_L);
   BUF[3]=I2C_ReadOneByte(GYRO_ADDRESS,GYRO_YOUT_H);
   InBuffer[1] = (BUF[3]<<8)|BUF[2];
    
   BUF[4]=I2C_ReadOneByte(GYRO_ADDRESS,GYRO_ZOUT_L);
   BUF[5]=I2C_ReadOneByte(GYRO_ADDRESS,GYRO_ZOUT_H);
   InBuffer[2] = (BUF[5]<<8)|BUF[4];  

   for(i = 0; i < 3; i ++)  
   {
      MPU9250_CalAvgValue(&MPU9250_Filter[i].Index, MPU9250_Filter[i].AvgBuffer, InBuffer[i], OutBuffer + i);
   }
   gyro[0] = *(OutBuffer + 0) - MPU9250_Offset.X;
   gyro[1] = *(OutBuffer + 1) - MPU9250_Offset.Y;
   gyro[2] = *(OutBuffer + 2) - MPU9250_Offset.Z;
}

/**
  * @brief Get compass datas
  * @param  None
  * @retval None
  */
void MPU9250_READ_MAG(void)
{ 
   uint8_t i;
   int16_t InBuffer[3] = {0}; 
   static int32_t OutBuffer[3] = {0};
   static MPU9250_AvgTypeDef MPU9250_Filter[3];

    I2C_WriteOneByte(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
    delay(10);
    I2C_WriteOneByte(MAG_ADDRESS,0x0A,0x01);  
    delay(10);
    BUF[0]=I2C_ReadOneByte (MAG_ADDRESS,MAG_XOUT_L);
    BUF[1]=I2C_ReadOneByte (MAG_ADDRESS,MAG_XOUT_H);
    InBuffer[1] =(BUF[1]<<8)|BUF[0];

    BUF[2]=I2C_ReadOneByte(MAG_ADDRESS,MAG_YOUT_L);
    BUF[3]=I2C_ReadOneByte(MAG_ADDRESS,MAG_YOUT_H);
    InBuffer[0] = (BUF[3]<<8)|BUF[2];
    
    BUF[4]=I2C_ReadOneByte(MAG_ADDRESS,MAG_ZOUT_L);
    BUF[5]=I2C_ReadOneByte(MAG_ADDRESS,MAG_ZOUT_H);
    InBuffer[2] = (BUF[5]<<8)|BUF[4]; 
    InBuffer[2] = -InBuffer[2];
   
   for(i = 0; i < 3; i ++)  
   {
      MPU9250_CalAvgValue(&MPU9250_Filter[i].Index, MPU9250_Filter[i].AvgBuffer, InBuffer[i], OutBuffer + i);
   } 
      magn[0] = *(OutBuffer + 0)-MPU9250_Magn_Offset.X_Off_Err;
      magn[1] = *(OutBuffer + 1)-MPU9250_Magn_Offset.Y_Off_Err;
      magn[2] = *(OutBuffer + 2)-MPU9250_Magn_Offset.Z_Off_Err;
}

/**
  * @brief  Check MPU9250,ensure communication succeed
  * @param  None
  * @retval true: communicate succeed
  *               false: communicate fualt 
  */
bool MPU9250_Check(void) 
{   
    if(WHO_AM_I_VAL == I2C_ReadOneByte(0x68, WHO_AM_I))
    {
      return true;
    }
    else 
    {
      return false;
    }
}

/**
  * @brief  Digital filter
  * @param *pIndex:
  * @param *pAvgBuffer:
  * @param InVal:
  * @param pOutVal:
  *
  * @retval None
  *               
  */
void MPU9250_CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{ 
  uint8_t i;
  
  *(pAvgBuffer + ((*pIndex) ++)) = InVal;
    *pIndex &= 0x07;
    
    *pOutVal = 0;
  for(i = 0; i < 8; i ++) 
    {
      *pOutVal += *(pAvgBuffer + i);
    }
    *pOutVal >>= 3;
}
/**
  * @brief  Initializes gyroscopes offset
  * @param  None
  * @retval None
  */
void MPU9250_InitGyrOffset(void)
{
  uint8_t i;
  int32_t TempGx = 0, TempGy = 0, TempGz = 0;
  
  for(i = 0; i < 128; i ++)
  {
    MPU9250_READ_GYRO();
    
    TempGx += gyro[0];
    TempGy += gyro[1];
    TempGz += gyro[2];

    delay(1);
  }

  MPU9250_Offset.X = TempGx >> 7;
  MPU9250_Offset.Y = TempGy >> 7;
  MPU9250_Offset.Z = TempGz >> 7;

}

char I2C_ReadOneByte(uint8_t addr,char reg)
{
  char data; // `data` will store the register data

  // Initialize the Tx buffer
  Wire1.beginTransmission(addr);
  // Put slave register address in Tx buffer
  Wire1.write(reg);
  // Send the Tx buffer, but send a restart to keep connection alive
  Wire1.endTransmission(false);
  // Read one byte from slave register address
  Wire1.requestFrom(addr, (uint8_t) 1);
  // Fill Rx buffer with result
  data = Wire1.read();
  // Return data read from slave register
  return data;
}

void I2C_WriteOneByte(uint8_t addr, char reg, char value)
{
  Wire1.beginTransmission(addr);  // Initialize the Tx buffer
  Wire1.write(reg);                           // Put slave register address in Tx buffer
  Wire1.write(value);                        // Put data in Tx buffer
  Wire1.endTransmission();                   //  Send the Tx buffer
}

void calibrateMagn(Arduino_GFX *gfx)
{
  int16_t temp[9];
  //Serial.printf("keep 10dof-imu device horizontal and it will read x y z axis offset value after 4 seconds\n");
  gfx->fillScreen(BLACK);
  gfx->setTextColor(RED);
  gfx->setTextSize(2/* x scale */, 2 /* y scale */, 1 /* pixel_margin */);
  gfx->setCursor(20, 10);
  gfx->println("keep 10dof-imu device horizontal and it will read x y z axis offset value after 5 seconds\n");
  delay(5000);
  //Serial.printf("start read all axises offset value\n");
  //gfx->println("start read all axises offset value\n");
  MPU9250_READ_MAG();
  temp[0] = magn[0];
  temp[1] = magn[1];
  temp[2] = magn[2];
  
  //Serial.printf("rotate z axis 180 degrees and it will read all axises offset value after 4 seconds\n");
  gfx->fillScreen(BLACK);
  gfx->setTextColor(BLUE);
  gfx->setCursor(20, 10);
  gfx->println("rotate z axis 180 degrees and it will read all axises offset value after 5 seconds\n");
  delay(5000);
  //Serial.printf("start read all axises offset value\n");
  //gfx->println("start read all axises offset value\n");
  MPU9250_READ_MAG();
  temp[3] = magn[0];
  temp[4] = magn[1];
  temp[5] = magn[2];

  //Serial.printf("flip 10dof-imu device and keep it horizontal and it will read all axises offset value after 8 seconds\n");
  gfx->fillScreen(BLACK);
  gfx->setTextColor(GREEN);
  gfx->setCursor(20, 10);
  gfx->println("flip 10dof-imu device and keep it horizontal and it will read all axises offset value after 5 seconds\n");
  delay(5000);
  //Serial.printf("start read all axises offset value\n");
  //gfx->println("start read all axises offset value\n");
  MPU9250_READ_MAG();
  temp[6] = magn[0];
  temp[7] = magn[1];
  temp[8] = magn[2];

  MPU9250_Magn_Offset.X_Off_Err = (temp[0]+temp[3])/2;
  MPU9250_Magn_Offset.Y_Off_Err = (temp[1]+temp[4])/2;
  MPU9250_Magn_Offset.Z_Off_Err = (temp[5]+temp[8])/2;

  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE);
  gfx->setCursor(20, 10);
  gfx->println("Move The Robot Back You have 4 seconds\n");
  delay(4000);
}

#ifdef __cplusplus
}
#endif
