#include "bmp180.h"
#include "math.h"
#include "i2c.h"

#define BMP1800ADDR  0xEE  
#define	BMP1800ADDR_Write   BMP1800ADDR | 0x00                //
#define	BMP1800ADDR_Read   BMP1800ADDR | 0x01	                //

BMP180_CAL_TypeDef calregs;

uint8 Bmp180Temp = 0;    //温度
uint16 Bmp180AirPressure = 0;  //气压
uint16 Bmp180Altitude = 0;     //高度

short ac1;
short ac2;
short ac3;
unsigned short ac4;
unsigned short ac5;
unsigned short ac6;
short b1;
short b2;
short mb;
short mc;
short md;
long b5;

void Bmp180_Init(void)
{ 
  uint8 buf[2] = {0};
  
  HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, 0xaa, I2C_MEMADD_SIZE_8BIT,&buf[0],2, 0x10);
  ac1 = buf[0]<<8|buf[1];
  HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, 0xac, I2C_MEMADD_SIZE_8BIT,&buf[0],2, 0x10);
  ac2 = buf[0]<<8|buf[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, 0xae, I2C_MEMADD_SIZE_8BIT,&buf[0],2, 0x10);
  ac3 = buf[0]<<8|buf[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, 0xb0, I2C_MEMADD_SIZE_8BIT,&buf[0],2, 0x10);
  ac4 = buf[0]<<8|buf[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, 0xb2, I2C_MEMADD_SIZE_8BIT,&buf[0],2, 0x10);
  ac5 = buf[0]<<8|buf[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, 0xb4, I2C_MEMADD_SIZE_8BIT,&buf[0],2, 0x10);
  ac6 = buf[0]<<8|buf[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, 0xb6, I2C_MEMADD_SIZE_8BIT,&buf[0],2, 0x10);
  b1 = buf[0]<<8|buf[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, 0xb8, I2C_MEMADD_SIZE_8BIT,&buf[0],2, 0x10);
  b2 = buf[0]<<8|buf[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, 0xba, I2C_MEMADD_SIZE_8BIT,&buf[0],2, 0x10);
  mb = buf[0]<<8|buf[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, 0xbc, I2C_MEMADD_SIZE_8BIT,&buf[0],2, 0x10);
  mc = buf[0]<<8|buf[1]; 
    HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, 0xbe, I2C_MEMADD_SIZE_8BIT,&buf[0],2, 0x10);
  md = buf[0]<<8|buf[1]; 
  
//  HalLcdWriteStringValue( "ac1:", ac1, 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "ac2:", ac2, 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "ac3:", ac3, 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "ac4:", ac4, 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "ac5:", ac5, 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "ac6:", ac6, 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "b1:", b1, 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "b2:", b2, 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "mb:", mb, 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "mc:", mc, 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "md:", md, 10, HAL_LCD_LINE_2 );
  
}

long BMP180_READ_UT(void)
{
  uint8 data[2];
  unsigned long ut;
  
  data[0]=BMP180_COM_CTRL_MEAS_TEMP;
  
  //	output = BMP_WRITE(I2C_USED, BMP180_DEV_ADDR, BMP180_ADDR_CTRL_MEAS, data);
//  I2c_Write_one_Byte(data[0], BMP180_ADDR_CTRL_MEAS,BMP180_DEV_ADDR);
    HAL_I2C_Mem_Write(&hi2c1, BMP1800ADDR_Write, BMP180_ADDR_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &data[0], 1, 0x10);
  //必须有4.5ms延时
  HAL_Delay(7);
  //        output = BMP_READ(I2C_USED, BMP180_DEV_ADDR, BMP180_ADDR_OUT_MSB, data, 2);
//  I2c_ReadByte(data,2, BMP180_ADDR_OUT_MSB,BMP1800ADDR);
    HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, BMP180_ADDR_OUT_MSB, I2C_MEMADD_SIZE_8BIT,&data[0],2, 0x10);
  ut = (data[0]<<8) | data[1]; 
  
  return ut;
}

short BMP180_CALC_TEMP(unsigned long ut,BMP180_CAL_TypeDef *reg_data){
  short temperature;
  long x1, x2;
  x1=(((long)ut - (long)ac6) * (long)ac5) >> 15;
  x2=((long)mc << 11) / (x1 + md);
  b5 = x1+x2;
  temperature = (b5+8) >> 4;
  return temperature;
}

unsigned char BMP180_READ_UP(long *up, unsigned char oss){
  uint8 data[4]={0};
  uint32 MSB,LSB,XLSB;
  
  data[0]=BMP180_COM_CTRL_MEAS_PRESS | (oss<<BMP180_BIT_CTRL_MEAS_OSS_SHIFT);
  //	output = BMP_WRITE(I2C_USED, BMP180_DEV_ADDR, BMP180_ADDR_CTRL_MEAS, data);
//  I2c_Write_one_Byte(data[0], BMP180_ADDR_CTRL_MEAS,BMP180_DEV_ADDR);
   HAL_I2C_Mem_Write(&hi2c1, BMP1800ADDR_Write, BMP180_ADDR_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &data[0], 1, 0x10);
  //必须有4.5ms延时
  HAL_Delay(7);
  
  //	output = BMP_READ(I2C_USED, BMP180_DEV_ADDR, BMP180_ADDR_OUT_MSB, data, 3);
//  I2c_ReadByte(data,3, BMP180_ADDR_OUT_MSB,BMP180_DEV_ADDR);
    HAL_I2C_Mem_Read(&hi2c1, BMP1800ADDR_Read, BMP180_ADDR_OUT_MSB, I2C_MEMADD_SIZE_8BIT,&data[0],3, 0x10);
//  result->MSB = data[0];
//  result->LSB = data[1];
//  result->XLSB = data[2];
  
  
//  HalLcdWriteStringValue( "MSB:", data[0], 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "LSB:", data[1], 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "XLSB:", data[2], 10, HAL_LCD_LINE_2 );
 
  MSB  = data[0];
  LSB  = data[1];
  XLSB = data[2];   
  MSB = MSB << 16;
  LSB = LSB << 8;
  XLSB = XLSB;
   
//  HalLcdWriteStringValue( "MSB:", MSB, 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "LSB:", LSB, 10, HAL_LCD_LINE_2 );
//  HalLcdWriteStringValue( "XLSB:",XLSB, 10, HAL_LCD_LINE_2 );
  //*up = ((data[0] << 16) | (data[1] << 8;) | data[2]; )>>(8-oss);
  *up = (MSB+LSB+XLSB) >>(8-oss);
  
  return 1;
}

long BMP180_CALC_PRESS(unsigned long up,BMP180_CAL_TypeDef *reg_data, unsigned char oss){
  long pressure, x1, x2, x3, b3, b6;
  unsigned long b4, b7;
  
  b6=b5 - 4000;
  
  /*****calculate B3************/
  x1 = (b6*b6) >> 12;
  x1 *= b2;
  x1 >>= 11;
  x2 = (ac2*b6);
  x2 >>= 11;
  x3 = x1 + x2;
  b3 = ((((((long)ac1)<<2) + x3) << oss)+2) >> 2;
  
  /*****calculate B4************/
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6*b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
  b7 = ((unsigned long)(up - b3) * (50000>>oss));
  if (b7 < 0x80000000)
    pressure = (b7 << 1) / b4;
  else
    pressure = (b7 / b4) << 1;
  x1 = pressure >> 8;
  x1 *= x1;
  x1 = (x1 * 3038) >> 16;
  x2 = (pressure * -7357) >> 16;
  pressure += (x1 + x2 + 3791) >> 4;/* pressure in Pa*/
 
  return pressure;
}

void Read_Pressure(void)
{
  long ut=0, up=0, pressure=0;
  short temperature;
  unsigned char oss = BMP180_OSS_ULTRAHIGHRES;
  int32 Altitude = 0;       //定义海拔

  //温度
  ut =  BMP180_READ_UT();
//  HalLcdWriteStringValue( "ut:", ut, 10, HAL_LCD_LINE_2 );
  temperature = BMP180_CALC_TEMP(ut, &calregs);
  Bmp180Temp = temperature / 10;
  
  //气压
  BMP180_READ_UP(&up, oss);  
//  HalLcdWriteStringValue_1( "up:", up, 10, HAL_LCD_LINE_2 );
  pressure=BMP180_CALC_PRESS(up, &calregs, oss);
  Bmp180AirPressure = pressure /100;
  //高度
  Altitude = (int32)(44330.0 * (1.0-pow((float)(pressure) / 101325.0, 1.0/5.255)));
  
  //超出范围(为负值)
 if(Altitude < 0 ) //|| Altitude > 0x7fff)
    //Altitude = fabs(Altitude);
   Altitude =0;
 
  Bmp180Altitude = (uint16)Altitude;
      
//  printf( "temperature:%d\n", temperature);
//  printf( "Bmp180Temp:%d\n", Bmp180Temp);
//  printf( "pressure:%d\n", pressure );
//  printf( "Bmp180AirPressure:%d\n", Bmp180AirPressure );
//  printf( "Altitude:%d\n", Altitude);
//  printf( "Bmp180Altitude:%d\n", Bmp180Altitude);
}


