/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#include "i2c.h"
/* Private function prototypes -----------------------------------------------*/

int16 Accx,Accy,Accz;

#define	ADDRESS_Write   SlaveAddress | 0x00                //
#define	ADDRESS_Read   SlaveAddress | 0x01	                //



//**************************************
//初始化MPU6050
//**************************************
void InitMPU6050(void)
{
  
  uint8_t WriteBuffer[10] = {0};
  
    //解除休眠状态
  WriteBuffer[0] = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS_Write, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, WriteBuffer, 1, 0x10);
   WriteBuffer[0] = 0x07;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS_Write, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, WriteBuffer, 1, 0x10);      
   WriteBuffer[0] = 0x06;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS_Write, CONFIG, I2C_MEMADD_SIZE_8BIT, WriteBuffer, 1, 0x10);
     WriteBuffer[0] = 0x18;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS_Write, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, WriteBuffer, 1, 0x10);
     WriteBuffer[0] = 0x01;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS_Write, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, WriteBuffer, 1, 0x10);

    HAL_Delay(10);
        mpu6050_verify(&Accx, &Accy, &Accz); //读取第一次的值
}
void  mpu6050_verify(int16 *x, int16 *y, int16 *z)
{
  uint8_t ReadBuffer[10] = {0};
  
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_XOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
        *x = (ReadBuffer[1]<<8)+ReadBuffer[0] ;
   HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_YOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_YOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
        *y = (ReadBuffer[1]<<8)+ReadBuffer[0] ;
        
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_ZOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_ZOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
        *z = (ReadBuffer[1]<<8)+ReadBuffer[0] ;
        
}
//**************************************
//合成数据
//**************************************    
void mpu6050_Test(int16 *x, int16 *y, int16 *z)
{
  uint8_t ReadBuffer[10] = {0};
         
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_XOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
        *x = (ReadBuffer[1]<<8)+ReadBuffer[0] ;
        *x -= Accx;
     HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_YOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_YOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
        *y = (ReadBuffer[1]<<8)+ReadBuffer[0] ;
        *y -= Accy;
    HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_ZOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_ZOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
        *z = (ReadBuffer[1]<<8)+ReadBuffer[0] ;
        *z -= Accz;
//	return (H<<8)+L;   //合成数据

//          uint8 Display_Buf[6] ={0};        
//          sprintf((char*)Display_Buf,"%d", *x );
//                    HalLcdWriteString((char*)Display_Buf,1);
//          sprintf((char*)Display_Buf,"%d",*y);
//                  HalLcdWriteString((char*)Display_Buf,1);
//          sprintf((char*)Display_Buf,"%d",*z);
//          
//          HalLcdWriteString((char*)Display_Buf,1);
        
        printf("%d,%d,%d\n",*x,*y,*z);
}







