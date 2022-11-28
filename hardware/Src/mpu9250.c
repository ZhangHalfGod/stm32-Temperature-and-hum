#include "mpu9250.h"
#include "i2c.h"

#define BufferSize  10

unsigned char TX_DATA[4];                               	 //
unsigned char BUF[10];                                           //
char  test=0; 			                        	 //
short T_X,T_Y,T_Z,T_T;	                                	 //
uint16_t ACC_X = 0,ACC_Y = 0,ACC_Z = 0,ACC_T = 0;		 //
uint16_t GYRO_X = 0,GYRO_Y = 0,GYRO_Z = 0,GYRO_T = 0;		 //
uint16_t MAG_X = 0,MAG_Y = 0,MAG_Z = 0,MAG_T = 0;		 //

uint8_t WriteBuffer[BufferSize] = {0},ReadBuffer[BufferSize] = {0};

void Init_MPU9250(void)
{
  //½â³ýÐÝÃß×´Ì¬
  WriteBuffer[0] = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS_Write, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, WriteBuffer, 1, 0x10);
  WriteBuffer[0] = 0x07;
  HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS_Write, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, WriteBuffer, 1, 0x10);
   WriteBuffer[0] = 0x06;
  HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS_Write, CONFIG, I2C_MEMADD_SIZE_8BIT, WriteBuffer, 1, 0x10);
   WriteBuffer[0] = 0x18;
  HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS_Write, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, WriteBuffer, 1, 0x10);
   WriteBuffer[0] = 0x01;
  HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS_Write, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, WriteBuffer, 1, 0x10);
}

//******MPU9250****************************************
void READ_MPU9250_ACCEL(void)
{
  
  HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDRESS_Read, ACCEL_XOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDRESS_Read, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
   ACC_X=	(ReadBuffer[1]<<8)|ReadBuffer[0];
   ACC_X/=164; 						   //?¨¢¨¨?????X?¨¢¨ºy?Y
   
  HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDRESS_Read, ACCEL_YOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[2],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDRESS_Read, ACCEL_YOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[3],1, 0x10);
   ACC_Y=	(ReadBuffer[3]<<8)|ReadBuffer[2];
   ACC_Y/=164; 						   //?¨¢¨¨?????Y?¨¢¨ºy?Y
   
   HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDRESS_Read, ACCEL_ZOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[4],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDRESS_Read, ACCEL_ZOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[5],1, 0x10);
   ACC_Z=	(ReadBuffer[5]<<8)|ReadBuffer[4];
   ACC_Z/=164; 					       //?¨¢¨¨?????Z?¨¢¨ºy?Y
}

void READ_MPU9250_GYRO(void)
{ 
  HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS_Read, GYRO_XOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS_Read, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);
   GYRO_X=	(ReadBuffer[1]<<8)|ReadBuffer[0];
   GYRO_X/=16.4; 						   //?¨¢¨¨?????X?¨¢¨ºy?Y

   HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS_Read, GYRO_YOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[2],1, 0x10);
   HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS_Read, GYRO_YOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[3],1, 0x10);
   GYRO_Y=	(ReadBuffer[3]<<8)|ReadBuffer[2];
   GYRO_Y/=16.4; 						   //?¨¢¨¨?????Y?¨¢¨ºy?Y
   
  HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS_Read, GYRO_ZOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[4],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS_Read, GYRO_ZOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[5],1, 0x10);

   GYRO_Z=	(ReadBuffer[5]<<8)|ReadBuffer[4];
   GYRO_Z/=16.4; 					       //?¨¢¨¨?????Z?¨¢¨ºy?Y

}


void READ_MPU9250_MAG(void)
{ 
  WriteBuffer[0] = 0x02;
  HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS_Write, 0x37, I2C_MEMADD_SIZE_8BIT, WriteBuffer, 1, 0x10);//turn on Bypass Mode 
  HAL_Delay(10);
  WriteBuffer[0] = 0x01;
  HAL_I2C_Mem_Write(&hi2c1, MAG_ADDRESS_Write, 0x0A, I2C_MEMADD_SIZE_8BIT, WriteBuffer, 1, 0x10);//turn on Bypass Mode 	
  HAL_Delay(10);
  HAL_I2C_Mem_Read(&hi2c1, MAG_ADDRESS_Read, MAG_XOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, MAG_ADDRESS_Read, MAG_XOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);
  MAG_X = (ReadBuffer[1]<<8) | ReadBuffer[0];
   
  HAL_I2C_Mem_Read(&hi2c1, MAG_ADDRESS_Read, MAG_YOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[2],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, MAG_ADDRESS_Read, MAG_YOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[3],1, 0x10);
  MAG_Y = (ReadBuffer[3]<<8)|ReadBuffer[2];   //?¨¢¨¨?????Y?¨¢¨ºy?Y
   
  HAL_I2C_Mem_Read(&hi2c1, MAG_ADDRESS_Read, MAG_ZOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[4],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, MAG_ADDRESS_Read, MAG_ZOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[5],1, 0x10);	 
  MAG_Z = (ReadBuffer[5]<<8)|ReadBuffer[4];
 					  
}

void Read_Mpu9250(void)
{
        READ_MPU9250_ACCEL();     //¼ÓËÙ¶È
#ifdef DEBUG_PRINTF  
        printf("ACC_X:%d,ACC_Y:%d,ACC_Z:%d\n",ACC_X,ACC_Y,ACC_Z);  //ÈýÖá
#endif

	READ_MPU9250_GYRO();      //ÍÓÂÝÒÇ£¨ÂÞÅÌ£©

#ifdef DEBUG_PRINTF  
        printf("GYRO_X:%d,GYRO_Y:%d,GYRO_Z:%d\n",GYRO_X,GYRO_Y,GYRO_Z); //ÍÓÂÝÒÇ£¨ÂÞÅÌ£©
#endif	
         READ_MPU9250_MAG();	   //´Å¸ÐÓ¦
        
#ifdef DEBUG_PRINTF  
        printf("MAG_X:%d,MAG_Y:%d,MAG_Z:%d\n",MAG_X,MAG_Y,MAG_Z);  //´Å¸ÐÓ¦
#endif
}



