#ifndef __HMPU9250_H__
#define __HMPU9250_H__

#include "stdint.h"

#define	GYRO_ADDRESS   0xD0             	        //
#define	GYRO_ADDRESS_Write   0xD0 | 0x00                //
#define	GYRO_ADDRESS_Read   0xD0 | 0x01	                //

#define   MAG_ADDRESS    0x18                           //
#define	MAG_ADDRESS_Write   0x18 | 0x00                 //
#define	MAG_ADDRESS_Read   0x18 | 0x01	                //

#define   ACCEL_ADDRESS  0xD0                           //
#define	ACCEL_ADDRESS_Write   0xD0 | 0x00               //
#define	ACCEL_ADDRESS_Read   0xD0 | 0x01	        //


//****************************************
#define	SMPLRT_DIV		0x19	                //
#define	CONFIG			0x1A	                //
#define	GYRO_CONFIG		0x1B	                //
#define	ACCEL_CONFIG	0x1C	                        //

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

		
#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08


#define	PWR_MGMT_1		0x6B	//
#define	WHO_AM_I		  0x75	//

extern uint16_t ACC_X,ACC_Y,ACC_Z,ACC_T;		 //X,Y,Z
extern uint16_t GYRO_X,GYRO_Y,GYRO_Z,GYRO_T;		 //X,Y,Z
extern uint16_t MAG_X,MAG_Y,MAG_Z,MAG_T;		 //X,Y,Z


/*-----------------------------------------------*/
void Init_MPU9250(void);
void READ_MPU9250_ACCEL(void);
void READ_MPU9250_GYRO(void);
void READ_MPU9250_MAG(void);
void Read_Mpu9250(void);
#endif

