#ifndef __BMP180_H__
#define __BMP180_H__
#include "fs_protocol.h"

extern uint8 Bmp180Temp ;
extern uint16 Bmp180AirPressure ;
extern uint16 Bmp180Altitude ;

#define BMP180_DEV_ADDR                0xEE
#define BMP180_WAIT_UT                 45 //waiting time in 0.1ms
#define BMP180_WAIT_UP_LOWPOWER        45
#define BMP180_WAIT_UP_STANDARD        75
#define BMP180_WAIT_UP_HIHGRES         135
#define BMP180_WAIT_UP_ULTRAHIGHRES    255
//#define I2C_DELAY 				ANBT_I2C_Delay(100000)

//oversampling settings
#define BMP180_OSS_LOWPOWER            0
#define BMP180_OSS_STANDARD            1
#define BMP180_OSS_HIGHRES             2
#define BMP180_OSS_ULTRAHIGHRES        0

//commands
#define BMP180_COM_CTRL_MEAS_TEMP      0x2E
#define BMP180_COM_CTRL_MEAS_PRESS     0x34
#define BMP180_COM_SOFT_RESET          0xB6

//control reg bits
#define BMP180_BIT_CTRL_MEAS_SCO       (1<<5)
#define BMP180_BIT_CTRL_MEAS_SCO_SHIFT 5
#define BMP180_BIT_CTRL_MEAS_OSS       (0x3<<6)
#define BMP180_BIT_CTRL_MEAS_OSS_SHIFT 6

//register values
#define BMP180_VALUE_ID                0x55

#define BMP180_CALREGS_BASE_ADDRESS    0xAA
#define BMP180_CTRLREGS_BASE_ADDRESS   0xF4

//addresses of control registers
#define BMP180_ADDR_OUT_XLSB           0xF8
#define BMP180_ADDR_OUT_LSB            0xF7
#define BMP180_ADDR_OUT_MSB            0xF6
#define BMP180_ADDR_CTRL_MEAS          0xF4
#define BMP180_ADDR_SOFT_RESET         0xE0
#define BMP180_ADDR_ID                 0xD0

//calibration registers
typedef struct{
	//integrated registers
	volatile short AC1;
	volatile short AC2;
	volatile short AC3;
	volatile unsigned short AC4;
	volatile unsigned short AC5;
	volatile unsigned short AC6;
	volatile short B1;
	volatile short B2;
	volatile short MB;
	volatile short MC;
	volatile short MD;
	//calculated parameters
	long B5;
}BMP180_CAL_TypeDef;

void Bmp180_Init(void);
long BMP180_READ_UT(void);
short BMP180_CALC_TEMP(unsigned long ut,BMP180_CAL_TypeDef *reg_data);
unsigned char BMP180_READ_UP(long *up, unsigned char oss);
long BMP180_CALC_PRESS(unsigned long up,BMP180_CAL_TypeDef *reg_data, unsigned char oss);
void Read_Pressure(void);

void HalLcdWriteStringValue_1( char *title, uint32 value, uint8 format, uint8 line );
#endif

