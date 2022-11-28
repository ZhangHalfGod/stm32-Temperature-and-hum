#ifndef FS_PROTOCOL_H
#define FS_PROTOCOL_H

#include "stdint.h"

typedef __INT8_T_TYPE__   int8;
typedef __UINT8_T_TYPE__ uint8;

typedef __INT16_T_TYPE__   int16;
typedef __UINT16_T_TYPE__ uint16;

typedef __INT32_T_TYPE__   int32;
typedef __UINT32_T_TYPE__ uint32;

/******************************************************************************
FS协议相关,一般不需要修改
*******************************************************************************/
/***************************************************************************************
协议格式：
|  标识   | 数据长度 |  数据偏移 | 数据类型 | 网络类型 |  设备地址 |   设备类型  |  电量   |数据域   |   校验和   |
|  0x21   |   1byte |   1byte   |  1byte   |  1byte  |   2byte   |    1byte   |  1byte  |256byte  |   1byte    |
******************************************************************************************/

//定义发送的数据
#define HEADER                 0x21

//定义发送数据数据域的最大长度                  
#define SENDDATALENGTH         18     //数据域长度


#define OFFSET                 9     //数据偏移长度
#define DATATYPEDATA           0x00  //数据类型   0x00：数据 0x01：命令 
#define DATATYPECMD            0x01  //数据类型   0x00：数据 0x01：命令
#define NWKTYPE                'W'   //网络类型

#define   FAN               'f'   //设备类型 风扇控
#define   LIGHT             'L'   //设备类型 光强
#define   TEMPHUM           'T'   //设备类型 温湿度
#define   MPU9250           'M'   //设备类型 九轴

#define   GAS               'G'   //设备类型 可燃气体
#define   FLAME             0x26  //设备类型 火焰  "F" - 0x20
#define   PHOTOACTOR        'P'   //设备类型 光电
#define   LAMP              'l'   //设备类型 灯控

#define   RELAY             'r'   //设备类型 继电器控
#define   BEEP              'b'   //设备类型 蜂鸣器控
#define   POTENTIOMETER     0x30  //设备类型 电位器  'P'- 0x20
#define   RAINFALL          'R' //设备类型 雨滴 'R' 

#define   INFRARED          'I'   //设备类型 人体红外 
#define   ULTRASONIC        'U'   //设备类型 超声波
#define   TOUCH             0x34 //设备类型 触摸 'T' - 0x20
#define   SOUND             'S'  //设备类型 声控开关
#define   FOG               'F'  //设备类型 烟雾

#define   MPU6050           0x2D  //设备类型 三轴 'M'- 0x20 
#define   BMP180            'B'   //设备类型 气压
#define   HALL              'H'           //设备类型 霍尔  ‘H’

#define   Err               'E'  //未识别传感器

#define    CMDON        '1'
#define    CMDOFF       '0'

  
//#define RXLENGHT   512   //USART2 串口缓冲区
  
extern uint8_t g_SendDataLength; 
extern uint8_t theMessageData[OFFSET + SENDDATALENGTH +1];

extern uint8_t SensorStatusflag;  

//光敏值
//extern uint8_t AdcData_H ;
//extern uint8_t AdcData_L ;

//发送数据结构体
typedef struct
{
  uint8_t   Header;               //标头
  uint8_t   DataLength;           //Number of bytes in TransData
  uint8_t   Offset;               //偏移量
  uint8_t   DataType;             //数据类型
  uint8_t   NwkType;              //网络类型
  uint8_t   ShortAddr[2];            //网络地址
  uint8_t   DeviceType;           //设备类型
  uint8_t   Bat;                  //电量
  uint8_t   Data[0];              //数据
  //  uint8_t   CRC8;                 //校验
} SendData_t;



#define ZIGBEE_SENSOR_MAXIMAL_NUM 30   //可以保存30个节点地址

typedef struct {
  uint16_t zigbee_sensor_id;              //跟fs协议相关，占用两个字节.
  uint16_t zigbee_sensor_shortaddr;       //跟zigbee协议相关，是指zigbee_device在PAN中的short_addr.
} zigbee_sensor_addr_t;

typedef struct {
  zigbee_sensor_addr_t zigbee_sensor_addr_table[ZIGBEE_SENSOR_MAXIMAL_NUM];     //存储传感器的地址信息（sensor_id + endpoint_shortaddr）
  uint8_t zigbee_sensor_nums;                             //已经连接的传感器的个数
} zigbee_sensors_desc_t;

//extern byte GenericApp_TransID;  // This is the unique message ID (counter)
//extern afAddrType_t GenericApp_DstAddr;
//extern endPointDesc_t GenericApp_epDesc;  
////extern devStates_t GenericApp_NwkState;

extern void delay_ms(uint16_t ms);
extern uint8_t crc8(uint8_t *data, uint8_t length);
extern void EndGettAnalyzeProtocol(SendData_t *PacktData,uint16_t Light);
extern void EndSendAnalyzeProtocol(void);
extern void num_to_str(unsigned int num, unsigned char *pstr, unsigned char num_len,uint8 type);
extern void Add_DataMsgBuff(void);

#endif



