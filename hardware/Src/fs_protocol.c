#include "string.h"
#include "lcd.h"
#include "fs_protocol.h"
#include "bsp.h"
#include "usart.h"
#include "switch.h"

#include "adc.h"
#include "p9813.h"
#include "dht11.h"
#include "mpu9250.h"
#include "mpu6050.h"
#include "ultrasonic.h"
#include "bmp180.h"


uint8  SensorStatusflag = '0';  //开关类型的当前状态（1/0）  
uint8  g_SendDataLength = 0;  //保存发送的数据长度

//发送数据的缓存区
uint8 theMessageData[OFFSET + SENDDATALENGTH +1] = {HEADER,0x00,OFFSET,DATATYPEDATA,NWKTYPE,0x00,0x00,0x00,0x00};


int8 HeartRateNumber = 0;

int8 find_sensor_id(uint16 sensor_id);

void num_to_str(unsigned int num, unsigned char *pstr, unsigned char num_len,uint8 type)
{
  int8 dispnum[10] = {0};      
  unsigned char  count = 0, i = 0;
  uint8 number = 0;
  
  //等于0的情况下
  if(num == 0)
  {
    while (num_len)
    {
      if(num_len == 1)
        pstr[i++] = '0';  //显示的高字节
      else
        pstr[i++] = ' '; 
      num_len--;
    }
    pstr[i++] = '\0'; 
    return;
  }
  if(type == 10){
    //不等0的情况下，转化成十进制字符
    while(num != 0)
    {           
      dispnum[count] = num % 10 + '0';   
      num = num / 10;
      count++;
    }
  }else if(type == 16)
  {
    //不等0的情况下，转化成16进制字符
    while(num != 0)
    {           
      dispnum[count] = num % 16;
      
      if( 0 <= dispnum[count] && dispnum[count] <= 9)
      {
        dispnum[count] += '0';
      }
      else if(10<= dispnum[count] && dispnum[count] <= 15)
      {
        dispnum[count] += 55;
      }
      num = num / 16;
      count++;
    }
  }
  
  //如果不够4个长度则使用‘ ’填充
  number = num_len - count;
  
  while(number--)
  {
    if(type == 10){  //10进制填充‘ ’
    pstr[i++] = ' ';
    }
    if(type == 16){  //16进制填充‘ ’
    pstr[i++] = '0';
    }
  }
  while (count--) //转化后的数据位于高字节
    pstr[i++] = dispnum[count];
  //添加'\0'
  pstr[i++] = '\0';
}

/******************************************************************************
* Name:    CRC-8               x8+x2+x+1
* Poly:    0x07
* Init:    0x00
* Refin:   False
* Refout:  False
* Xorout:  0x00
* Note:
*****************************************************************************/
uint8 crc8(uint8 *data, uint8 length)
{
  uint8 i;
  uint8 crc = 0;        // Initial value
  while(length--)
  {
    crc ^= *data++;        // crc ^= *data; data++;
    for ( i = 0; i < 8; i++ )
    {
      if ( crc & 0x80 )
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}
/***********************************************************************************************************
* 函数名称:  EndGettAnalyzeProtocol(SendData_t *PacktData,uint16 Light)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: 接收协调器发送的命令
*
* 作    者: 2017/2/10, by SongLei
* 邮    箱: 1170152143@qq.com
************************************************************************************************************/
void  EndGettAnalyzeProtocol(SendData_t *PacktData,uint16 Light)
{
  uint8 Crc8Data;
  uint8 status = 0,i=0;
  uint16_t RxDeviceAddr = 0x0000;
  
  //验证数据是否正确 
  Crc8Data = crc8((uint8 *)PacktData, Light-1);

  if(Crc8Data == PacktData->Data[PacktData->DataLength])
  {     
    //HalUARTWrite(0,"CRC8 Success!",13);
    printf("CRC8 Success!\n");
  }
  else
  {
    //HalUARTWrite(0,"CRC8 Failed!",12);
  //  printf("CRC8 Failed!\n");
    return ;
  }

#ifdef DEBUG_PRINTF  
 for(i=0;i<Light;i++)
  {
    printf("%x ",((uint8*)PacktData)[i]);
  }
  printf("\n");
#endif
  
  //分析数据
  while(Light--)
  {
    switch(status)
    {
    case 0: //data header
      if(PacktData->Header == HEADER)
      {
        status++;  //下一个状态
      }
      else{
      }
      break;
    case 1://data light 
      if(PacktData->DataLength <= Light)
      { 
        status++;
      }
      else{ 
      }
      break;
    case 2://data 偏移
      if(PacktData->Offset == OFFSET)
      {
        status++;
      }
      else{ 
      }
      break;
    case 3://数据类型
      if(PacktData->DataType == DATATYPECMD)
      {
        //            HalLcdWriteStringValue( "status:", status, 10, HAL_LCD_LINE_2 );
        status++;
      }
      else{ 
        return;
      }
      break;
    case 4://网络类型
      if(PacktData->NwkType == NWKTYPE)
      {
        //           HalLcdWriteStringValue( "status:", status, 10, HAL_LCD_LINE_2 );
        status++;
      }
      else{ 
        return;
      }
      break;
    case 5://设备地址
      //          if((HI_UINT16(PacktData->ShortAddr) == LO_UINT16(EndShortAddr)) && 
      //             (LO_UINT16(PacktData->ShortAddr) == HI_UINT16(EndShortAddr)) )
      RxDeviceAddr = PacktData->ShortAddr[0];
      RxDeviceAddr |= PacktData->ShortAddr[1] << 8;
      if(RxDeviceAddr == EndShortAddr)
      {
        //            HalLcdWriteStringValue( "status:", status, 10, HAL_LCD_LINE_2 );
        status++;
      }
      else{
        return;
      }
      break;
    case 6://传感器类型
      if(PacktData->DeviceType == g_CoreType)
      {
        //          HalLcdWriteStringValue( "status:", status, 10, HAL_LCD_LINE_2 );
        status++;
      }
      else{ 
      }
      break;
    case 7://数据
       if(PacktData->DeviceType == LAMP)
            { 
              rdat = PacktData->Data[0];
              gdat = PacktData->Data[1];
              bdat = PacktData->Data[2];
              P9813_OnOff(rdat,gdat,bdat);
            }
       else if(PacktData->DeviceType == RELAY)
            {           
             SwitchOnOff(PacktData->Data[0]);       
            }
       else if(PacktData->DeviceType == FAN)
            {           
             Switch_FAN(PacktData->Data[0]);       
            } 
       else if(PacktData->DeviceType == BEEP)
            {   
             Switch_BEEP(PacktData->Data[0]); 
            }
            break;
      
    default:
      break;
    }
  }
}      
/***********************************************************************************************************
* 函数名称: Add_DataMsgBuff(void)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: --
*
* 作    者: 2017/2/13, by SongLei
* 邮    箱: 1170152143@qq.com
************************************************************************************************************/
void Add_DataMsgBuff(void)
{
  
  switch(g_CoreType)
  {
  case FAN :                    
     theMessageData[OFFSET]   =  SensorStatusflag;
    break;  
  case  LIGHT :
     theMessageData[OFFSET]   =   AdcData_H;
     theMessageData[OFFSET+1] =   AdcData_L ;
    break; 
  case  TEMPHUM :
     theMessageData[OFFSET]   =   ucharT_data_H ;
     theMessageData[OFFSET+1] =   ucharRH_data_H;
    break; 
  case  MPU9250:
     theMessageData[OFFSET ]    = ACC_X >> 8;        //加速度
     theMessageData[OFFSET + 1] = ACC_X;
     theMessageData[OFFSET + 2] = ACC_Y >> 8;
     theMessageData[OFFSET + 3] = ACC_Y;
     theMessageData[OFFSET + 4] = ACC_Z >> 8;
     theMessageData[OFFSET + 5] = ACC_Z;
     
     theMessageData[OFFSET + 6] = GYRO_X >> 8;  //陀螺仪
     theMessageData[OFFSET + 7] = GYRO_X;
     theMessageData[OFFSET + 8] = GYRO_Y >> 8;
     theMessageData[OFFSET + 9] = GYRO_Y;
     theMessageData[OFFSET + 10] = GYRO_Z >> 8;
     theMessageData[OFFSET + 11] = GYRO_Z;
                                                  //磁力
     theMessageData[OFFSET + 12] = MAG_X >> 8;
     theMessageData[OFFSET + 13] = MAG_X;
     theMessageData[OFFSET + 14] = MAG_Y >> 8;
     theMessageData[OFFSET + 15] = MAG_Y;
     theMessageData[OFFSET + 16] = MAG_Z >> 8;
     theMessageData[OFFSET + 17] = MAG_Z;
    break;
    
    case MPU6050 :  
     theMessageData[OFFSET ] =    mpu6050_ACC_X >> 8;        //加速度
     theMessageData[OFFSET + 1] = mpu6050_ACC_X;
     theMessageData[OFFSET + 2] = mpu6050_ACC_Y >> 8;
     theMessageData[OFFSET + 3] = mpu6050_ACC_Y;
     theMessageData[OFFSET + 4] = mpu6050_ACC_Z >> 8;
     theMessageData[OFFSET + 5] = mpu6050_ACC_Z;
    break;
    case BMP180 :
     theMessageData[OFFSET ] = Bmp180Temp;        //温度
     theMessageData[OFFSET + 1] = Bmp180AirPressure >> 8;//气压
     theMessageData[OFFSET + 2] = Bmp180AirPressure;
     theMessageData[OFFSET + 3] = Bmp180Altitude >> 8;//高度
     theMessageData[OFFSET + 4] = Bmp180Altitude;
      break;
    
  case GAS :
     theMessageData[OFFSET]   =   AdcData_H;
     theMessageData[OFFSET+1] =   AdcData_L ;    
    break; 
  case FLAME :
     theMessageData[OFFSET]   =   AdcData_H;
     theMessageData[OFFSET+1] =   AdcData_L ;
    break; 
  case PHOTOACTOR :
     theMessageData[OFFSET]   =  SensorStatusflag;  
    break; 
  case LAMP :
    theMessageData[OFFSET]   = rdat;
    theMessageData[OFFSET+1] = gdat;
    theMessageData[OFFSET+2] = bdat;  
    break; 
    
  case  RELAY:
     theMessageData[OFFSET]   =  SensorStatusflag;     
    break; 
  case  BEEP:
     theMessageData[OFFSET]   =  SensorStatusflag;
    break; 
  case  POTENTIOMETER:
     theMessageData[OFFSET]   =   AdcData_H;
     theMessageData[OFFSET+1] =   AdcData_L;
    break; 
  case  RAINFALL:
     theMessageData[OFFSET]   =    AdcData_H;
     theMessageData[OFFSET+1] =    AdcData_L ;    
    break; 
    
  case  INFRARED:
     theMessageData[OFFSET]   =  SensorStatusflag;     
    break; 
  case  ULTRASONIC:
     theMessageData[OFFSET]   =    (ultrasonicValue>>8);
     theMessageData[OFFSET+1] =    ultrasonicValue ;       
    break; 
  case  TOUCH:
     theMessageData[OFFSET]   =  SensorStatusflag;   
    break; 
  case  SOUND :         
     theMessageData[OFFSET]   =  SensorStatusflag;
    break; 
  case  FOG:
     theMessageData[OFFSET]   =   AdcData_H;
     theMessageData[OFFSET+1] =   AdcData_L;
    break; 
      
  default:
    break;
  }
}


/***********************************************************************************************************
* 函数名称: EndSendAnalyzeProtocol(uint8 *packet)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: 发送采集的数据函数/协调器发送数据到节点
*
* 作    者: 2017/2/10, by SongLei
* 邮    箱: 1170152143@qq.com
************************************************************************************************************/
void  EndSendAnalyzeProtocol(void)
{  

//#if  (NWKTYPE == 'Z')  
////  节点网络地址
//  osal_memcpy(&theMessageData[OFFSET -4],(uint8*)&EndShortAddr,2);
//#else
//  memcpy(&theMessageData[OFFSET -4],(uint8*)&EndShortAddr,2);
//#endif
  memcpy(&theMessageData[OFFSET -4],(uint8*)&EndShortAddr,2);
//设备类型
  theMessageData[OFFSET - 2] = g_CoreType;
//下面填充数据域部分  
//添加ADC的数据 
   Add_DataMsgBuff();
//电量检测
  theMessageData[OFFSET -1] = Get_volatile();
  //校验CRC8
  theMessageData[OFFSET + g_SendDataLength] = crc8(theMessageData,OFFSET + g_SendDataLength);
 
  //rcm add
  //printf("send_data:%d\n",theMessageData[OFFSET]<<8 |theMessageData[OFFSET+1] );//old
  printf("st%dh%d\n",theMessageData[OFFSET],theMessageData[OFFSET+1] );//old
   if ( HAL_UART_Transmit(&huart2, theMessageData, OFFSET + g_SendDataLength + 1,0xFFFF) == HAL_OK )
  {
      // Successfully requested to be sent.
      NetWorkLedstatusOnOff(NetWorkState_LED3, NetWorkState_LED3_Flag);//LED灯闪烁进行提醒(发送)
      NetWorkState_LED3_Flag = (~NetWorkState_LED3_Flag);
  }
  else
  {
    // Error occurred in request to send.
  }
}

