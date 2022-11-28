#include "display.h"
#include "lcd.h"
#include "fs_protocol.h"
#include "wificonfig.h"
#include "gpio.h"
#include "adc.h"
#include "ultrasonic.h"
#include "dht11.h"
#include "mpu9250.h"
#include "p9813.h"
#include "mpu6050.h"
#include "bmp180.h"

extern uint16_t  EndShortAddr;

/***********************************************************************************************************
* 函数名称: DisplayDeviceLogo(void)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: 显示设备类型
*
* 作    者: 2017/2/10, by SongLei
* 邮    箱: 1170152143@qq.com
************************************************************************************************************/
void DisplayDeviceLogo(void)
{
//  uint8_t buf[6] = {0};
//  uint8_t i=0;
//向led输出相关信息
  Lcd_Clear(YELLOW) ;
  Gui_DrawFont_GBK16(0,0,RED,CYAN,"  Wifi topology ");
  Gui_DrawFont_GBK16(0,16,RED,CYAN,"    EndDevice   ");
  Gui_DrawFont_GBK16(0,32,BLACK,YELLOW,"SSID:");
  Gui_DrawFont_GBK16(94,32,BLACK,YELLOW,SSID);
 // Gui_DrawFont_GBK16(0,48,BLACK,YELLOW,"IP:");
 //Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,StationIP);
  
//显示RSSI值
//  Gui_DrawFont_GBK16(0,112,BLACK,YELLOW,"RSSI:-");

  
 switch(g_CoreType)
  {
      case FAN :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "FanSensor:");
        break;  
      case  LIGHT :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "LightSensor:");
        break; 
      case  TEMPHUM :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "DTH11:");
        break; 
      case  MPU9250 :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "ACC_X:");
          Gui_DrawFont_GBK16(0,80,BLACK,YELLOW, "ACC_Y:");
          Gui_DrawFont_GBK16(0,96,BLACK,YELLOW, "ACC_Z:");
        break;
        
      case GAS :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "GasSensor:");
        break; 
      case FLAME :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "FlameSensor:");
        break; 
      case PHOTOACTOR :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "Photoactor:");
        break; 
      case LAMP :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "RED:");
          Gui_DrawFont_GBK16(0,80,BLACK,YELLOW, "GREEN:");
          Gui_DrawFont_GBK16(0,96,BLACK,YELLOW, "BLUE:");
        break; 
        
      case  RELAY :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "RelaySensor:");
        break; 
      case  BEEP :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "BeepSensor:");
        break; 
      case  POTENTIOMETER :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "Potential:");
        break; 
      case  RAINFALL :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "RainDrop: ");  
        break; 
        
      case  INFRARED :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "Infrared:");
        break; 
      case  ULTRASONIC :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "Ultrasonic:");
        break; 
      case  TOUCH :
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "TouchSensor:");
        break; 
      case  SOUND :         
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "Sound :");
        break; 
      case  FOG :         
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "Fog :");
        break; 
      case  MPU6050 :
      Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "3_ACC_X:");
      Gui_DrawFont_GBK16(0,80,BLACK,YELLOW, "3_ACC_Y:");
      Gui_DrawFont_GBK16(0,96,BLACK,YELLOW, "3_ACC_Z:");
    break;
      case BMP180 :        
      Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "Bmp180Temp:");
      Gui_DrawFont_GBK16(0,80,BLACK,YELLOW, "Pressure :");
      Gui_DrawFont_GBK16(0,96,BLACK,YELLOW, "Altitude:");
    break;
       case  Err :         
          Gui_DrawFont_GBK16(0,64,BLACK,YELLOW, "   No Sensor  ");
      break;        
      default:
        break;
      
  }   
  

  
}


uint8 Display_AccBuf[3][6] = {0};
int16_t mpu6050_ACC_X = 0,mpu6050_ACC_Y = 0,mpu6050_ACC_Z = 0;
/***********************************************************************************************************
* 函数名称: DisplayData(void)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: LCD显示数据和RSSI值
*
* 作    者: 2017/2/10, by SongLei
* 邮    箱: 1170152143@qq.com
************************************************************************************************************/
void DisplayData(void)
{
  uint8 Display_Buf[6] = {0};
  switch(g_CoreType)
  {  
    case FAN :
          if(SensorStatusflag == '1')
              Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "ON ");
          else if(SensorStatusflag == '0')
              Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "OFF");
      break;  
    case  LIGHT :
          num_to_str(AdSensor(), Display_Buf, 4 , 10 );
          Gui_DrawFont_GBK16(94,64,BLACK,YELLOW, Display_Buf);
      break; 
    case  TEMPHUM :
          DHT11_TEST();
          num_to_str(ucharT_data_H, Display_Buf, 4 , 10 );      
          Gui_DrawFont_GBK16(94,64,BLACK,YELLOW, Display_Buf);
          num_to_str(ucharRH_data_H, Display_Buf, 4 , 10 );      
          Gui_DrawFont_GBK16(94,80,BLACK,YELLOW, Display_Buf);
      break; 
    case  MPU9250 :
          Read_Mpu9250();
          num_to_str(ACC_X, Display_Buf, 4 , 10 );//转化为十进制
          Gui_DrawFont_GBK16(94,64,BLACK,YELLOW, Display_Buf);   

          num_to_str(ACC_Y, Display_Buf, 4 , 10 );//转化为十进制
          Gui_DrawFont_GBK16(94,80,BLACK,YELLOW, Display_Buf); 

          num_to_str(ACC_Z, Display_Buf, 4 , 10 );//转化为十进制
          Gui_DrawFont_GBK16(94,96,BLACK,YELLOW, Display_Buf);
          
      break;
     case  MPU6050 :
          mpu6050_Test(&mpu6050_ACC_X, &mpu6050_ACC_Y, &mpu6050_ACC_Z);
          //num_to_str(ACC_X, Display_Buf, 4 , 10 );//转化为十进制
          Gui_DrawFont_GBK16(78,64,YELLOW,YELLOW,Display_AccBuf[0]);
        //  if(ACC_X>)
          sprintf((char*)Display_AccBuf[0],"%d",mpu6050_ACC_X);
          Gui_DrawFont_GBK16(78,64,BLACK,YELLOW, Display_AccBuf[0]);   

//          num_to_str(ACC_Y, Display_Buf, 4 , 10 );//转化为十进制
          Gui_DrawFont_GBK16(78,80,YELLOW,YELLOW,Display_AccBuf[1]);
          sprintf((char*)Display_AccBuf[1],"%d",mpu6050_ACC_Y);
          Gui_DrawFont_GBK16(78,80,BLACK,YELLOW, Display_AccBuf[1]); 

//          num_to_str(ACC_Z, Display_Buf, 4 , 10 );//转化为十进制
          Gui_DrawFont_GBK16(78,96,YELLOW,YELLOW,Display_AccBuf[2]);
          sprintf((char*)Display_AccBuf[2],"%d",mpu6050_ACC_Z);
          Gui_DrawFont_GBK16(78,96,BLACK,YELLOW, Display_AccBuf[2]);
          
      break;
       case BMP180 :
          Read_Pressure();
          num_to_str(Bmp180Temp, Display_Buf, 3 , 10 );      
          Gui_DrawFont_GBK16(94,64,BLACK,YELLOW, Display_Buf);
          num_to_str(Bmp180AirPressure, Display_Buf, 4 , 10 );      
          Gui_DrawFont_GBK16(94,80,BLACK,YELLOW, Display_Buf);
          num_to_str(Bmp180Altitude, Display_Buf, 4 , 10 );      
          Gui_DrawFont_GBK16(94,96,BLACK,YELLOW, Display_Buf);          
      break; 

      
    case GAS :
          num_to_str(AdSensor(), Display_Buf, 4 , 10 );
          Gui_DrawFont_GBK16(94,64,BLACK,YELLOW, Display_Buf);      
            break; 
    case FLAME :
          num_to_str(AdSensor(), Display_Buf, 4 , 10 );
          Gui_DrawFont_GBK16(94,64,BLACK,YELLOW, Display_Buf);
      break; 
    case PHOTOACTOR :
         if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)== 1)
         {   
             SensorStatusflag = '1';
             Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "ON ");
         }
         else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)== 0)
         {
             SensorStatusflag = '0';              
             Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "OFF");
         }
      break; 
    case LAMP :
          P9813_OnOff(rdat,gdat,bdat);
          num_to_str(rdat, Display_Buf, 3 , 10 );      
          Gui_DrawFont_GBK16(94,64,BLACK,YELLOW, Display_Buf);
          num_to_str(gdat, Display_Buf, 3 , 10 );      
          Gui_DrawFont_GBK16(94,80,BLACK,YELLOW, Display_Buf);
          num_to_str(bdat, Display_Buf, 3 , 10 );      
          Gui_DrawFont_GBK16(94,96,BLACK,YELLOW, Display_Buf);          
      break; 
      
    case  RELAY :
          if(SensorStatusflag == '1')
              Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "ON ");
          else if(SensorStatusflag == '0')
              Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "OFF");
      break; 
    case  BEEP :
          if(SensorStatusflag == '1')
              Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "ON ");
          else if(SensorStatusflag == '0')
              Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "OFF");
      break; 
    case  POTENTIOMETER : //电位器
          num_to_str(AdSensor(), Display_Buf, 4 , 10 );
          Gui_DrawFont_GBK16(94,64,BLACK,YELLOW, Display_Buf);
      break; 
    case  RAINFALL :
          num_to_str(AdSensor(), Display_Buf, 4 , 10 );
          Gui_DrawFont_GBK16(94,64,BLACK,YELLOW, Display_Buf);
      break; 
     
    case  INFRARED :
         if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)== 1)
         {   
             SensorStatusflag = '1';
             Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "ON ");
         }
         else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)== 0)
         {
             SensorStatusflag = '0';              
             Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "OFF");
         }
      break; 
    case  ULTRASONIC :
          ultrasonicUsing();
          num_to_str(ultrasonicValue, Display_Buf, 4 , 10 );      
          Gui_DrawFont_GBK16(94,64,BLACK,YELLOW, Display_Buf);          
          
      break; 
    case  TOUCH :
         if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)== 1)
         {   
             SensorStatusflag = '1';
             Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "ON ");
         }
         else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)== 0)
         {
             SensorStatusflag = '0';              
             Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "OFF");
         }
      break; 
    case  SOUND :         
         if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)== 1)
         {   
             SensorStatusflag = '1';
             Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "ON ");
         }
         else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)== 0)
         {
             SensorStatusflag = '0';              
             Gui_DrawFont_GBK16(102,64,BLACK,YELLOW, "OFF");
         }
      break; 
      
      case  FOG :         
          num_to_str(AdSensor(), Display_Buf, 4 , 10 );
          Gui_DrawFont_GBK16(94,64,BLACK,YELLOW, Display_Buf);
      break; 

    //下面添加显示采集的数据或状态值
    
  default:
    break;
  }
  
    //显示接收到的信号强度值RSSI    
//    uint8 Rssi[2]={0};
//    num_to_str(DeviceRssi, Rssi, 2,10);
//    Gui_DrawFont_GBK16(48,112,BLACK,YELLOW,Rssi);
//    osal_memset(Rssi,0,osal_strlen(Rssi));
//    DeviceRssi = 0; 
}

