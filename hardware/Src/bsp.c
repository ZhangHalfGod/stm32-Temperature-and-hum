#include "bsp.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"

#include "fs_protocol.h"
#include "switch.h"
#include "ultrasonic.h"
#include "mpu9250.h"
#include "mpu6050.h"
#include "bmp180.h"

GPIO_PinState NetWorkState_LED2_Flag = GPIO_PIN_RESET;
GPIO_PinState NetWorkState_LED3_Flag = GPIO_PIN_RESET;
GPIO_PinState NetWorkState_LED4_Flag = GPIO_PIN_RESET;

uint8_t  g_CoreType = 0;   //传感板的类型
uint16_t DEVICE_ADDR = 0x00; //传感器地址
uint16_t EndShortAddr ; 
uint32_t ADC_DMA_Value[ADC_NUM] = {0};    //AD采集的

GPIO_TypeDef* LEDS_GPIOx_PORT[LEDSCOMn]={GPIOB,GPIOB,GPIOB}; 
const uint32_t LEDS_GPIOx_Pin[LEDSCOMn]={GPIO_PIN_2,GPIO_PIN_1,GPIO_PIN_0};


/***********************************************************************************************************
* 函数名称: void Bsp_Init(void)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: 板级初始化，AD、GPIO
*
* 作    者: 2017/2/9, by SongLei
* 邮    箱: 1170152143@qq.com
************************************************************************************************************/
void Bsp_Init(void)
{
  //读核心板的类型
  g_CoreType= Read_CoreType();
  //初始化对应的传感器
    EndShortAddr = LO_UINT16(DEVICE_ADDR)<<8 | HI_UINT16(DEVICE_ADDR);
  Board_Init(g_CoreType);
}


/***********************************************************************************************************
* 函数名称: void Board_Init(tpye)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: --
*
* 作    者: 2017/2/10, by SongLei
* 邮    箱: 1170152143@qq.com
************************************************************************************************************/
void Board_Init(uint8 tpye)
{
  switch(tpye)
  {
      case FAN :                    
          D1_OUT_GPIO_Init(); 
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
        break;  
      case  LIGHT :        
        break; 
      case  TEMPHUM :
        break; 
      case  MPU9250:
         MX_I2C1_Init();
          Init_MPU9250(); 
        break;
        
      case GAS :   
        break; 
      case FLAME :
          D2_IN_GPIO_Init(); 
        break; 
      case PHOTOACTOR :
          D2_IN_GPIO_Init(); 
        break; 
      case LAMP :
          D1_OUT_GPIO_Init(); 
          D2_OUT_GPIO_Init(); 
        break; 
        
      case  RELAY:
          D1_OUT_GPIO_Init(); 
        break; 
      case  BEEP:
          MX_TIM17_Init();
        break; 
      case  POTENTIOMETER:
        break; 
      case  RAINFALL:        
        break; 
        
      case  INFRARED:
          D2_IN_GPIO_Init(); 
        break; 
      case  ULTRASONIC:
      ultrasonicUsing_Init()  ; 
        break; 
      case  TOUCH:
          D1_IN_GPIO_Init(); 
        break; 
      case  SOUND :         
          D2_IN_GPIO_Init();  
        break; 
      case FOG :   
        break;   
     case  MPU6050 :   
       MX_I2C1_Init();
       InitMPU6050();	//初始化MPU6050
    break; 
  case  BMP180 :  
    MX_I2C1_Init();
      Bmp180_Init();  
    break; 
  default:
    break;
  }
}
/***********************************************************************************************************
* 函数名称: void Read_CoreType(void)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: M0读取传感板D7引脚（核心板P0_1）的AD值，通过AD值进行判断核心板类型
*           WiFi   D7   PA5
* 作    者: 2017/2/10, by SongLei
* 邮    箱: 1170152143@qq.com
************************************************************************************************************/
uint8 Read_CoreType(void)
{
  uint32 adcCoreTypes = 0;
  uint8 DeviceType[2] = {0};
  //开始DMA转换ADC
  HAL_ADC_Start_DMA(&hadc,(uint32_t*)ADC_DMA_Value,ADC_NUM);
  
  HAL_Delay(500);
  
  adcCoreTypes = (uint16_t)ADC_DMA_Value[ADC_IN5];    //获得通道5的ADC值
  
  adcCoreTypes = (adcCoreTypes * 3300)/4096;         //换算ADC的值%d mV

#ifdef DEBUG_PRINTF   
  printf("CoreADC:%d mV\n",adcCoreTypes);
#endif
  
  if(ADCCOMP(adcCoreTypes,FAN_ADC-ADC_ERRORS,FAN_ADC+ADC_ERRORS))                //风扇 理论AD采集值 16
  {
    DeviceType[0]  =  FAN;
    DEVICE_ADDR =  FAN_ADDR;
    theMessageData[1] = g_SendDataLength = 1;           //数据域长度
  }
  else if(ADCCOMP(adcCoreTypes,LIGHT_ADC-ADC_ERRORS_L,LIGHT_ADC+ADC_ERRORS_L))          //光强 理论AD采集值 65
  {
    DeviceType[0] =  LIGHT;
    DEVICE_ADDR =  LIGHT_ADDR ;
    theMessageData[1] = g_SendDataLength = 2;           //数据域长度
  }  
  else if(ADCCOMP(adcCoreTypes,TEMPHUM_ADC-ADC_ERRORS,TEMPHUM_ADC+ADC_ERRORS))         //温湿度 理论AD采集值105
  {
    DeviceType[0] =  TEMPHUM;
    DEVICE_ADDR = TEMPHUM_ADDR;
    theMessageData[1] = g_SendDataLength = 2;           //数据域长度
  }
  else if(ADCCOMP(adcCoreTypes,MPU9250_ADC-ADC_ERRORS,MPU9250_ADC+ADC_ERRORS))        //九轴 理论AD采集值210
  {
    DeviceType[0] =  MPU9250 ;
    DEVICE_ADDR = MPU9250_ADDR ;
    theMessageData[1] = g_SendDataLength = 18;          //数据域长度
  }  
  else if(ADCCOMP(adcCoreTypes,GAS_ADC-ADC_ERRORS,GAS_ADC+ADC_ERRORS))        //可燃气体 理论AD采集值300
  {
    DeviceType[0] =  GAS;
    DEVICE_ADDR = GAS_ADDR ;
    theMessageData[1] = g_SendDataLength = 2;           //数据域长度
  }
  else if(ADCCOMP(adcCoreTypes,FLAME_ADC-ADC_ERRORS,FLAME_ADC+ADC_ERRORS))        //火焰  理论AD采集值 430
  {
    DeviceType[0] =  FLAME;
    DEVICE_ADDR = FLAME_ADDR ;
    theMessageData[1] = g_SendDataLength = 2;           //数据域长度
  }
  else if(ADCCOMP(adcCoreTypes,PHOTOACTOR_ADC-ADC_ERRORS,PHOTOACTOR_ADC+ADC_ERRORS))       //光电开关 理论AD采集值 550
  {
    DeviceType[0] =  PHOTOACTOR;
    DEVICE_ADDR = PHOTOACTOR_ADDR;
    theMessageData[1] = g_SendDataLength = 1;           //数据域长度
  }
  else if(ADCCOMP(adcCoreTypes,LAMP_ADC-ADC_ERRORS,LAMP_ADC+ADC_ERRORS))        //RGB三色灯 理论AD采集值627
  {
    DeviceType[0] = LAMP;
    DEVICE_ADDR = LAMP_ADDR;
    theMessageData[1] = g_SendDataLength = 3;           //数据域长度
  }
  else if(ADCCOMP(adcCoreTypes,RELAY_ADC-ADC_ERRORS,RELAY_ADC+ADC_ERRORS))       //继电器 理论AD采集值700
  {
    DeviceType[0] = RELAY ;
    DEVICE_ADDR = RELAY_ADDR;
    theMessageData[1] = g_SendDataLength = 1;           //数据域长度
  }
  else if(ADCCOMP(adcCoreTypes,BEEP_ADC-ADC_ERRORS,BEEP_ADC+ADC_ERRORS))      //蜂鸣器 理论AD采集值818
  {
    DeviceType[0] =  BEEP;
    DEVICE_ADDR = BEEP_ADDR;
    theMessageData[1] = g_SendDataLength = 1;            //数据域长度
  }
  else if(ADCCOMP(adcCoreTypes,POTENTIOMETER_ADC-ADC_ERRORS,POTENTIOMETER_ADC+ADC_ERRORS))       //电位器 理论AD采集值908
  {
    DeviceType[0] = POTENTIOMETER ;
    DEVICE_ADDR = POTENTIOMETER_ADDR;
    theMessageData[1] = g_SendDataLength = 2;           //数据域长度
  }
  else if(ADCCOMP(adcCoreTypes,RAINFALL_ADC-ADC_ERRORS,RAINFALL_ADC+ADC_ERRORS))       //降雨 理论AD采集值1055
  {
    DeviceType[0] =  RAINFALL ;
    DEVICE_ADDR = RAINFALL_ADDR ;
    theMessageData[1] = g_SendDataLength = 2;           //数据域长度
  }
  else if(ADCCOMP(adcCoreTypes,INFRARED_ADC-ADC_ERRORS,INFRARED_ADC+ADC_ERRORS))      //人体红外 理论AD采集值1185
  {
    DeviceType[0] =  INFRARED ;
    DEVICE_ADDR = INFRARED_ADDR ;
    theMessageData[1] = g_SendDataLength = 1;           //数据域长度
  }
  else if(ADCCOMP(adcCoreTypes,ULTRASONIC_ADC-ADC_ERRORS,ULTRASONIC_ADC+ADC_ERRORS))     //超声波  理论AD采集值1336
  {
    DeviceType[0] = ULTRASONIC ;
    DEVICE_ADDR = ULTRASONIC_ADDR ;
    theMessageData[1] = g_SendDataLength = 2;           //数据域长度
  }
  else if(ADCCOMP(adcCoreTypes,TOUCH_ADC-ADC_ERRORS,TOUCH_ADC+ADC_ERRORS))        //触摸按键 理论AD采集值1487
  {
    DeviceType[0] = TOUCH;
    DEVICE_ADDR = TOUCH_ADDR;
    theMessageData[1] = g_SendDataLength = 1;  
  }
  else if(ADCCOMP(adcCoreTypes,SOUND_ADC-ADC_ERRORS,SOUND_ADC+ADC_ERRORS))         //声控开关 理论AD采集值1650
  {
    DeviceType[0] = SOUND;
    DEVICE_ADDR = SOUND_ADDR ;
    theMessageData[1] = g_SendDataLength = 1;     
  }
  else if(ADCCOMP(adcCoreTypes,FOG_ADC-ADC_ERRORS,FOG_ADC+ADC_ERRORS))         //烟雾开关 理论AD采集值1650
  {
    DeviceType[0] = FOG ;
    DEVICE_ADDR = FOG_ADDR;
    theMessageData[1] = g_SendDataLength = 2;     
  }  
//  else if(ADCCOMP(adcCoreTypes,0,20))//心率 理论AD采集值1964
//  {
//    DeviceType[0] = HEARTRATE;
//    theMessageData[1] = g_SendDataLength = 1;     //添加
//  }
  else if(ADCCOMP(adcCoreTypes,MPU6050_ADC-ADC_ERRORS,MPU6050_ADC+ADC_ERRORS))//三轴 理论AD采集值
  {
    DeviceType[0] = MPU6050;
    DEVICE_ADDR = MPU6050_ADDR;
    theMessageData[1] = g_SendDataLength = 6;     //添加
  }
  else if(ADCCOMP(adcCoreTypes,BMP180_ADC-ADC_ERRORS,BMP180_ADC+ADC_ERRORS))//气压 理论AD采集值
  {
    DeviceType[0] = BMP180 ;
     DEVICE_ADDR = BMP180_ADDR;
    theMessageData[1] = g_SendDataLength = 5;     //添加
  }
    else         //烟雾开关 理论AD采集值1650
  {
    DeviceType[0] = Err ;
//    DEVICE_ADDR = Err_ADDR;

  }  
#ifdef    DEBUG_PRINTF 
  printf("DeviceType:%d\n",DeviceType[0]);
#endif
//  HAL_ADC_Stop_DMA(&hadc);
  return DeviceType[0];
}

/***********************************************************************************************************
* 函数名称: uint8 Get_volatile(void)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: ZigBee 电池采集 P0_0
*           WiFi   电池采集 PA0
* 作    者: 2017/2/10, by SongLei
* 邮    箱: 1170152143@qq.com
************************************************************************************************************/
uint8 Get_volatile(void)
{
  uint32 adcBt =0;
  uint8 percent = 0;
  
//  adcBt = HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12);
//  HAL_ADC_Start_DMA(&hadc,(uint32_t*)ADC_DMA_Value,ADC_NUM);//开始转换ADC
  adcBt = (uint16_t)ADC_DMA_Value[ADC_IN0];
  
  adcBt = (adcBt * 3300)/4096;

#ifdef DEBUG_PRINTF  
  //rcm add 
  //printf("adcBt:%d\n",adcBt);//old
#endif
  
  /*我们规定最低放电电压为3.51v ,经过分压AD检测到的应该是2.34v*/
  /*我们规定最低放电电压为4.20v ,经过分压AD检测到的应该是2.8v*/
  //计算百分比   满电的时候是2.8v  ,该充电的时候是2.34v, 460是满电2800与没电2340的差
  percent = ((adcBt - 2340)*100) / 460;
  
  return percent;
}

/***********************************************************************************************************
* 函数名称: NetWorkLedstatusOnOff(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: 显示网络状态（配置，发送，接收）
*
* 作    者: 2017/2/26, by SongLei
* 邮    箱: 1170152143@qq.com
************************************************************************************************************/
void NetWorkLedstatusOnOff(uint8_t leds, GPIO_PinState PinState)
{
    HAL_GPIO_WritePin(LEDS_GPIOx_PORT[leds], LEDS_GPIOx_Pin[leds], PinState); 
}







