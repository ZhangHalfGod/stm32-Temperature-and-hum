#include "switch.h"
#include "bsp.h"
#include "fs_protocol.h"
#include "tim.h"





/***********************************************************************************************************
* 函数名称: void SwitchOn(uint8 type)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能:状态控制开关的开/闭控制 
*
* 作    者: 
* 邮    箱: 
************************************************************************************************************/
void SwitchOnOff(uint8 type)
{
  switch(type)
  {
  case CMDON://开设备
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, GPIO_PIN_SET) ;
      SensorStatusflag = '1';
    break;
    
  case CMDOFF://关设备
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, GPIO_PIN_RESET) ;
      SensorStatusflag = '0' ;
    break;
    
  default:
    break;
  }
}

void Switch_FAN(uint8 type)
{
  switch(type)
  {
  case CMDON://开设备
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, GPIO_PIN_RESET) ;
      SensorStatusflag = '1' ;
      printf("fan on!\n");
      
    break;    
  case CMDOFF://关设备
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, GPIO_PIN_SET) ;
      SensorStatusflag = '0' ;
       printf("fan off!\n");
    break;
    
  default:
    break;
  }
}

void Switch_BEEP(uint8 type)
{
  switch(type)
  {
  case CMDON://开设备
      HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);
      SensorStatusflag = '1' ;
    break;    
  case CMDOFF://关设备
      HAL_TIMEx_PWMN_Stop(&htim17, TIM_CHANNEL_1);
      SensorStatusflag = '0' ;
    break;
    
  default:
    break;
  }
}
      

