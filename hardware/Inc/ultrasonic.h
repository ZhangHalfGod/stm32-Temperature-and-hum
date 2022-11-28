/**
  ******************************************************************************
  * File Name          : ultrasonic.h
  * Description        : 用于超声波测距模块的初始化和采集数据   。                 
  ******************************************************************************
  *

  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ultrasonic_H
#define ultrasonic_H
#include "stdint.h"

/* Includes ------------------------------------------------------------------*/

extern uint32_t ultrasonicValue;


void ultrasonicUsing(void);
void ultrasonicUsing_Init(void);   

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */



/* USER CODE END Private defines */


#endif
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
