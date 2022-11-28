/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "dht11.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "bsp.h"
#include "lcd.h"
#include "wificonfig.h"
#include "fs_protocol.h"
#include "tim.h"
#include "display.h"   

#define H7 262         
#define H6 286
#define H5 311
#define H4 349
#define H3 392
#define H2 440
#define H1 494
#define Z7 523
#define Z6 587
#define Z5 659
#define Z4 698
#define Z3 784
#define Z2 880
#define Z1 987
#define L7 1046
#define L6 1174
#define L5 1318
#define L4 1396
#define L3 1567
#define L2 1760
#define L1 1975

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define APPLICATION_ADDRESS     (uint32_t)0x08003000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;

#if   (defined ( __CC_ARM ))
  __IO uint32_t VectorTable[48] __attribute__((at(0x20000000)));
#elif (defined (__ICCARM__))
#pragma location = 0x20000000
  __no_init __IO uint32_t VectorTable[48];
#elif defined   (  __GNUC__  )
  __IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
#elif defined ( __TASKING__ )
  __IO uint32_t VectorTable[48] __at(0x20000000);
#endif
/* USER CODE END 0 */
  
void delay(int n)
{
    int i,j;
    for(i = 0; i < n; i++)  
      for(j = 0; j < 10000; j++);   
}
void myDelay(int n)
{
  int i, j;
  for(i = 0; i < n; i++)
  {
    for(j = 0; j < 5; j++)
    {
      ;
    }
  }
}
void beep(int freq, int ms)     //
{
  int i;
  for(i = 0; i < ms * 1000 / freq / 2; i++)
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, GPIO_PIN_RESET);
    myDelay(freq);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, GPIO_PIN_SET);
    myDelay(freq);	   
  }
}

void systemInit()
{
 /* USER CODE BEGIN 1 */
     uint32_t i = 0;

  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */ 

/* Relocate by software the vector table to the internal SRAM at 0x20000000 ***/  

  /* Copy the vector table from the Flash (mapped at the base of the application
     load address 0x08003000) to the base address of the SRAM at 0x20000000. */
  for(i = 0; i < 48; i++)
  {
    VectorTable[i] = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
  }

  /* Enable the SYSCFG peripheral clock*/
  //RCC_APB2PeriphResetCmd(RCC_APB2Periph_SYSCFG, ENABLE);
   // RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);    
	__HAL_RCC_SYSCFG_CLK_ENABLE();
  /* Remap SRAM at 0x00000000 */
   // SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
   __HAL_SYSCFG_REMAPMEMORY_SRAM();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();//初始化串口2

  /* USER CODE BEGIN 2 */
  Lcd_Init();
  
#if 1
  //使能串口2/1 空闲中断
  UsartType1.dmaSend_flag =0;
  UsartType2.dmaSend_flag =0;

  Bsp_Init(); 
  DisplayDeviceLogo();  //显示设备logo
  
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE); 
  HAL_UART_Receive_DMA(&huart2,a_Usart2_RxBuffer,RXLENGHT); 
  
  Wifi_Config();//old
  //Wifi_Read_IP();//new
  Gui_DrawFont_GBK16(0,48,BLACK,YELLOW,StationIP);
  MX_TIM3_Init();

#endif
  /* USER CODE END 2 */
}
void send();
void receive();
void ledBlueSwitch(int sw);
void ledYellowSwitch(int sw);
void ledGreenSwitch(int sw);
void fanSwitch(int sw);


void ledInit()
{
 //1.配置AHB外部时钟使能寄存器（RCC_AHBENR）
  //定义一个指针变量rcc_ahbenr,保存寄存器地址
  uint32_t* rcc_ahbenr=(uint32_t*)0x40021014;
//  *rcc_ahbenr = *rcc_abhenr |(1<<18);
  *rcc_ahbenr |= 1<<18;
  //2.配置GPIO 端口模式寄存器（GPIO/mode）
  uint32_t* gpiob_moder = (uint32_t*)0x48000400;
  
  
  *gpiob_moder |=1; 
  *gpiob_moder |=1<<2; //让PB1引脚处于输出模式蓝  
  *gpiob_moder |=1<<4; //让PB2引脚处于输出模式黄
  *gpiob_moder |=1<<14; //让PB7引脚处于输出模式
  
  
  ledBlueSwitch(0);
  ledYellowSwitch(0);
  ledGreenSwitch(0);
  fanSwitch(0);
}
//控制蓝LED亮灭的函数参数sw,如果传递是1，表达开蓝灯，传递是0，表达关蓝灯
void ledBlueSwitch(int sw)
{
  //3.GPIO端口输出数据寄存器（GPIOB_ODR）
  uint32_t* gpio_odr = (uint32_t*)0x48000414;
  if(sw)//if(sw==1)表达开LED蓝，1就是真
    *gpio_odr &= ~(1<<1);//让 PB1引脚一个低电平LED蓝亮
  else
    *gpio_odr |= (1<<1);//让 PB1引脚一个高电平LED蓝灭
}

void ledYellowSwitch(int sw)
{
  //3.GPIO端口输出数据寄存器（GPIOB_ODR）
  uint32_t* gpio_odr = (uint32_t*)0x48000414;
  if(sw)//if(sw==1)表达开LED蓝，1就是真
    *gpio_odr &= ~(1<<2);//让 PB2引脚一个低电平LED黄亮
  else
    *gpio_odr |= (1<<2);//让 PB2引脚一个高电平LED黄灭
}

void ledGreenSwitch(int sw)
{
  //3.GPIO端口输出数据寄存器（GPIOB_ODR）
  uint32_t* gpio_odr = (uint32_t*)0x48000414;
  if(sw)//if(sw==1)表达开LED蓝，1就是真
    *gpio_odr &= ~(1);//让 PB0引脚一个低电平LED绿亮
  else
    *gpio_odr |= (1);//让 PB0引脚一个高电平LED绿灭
}

//风扇 所接引脚PB7，低电平开，高电平关
void fanSwitch(int sw)
{
  //3.GPIO端口输出数据寄存器（GPIOB_ODR）
  uint32_t* gpio_odr = (uint32_t*)0x48000414;
  if(sw)//if(sw==1)
    *gpio_odr &= ~(1<<7);//让 PB2引脚一个低电平
  else
    *gpio_odr |= (1<<7);//让 PB2引脚一个高电平
}









int main(void)
{
  systemInit();
  ledInit();
  
  while(1)
  {
    
  
//    char buf[100] = { 0 };//用来保存格式化的字符串
    char buf[100] = "SunShine Welcome";//buf数组中存储“hello”字符串
     DHT11_READ();//采集温湿度
    sprintf(buf, "ht,%d.%d,%d.%d  ", ucharT_data_H, ucharT_data_L,ucharRH_data_H,ucharRH_data_L);//格式化字符串
//    Gui_DrawFont_GBK16(10,106,BLACK,YELLOW,buf);//将buf中的“temp:%d.%d”打印在屏幕上
    
//    AdSensor();//采集关照强度
//     sprintf(buf, "light,%d.%d  ", AdcData_H,AdcData_L);//格式化字符串
     
     
     HAL_UART_Transmit(&huart2, buf, strlen(buf),0xFFFF);//将字符串发送到QT界面   
    Gui_DrawFont_GBK16(0, 90, BLACK, YELLOW, buf);//将发送的内容，显示在LCD屏幕上
    Delay_ms(200);
      
    //接收数据
	if(UsartType2.receive_flag == 1)
	{  
		
                if(strncmp(UsartType2.usartDMA_rxBuf,"LEDB",4)==0)//
                          {
                          
                          if(UsartType2.usartDMA_rxBuf[4]=='1')
                          {
                            Gui_DrawFont_GBK16(60,106,BLACK,BLUE,"on");
                            ledBlueSwitch(1);
                          }
                          else if (UsartType2.usartDMA_rxBuf[4]=='0')
                          {
                            Gui_DrawFont_GBK16(10,106,BLACK,BLUE,"off");
                            ledBlueSwitch(0);
                          }
                        
                          }
                        
                if(strncmp(UsartType2.usartDMA_rxBuf,"LEDG",4)==0)//
                        {
                         
                          if(UsartType2.usartDMA_rxBuf[4]=='1')
                          {
                            Gui_DrawFont_GBK16(10,106,BLACK,GREEN,"on");
                            ledGreenSwitch(1);
                           
                            
                          }
                          else if (UsartType2.usartDMA_rxBuf[4]=='0')
                          {
                            Gui_DrawFont_GBK16(10,106,BLACK,GREEN,"off");
                            ledGreenSwitch(0);
                        }
                        }
                        
                        
                 if(strncmp(UsartType2.usartDMA_rxBuf,"LEDY",4)==0)//
                        {
                         
                          if(UsartType2.usartDMA_rxBuf[4]=='1')
                          {
                            Gui_DrawFont_GBK16(10,106,BLACK,YELLOW,"on");
                           ledYellowSwitch(1);
                            
                          }
                          else if (UsartType2.usartDMA_rxBuf[4]=='0')
                          {
                            Gui_DrawFont_GBK16(10,106,BLACK,YELLOW,"off");
                            
                             ledYellowSwitch(0);
                          }
                        }
                        
                        
                        
                 if(strncmp(UsartType2.usartDMA_rxBuf,"FAN",3)==0)//
                        {
                         
                          if(UsartType2.usartDMA_rxBuf[3]=='1')
                          {
                            Gui_DrawFont_GBK16(10,106,BLACK,GREEN,"Fan on");
                            fanSwitch(1);
                            
                          }
                          else if (UsartType2.usartDMA_rxBuf[3]=='0')
                          {
                            Gui_DrawFont_GBK16(10,106,BLACK,GREEN,"Fan off");
                            fanSwitch(0);
                          }
                        }
                       
                      
          
          
		Gui_DrawFont_GBK16(10,106,BLACK,YELLOW,UsartType2.usartDMA_rxBuf);//将收到的字符串显示在屏幕上
		memset(UsartType2.usartDMA_rxBuf,0,sizeof(UsartType2.usartDMA_rxBuf));//清空数组
        UsartType2.receive_flag == 0;
		Delay_ms(200);
	}
  }
  /* USER CODE END 3 */

}

void send()
{
  char buf[20];
 
//  switch(g_CoreType)
//  {
//  case LIGHT:
    AdSensor();
    sprintf(buf, "light,%d.%d", AdcData_H,AdcData_L);
//      break;
//  case TEMPHUM:
//     DHT11_READ();
//      sprintf(buf, "ht,%d.%d,%d.%d", ucharT_data_H, ucharT_data_L, ucharRH_data_H, ucharRH_data_L);
//      break;
//  default:
//    return;
//  }
  HAL_UART_Transmit(&huart2, buf, strlen(buf), 0xffff);
  Gui_DrawFont_GBK16(0, 90, BLACK, YELLOW, buf);
}



void receive()
{
  
   //春节序曲
  uint16_t v3[]={Z3,Z2,Z3,Z5,Z5,Z6,H1,Z6,H3,H1,H3,Z7,Z6,Z5,Z3,Z5,Z6,Z5,Z6,Z5,Z5,Z6,H1,Z6,H1,H2,H1,Z6,Z5,Z3,Z5,Z6,H1,H2,H1,Z7,Z6,Z5,Z6,Z5,
                                                          L3,L2,L3,L5,L5,L6,Z1,L6,Z3,Z1,Z3,L7,L5,L5,L3,L5,L6,L5,L6,L5,L5,L6,Z1,L6,Z1,Z2,Z1,L6,L5,L3,L5,L6,Z1,Z2,Z1,L7,L6,L5,L6,Z1,L5,0};
  uint8_t t3[]={2,4,2,2,4,2,4,2,2,8,2,4,2,2,4,2,3,1,4,8,3,1,4,2,4,2,4,2,2,8,3,1,4,2,4,2,3,1,4,8,
								2,4,2,2,4,2,4,2,2,8,2,4,2,2,4,2,3,1,4,8,3,1,4,2,4,2,4,2,2,8,3,1,4,2,4,2,3,1,2,2,8,16};
  if(UsartType2.receive_flag == 1)
  {
/*
    if(strcmp(UsartType2.usartDMA_rxBuf, "open") == 0)
    {
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);
    //  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 , GPIO_PIN_RESET);
  //   play_music(v3,t3,sizeof(t3)/sizeof(t3[0]));
      Gui_DrawFont_GBK16(0, 90, BLACK, YELLOW, "open ");
      memset(UsartType2.usartDMA_rxBuf, 0, UsartType2.Usart_rx_len);
    }
    else if(strcmp(UsartType2.usartDMA_rxBuf, "close") == 0)
    {
     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_SET);  
    //  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7 , GPIO_PIN_SET);     
      Gui_DrawFont_GBK16(0, 90, BLACK, YELLOW, "close");
      memset(UsartType2.usartDMA_rxBuf, 0, UsartType2.Usart_rx_len);
    }
*/
    //LEDB
    //LEDY
    //LEDG
    //LEDR
    //FAN
    
    if(strncmp(UsartType2.usartDMA_rxBuf, "LEDY", strlen("LEDY")) == 0)
    {
      if(UsartType2.usartDMA_rxBuf[4] == '1')
      {
       ledYellowSwitch(1);
       Gui_DrawFont_GBK16(0, 90, BLACK, YELLOW, "open ");
      }
      else if(UsartType2.usartDMA_rxBuf[4] == '0')
      {
       ledYellowSwitch(0);
       Gui_DrawFont_GBK16(0, 90, BLACK, YELLOW, "close ");
      } 
      memset(UsartType2.usartDMA_rxBuf, 0, UsartType2.Usart_rx_len);
       
    }
    else if(strncmp(UsartType2.usartDMA_rxBuf, "LEDB", strlen("LEDB")) == 0)
    {
      if(UsartType2.usartDMA_rxBuf[4] == '1')
      {
       ledBlueSwitch(1);
       Gui_DrawFont_GBK16(0, 90, BLACK, YELLOW, "open ");
      }
      else if(UsartType2.usartDMA_rxBuf[4] == '0')
      {
        ledBlueSwitch(0);
       Gui_DrawFont_GBK16(0, 90, BLACK, YELLOW, "close ");
      } 
      memset(UsartType2.usartDMA_rxBuf, 0, UsartType2.Usart_rx_len);
       
    }
     else if(strncmp(UsartType2.usartDMA_rxBuf, "LEDG", strlen("LEDG")) == 0)
    {
      if(UsartType2.usartDMA_rxBuf[4] == '1')
      {
       ledGreenSwitch(1);
       Gui_DrawFont_GBK16(0, 90, BLACK, YELLOW, "open ");
      }
      else if(UsartType2.usartDMA_rxBuf[4] == '0')
      {
       ledGreenSwitch(0);
       Gui_DrawFont_GBK16(0, 90, BLACK, YELLOW, "close ");
      } 
      memset(UsartType2.usartDMA_rxBuf, 0, UsartType2.Usart_rx_len);  
    }
     else if(strncmp(UsartType2.usartDMA_rxBuf, "FAN", strlen("FAN")) == 0)
    {
      if(UsartType2.usartDMA_rxBuf[3] == '1')
      {
        fanSwitch(1);
       Gui_DrawFont_GBK16(0, 90, BLACK, YELLOW, "open ");
      }
      else if(UsartType2.usartDMA_rxBuf[3] == '0')
      {
       fanSwitch(1);
       Gui_DrawFont_GBK16(0, 90, BLACK, YELLOW, "close ");
      } 
      memset(UsartType2.usartDMA_rxBuf, 0, UsartType2.Usart_rx_len);
       
    }
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{     
  while((USART1->ISR & 0X40)==0);
  USART1->TDR = (uint8_t) ch;      
  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
