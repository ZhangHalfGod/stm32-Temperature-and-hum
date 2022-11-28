#include "stdint.h"
#include "bsp.h"
#include "wificonfig.h"
#include "usart.h"
#include "string.h"
#include "stm32f0xx_hal.h"
//#include "fs_protocol.h"

#define TimeDelay  100

uint8_t NowWifiConfig = 2;  //是否真处于配置模式，Wifi配置和接收两种模式

/*****************************AT指令*******************************************/
static uint8_t *At_QuitCipsend = "+++";                //退出透传
static uint8_t *At_CipClose = "AT+CIPCLOSE\r\n";       //断开连接 
static uint8_t *At_CwqAp = "AT+CWQAP\r\n";		//断开AP连接
static uint8_t *At_CwMode = "AT+CWMODE?\r\n";	        //查询工作模式
static uint8_t *At_Restore= "AT+RESTORE\r\n";		//回复出厂设置
static uint8_t *At_CwMode3 = "AT+CWMODE=3\r\n";        //设置为AP兼STATION模式



static uint8_t *At_Cifsr = "AT+CIFSR\r\n";       //获取本地IP


uint8_t *SSID = "1002";     //设置WiFI的SSID

//uint8_t At_CwjAp[50] = {0};
//修改30行/////////////////////////////////////////////////////////////
uint8_t *At_CwjAp = "AT+CWJAP=\"sunshine\",\"sunshine123456\"\r\n"; //加入AP
//uint8_t *At_CwjAp = "AT+CWJAP=\"goodlife\",\"fl021126\"\r\n"; //加入AP
static uint8_t *At_CipMux = "AT+CIPMUX=0\r\n";	        //设置单链接	
//uint8_t *At_CipStart = "AT+CIPSTART=\"UDP\",\"192.168.0.15\",8080,20000,0\r\n";//建立UDP连接  


//static uint8_t *At_CipStart = "AT+CIPSTART=\"UDP\",\"192.168.0.15\",8080,20000,0\r\n";//建立UDP连接  //old:
//static uint8_t *At_CipStart = "AT+CIPSTART=\"UDP\",\"192.168.43.248\",8080,20000,0\r\n";//建立UDP连接  //new:
//修改38行///////////////////////////////////////////////////////////////////
//stm32是发送端  需要提前知道QT界面IP和端口号
//STM32 20000是自己绑定的端口号
//QT界面是接收端
//",QT端口号,STM32端口号,0"

static uint8_t *At_CipStart = "AT+CIPSTART=\"UDP\",\"192.168.43.180\",8080,20000,0\r\n";//建立UDP连接  //new:

static uint8_t *At_CipMode = "AT+CIPMODE=1\r\n";       //设置透传模式
static uint8_t *At_CipSend = "AT+CIPSEND\r\n";	        //开启透传

//AT指令对应的回复
static uint8_t * At_CwMode_Ack = "AT+CWMODE?\r\r\n+CWMODE:2\r\n\r\nOK\r\n";//配置为了AP模式  
//uint8_t At_CwjAp_Ack_V1[50] = {0};
//修改47行///////////////////////////////////////////////////////////////////
static uint8_t *At_CwjAp_Ack_V1 = "AT+CWJAP=\"sunshine\",\"sunshine123456\"\r\r\n";
//static uint8_t *At_CwjAp_Ack_V1 = "AT+CWJAP=\"goodlife\",\"fl021126\"\r\r\n";
static uint8_t *At_CwjAp_Ack_V2 = "WIFI CONNECTED\r\r\n";
static uint8_t *At_CwjAp_Ack_V3 = "WIFI GOT IP\r\r\n";
static uint8_t *At_CwjAp_Ack_V4 = "\r\nOK\r\n";
//static uint8_t *At_CwqAp_Ack = "AT+CWQAP\r\r\n\r\nOK\r\n";
static uint8_t *At_CipSend_Ack = "AT+CIPSEND\r\r\n\r\nOK\r\n\r\n>";	    //开启透传应答
static uint8_t *At_QuitCipsend_Ack = "+++";                //退出透传

uint8_t StationIP[14] = {0};


/***********************************************************************************************************
* 函数名称: void Reset_RxbuffClear(void)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: --
*
* 作    者: 2017/2/22, by SongLei
* 邮    箱: 1170152143@qq.com
************************************************************************************************************/
void Reset_RxbuffClear(void)
{
  UsartType2.receive_flag = 0;//清零标记
  
#if (DEBUG_PRINTF == TRUE) 
  printf("WiFi:%s\n",UsartType2.usartDMA_rxBuf);
#endif
  
  memset(UsartType2.usartDMA_rxBuf,0,strlen(UsartType2.usartDMA_rxBuf));
  
  NetWorkLedstatusOnOff(NetWorkState_LED4, NetWorkState_LED4_Flag);
  NetWorkState_LED4_Flag = (~NetWorkState_LED4_Flag);
}
/***********************************************************************************************************
* 函数名称: void Wifi_Config(void)
* 输入参数: 
*               
* 返回参数: 
*
* 功    能: 使用串口方式配置wifi，实现AT透传模式（UDP）
*
* 作    者: 2017/2/23, by SongLei
* 邮    箱: 1170152143@qq.com
************************************************************************************************************/
void Wifi_Config(void)
{
  uint8_t i=0;
  //因为启动USART DMA时会首次调用一次HAL_UART_RxCpltCallback();所以清0
  
  UsartType2.receive_flag = 0;
  
  //透传模式还没有开始，现在是在配置WiFi，并默认开启的有回显功能
  NowWifiConfig = 0;
    /****************** 恢复出厂设置 *************************/  
  if(Wifi_RESTORE() ==1 )
  {
    memset(UsartType2.usartDMA_rxBuf,0,1024);
    printf("恢復出廠設置 OK\n");
  }else printf("恢復出廠設置 Faile\n");
  
REST:
  HAL_Delay(100);  //等待WiFi复位完成（wifi复位时会打印一串无用信息）
  Reset_RxbuffClear();
  /******************start 退出透传*************************/
  HAL_UART_Transmit(&huart2, At_QuitCipsend, strlen((char*)At_QuitCipsend), 0xffff);
  while(UsartType2.receive_flag != 1);
  if (0 != strncmp((char*)UsartType2.usartDMA_rxBuf, (char*)At_QuitCipsend_Ack, strlen((char*)At_QuitCipsend_Ack))) 
  {
    Reset_RxbuffClear(); 
    goto REST;
  }
  else
  {
    Reset_RxbuffClear(); 
  }
  
  HAL_UART_Transmit(&huart2, At_QuitCipsend, strlen((char*)At_QuitCipsend), 0xffff);
  while(UsartType2.receive_flag != 1);
  Reset_RxbuffClear();
  HAL_UART_Transmit(&huart2, At_QuitCipsend, strlen((char*)At_QuitCipsend), 0xffff);
  while(UsartType2.receive_flag != 1);
  Reset_RxbuffClear();
  /****************** 断开上次连接 *************************/
  HAL_UART_Transmit(&huart2, At_CipClose, strlen((char*)At_CipClose), 0xffff);
  while(UsartType2.receive_flag != 1);
  Reset_RxbuffClear();
  HAL_UART_Transmit(&huart2, At_CipClose, strlen((char*)At_CipClose), 0xffff);
  while(UsartType2.receive_flag != 1);
  Reset_RxbuffClear();
  HAL_UART_Transmit(&huart2, At_CipClose, strlen((char*)At_CipClose), 0xffff);
  while(UsartType2.receive_flag != 1);
  Reset_RxbuffClear();
  
  /****************** 断开与AP连接 *************************/
  HAL_UART_Transmit(&huart2, At_CwqAp, strlen((char*)At_CwqAp), 0xffff);
  HAL_Delay(TimeDelay);
  while(UsartType2.receive_flag != 1);
  Reset_RxbuffClear();
  HAL_UART_Transmit(&huart2, At_CwqAp, strlen((char*)At_CwqAp), 0xffff);
  HAL_Delay(TimeDelay);
  while(UsartType2.receive_flag != 1);
  Reset_RxbuffClear();
  
err:
  
  /****************** 设置AP+ATATION模式 *******************/
  HAL_UART_Transmit(&huart2, At_CwMode3, strlen((char*)At_CwMode3), 0xffff);
  while(UsartType2.receive_flag != 1);
  HAL_Delay(TimeDelay);
  Reset_RxbuffClear();
  
  /****************** 查询工作模式 *************************/
  HAL_UART_Transmit(&huart2, At_CwMode, strlen((char*)At_CwMode), 0xffff);
  while(UsartType2.receive_flag != 1);
  //  UsartType2.receive_flag = 0;
  //  HAL_Delay(TimeDelay);
  if (0 == strncmp((char*)UsartType2.usartDMA_rxBuf, (char*)At_CwMode_Ack, strlen((char*)At_CwMode_Ack))) 
  {//配置为AP模式
    HAL_UART_Transmit(&huart2, At_Restore, strlen((char*)At_Restore), 0xffff);
    __HAL_UART_DISABLE_IT(&huart2,UART_FLAG_RXNE);
    HAL_Delay(4000);
    MX_USART1_UART_Init();
    printf("Restore to factory settings\n");
    __HAL_UART_ENABLE_IT(&huart2,UART_FLAG_RXNE);
    Reset_RxbuffClear(); 
    goto REST;
  }
  else
  {
    Reset_RxbuffClear(); 
  }
  Reset_RxbuffClear(); 
  
  //snprintf((char*)At_CwjAp ,50,"AT+CWJAP=\"farsight_%s\",\"1234567890123\"\r\n",SSID);//old:
  //snprintf((char*)At_CwjAp ,50,"AT+CWJAP=\"fssy7051\",\"fssy7051\"\r\n",SSID);//new
  /****************** 加入 AP ******************************/
  HAL_UART_Transmit(&huart2, At_CwjAp, strlen((char*)At_CwjAp), 0xffff);
  
  // snprintf((char*)At_CwjAp_Ack_V1 ,50,"AT+CWJAP=\"farsight_%s\",\"1234567890123\"\r\r\n",SSID);//old:
  //snprintf((char*)At_CwjAp_Ack_V1 ,50,"AT+CWJAP=\"fssy7051\",\"fssy7051\"\r\r\n");//new
  
    while(UsartType2.receive_flag != 1);
    while(!strstr(UsartType2.usartDMA_rxBuf,At_CwjAp_Ack_V1));
   {

//     HAL_Delay(50);
   }
  i=7;
  while(i--)  //循环7次
  {
    if (0 == strncmp((char*)UsartType2.usartDMA_rxBuf, (char*)At_CwjAp_Ack_V1, strlen((char*)At_CwjAp_Ack_V1))) 
    {
      //printf("connection AP Success\n");
      Reset_RxbuffClear();
      while(UsartType2.receive_flag != 1);
      if (0 == strncmp((char*)UsartType2.usartDMA_rxBuf, (char*)At_CwjAp_Ack_V2, strlen((char*)At_CwjAp_Ack_V2))) 
      {
        //printf("connection AP Success\n");
        Reset_RxbuffClear();
      }
      Reset_RxbuffClear();
      while(UsartType2.receive_flag != 1);
      if (0 == strncmp((char*)UsartType2.usartDMA_rxBuf, (char*)At_CwjAp_Ack_V3, strlen((char*)At_CwjAp_Ack_V3))) 
      {
        //printf("connection AP Success\n");
        Reset_RxbuffClear();
      }
      Reset_RxbuffClear();
      while(UsartType2.receive_flag != 1);
      if (0 == strncmp((char*)UsartType2.usartDMA_rxBuf, (char*)At_CwjAp_Ack_V4, strlen((char*)At_CwjAp_Ack_V4))) 
      {
        //printf("connection AP Success\n");
        Reset_RxbuffClear();
      }
      break;
    }
    else
    {
      Reset_RxbuffClear();
      printf("connection AP Fail %d\n",i);
      HAL_Delay(1000);
      if(i == 1)
        goto err;
    }
  }
  /****************** 设置单链接模式 *******************/
  HAL_UART_Transmit(&huart2, At_CipMux, strlen((char*)At_CipMux), 0xffff); 
  HAL_Delay(TimeDelay);
  Reset_RxbuffClear();
  //    /****************** 断开 UDP 连接 *******************/
  HAL_UART_Transmit(&huart2, At_CipClose, strlen((char*)At_CipClose), 0xffff); 
  HAL_Delay(TimeDelay);
  Reset_RxbuffClear();
  /****************** 建立 UDP 连接 *******************/
  HAL_UART_Transmit(&huart2, At_CipStart, strlen((char*)At_CipStart), 0xffff);
  HAL_Delay(1000);
  Reset_RxbuffClear(); 
  /****************** 查询IP *******************/
  Wifi_Read_IP(); 
  HAL_Delay(TimeDelay);
  Reset_RxbuffClear();
  /****************** 设置透传模式 *******************/
  HAL_UART_Transmit(&huart2, At_CipMode, strlen((char*)At_CipMode), 0xffff);
  HAL_Delay(TimeDelay);
  Reset_RxbuffClear();

  /****************** 开启透传模式 *******************/
  HAL_UART_Transmit(&huart2, At_CipSend, strlen((char*)At_CipSend), 0xffff);
  HAL_Delay(TimeDelay);
  if (0 == strncmp((char*)UsartType2.usartDMA_rxBuf, (char*)At_CipSend_Ack, strlen((char*)At_CipSend_Ack))) 
  {
#if (DEBUG_PRINTF == TRUE) 
    printf("Start Transparent Success\n");
#endif
    Reset_RxbuffClear();
  }
  else
  {
#if (DEBUG_PRINTF == TRUE) 
    printf("Start Transparent Fail\n");
#endif
    goto REST;
  }
  
  NetWorkState_LED2_Flag = (~NetWorkState_LED2_Flag);
  NetWorkState_LED3_Flag = (~NetWorkState_LED3_Flag);
  //关闭收发LED的灯
  NetWorkLedstatusOnOff(NetWorkState_LED2, NetWorkState_LED2_Flag);
  NetWorkLedstatusOnOff(NetWorkState_LED3, NetWorkState_LED3_Flag);
  
  //进入透传模式
  NowWifiConfig = 1; 
  
  UsartType2.receive_flag = 0;
}
void Wifi_Read_IP(void)
{
  uint8_t i=0;
  HAL_UART_Transmit(&huart2, At_Cifsr, strlen((char*)At_Cifsr), 0xffff);
  HAL_Delay(TimeDelay);
  // if (0 == strncmp((char*)UsartType2.usartDMA_rxBuf, (char*)At_CipSend_Ack, strlen((char*)At_CipSend_Ack))) 
  //  strstr(usartDMA_rxBuf,STAIP)
  char delims[] = "\"";
  char *result = NULL;
  result = strtok( (char*)UsartType2.usartDMA_rxBuf, delims );
  //result = "192.168.1.1";
  while( result != NULL ) {
//    printf("result is \"%s\"\n", result );
    if(i++ == 5 )
    {  
//        StationIP = result;
      
     //rcm add 
      //memcpy(StationIP,result,strlen(result));//old
     memcpy(StationIP,result,strlen(result));//new
//    printf("StationIP:%s\n", StationIP );
    }   
    result=strtok(NULL,delims);
  }
}
uint8_t Wifi_RESTORE(void)//wifi恢复出厂设置
{
REST: 
  /******************start 退出透传*************************/
  HAL_UART_Transmit(&huart2, At_QuitCipsend, strlen((char*)At_QuitCipsend), 0xffff);
  while(UsartType2.receive_flag != 1);
  if (0 != strncmp((char*)UsartType2.usartDMA_rxBuf, (char*)At_QuitCipsend_Ack, strlen((char*)At_QuitCipsend_Ack))) 
  {
    Reset_RxbuffClear(); 
    goto REST;
  }
  else
  {
    Reset_RxbuffClear(); 
  }
  
  HAL_UART_Transmit(&huart2, At_Restore, strlen((char*)At_Restore), 0xffff);
  
//   while(UsartType2.receive_flag != 1);
//   while(!strstr((char *)UsartType2.usartDMA_rxBuf,"@ 1000"))
//   {
//     Reset_RxbuffClear();
//     while(UsartType2.receive_flag != 1);
//   }

   __HAL_UART_DISABLE(&huart2);
   __HAL_UART_DISABLE_IT(&huart2,UART_IT_IDLE); 
   
   HAL_Delay(1000);
   Reset_RxbuffClear();
   __HAL_UART_ENABLE(&huart2);
   __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
   Reset_RxbuffClear();   
//    while(UsartType2.receive_flag != 1);
//    while(!strstr((char *)UsartType2.usartDMA_rxBuf,"ready"))
//   {
//     Reset_RxbuffClear();
//     HAL_Delay(100);
//     while(UsartType2.receive_flag != 1);
//   }
//   Reset_RxbuffClear();
   return 1; 


}

