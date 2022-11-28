#ifndef __WIFICONFIG_H
#define __WIFICONFIG_H

#include "stdint.h"


extern uint8_t NowWifiConfig;  //是否真处于配置模式，Wifi配置和接收两种模式

extern uint8_t *SSID;

extern uint8_t StationIP[14] ;

void Wifi_Config(void);
void Reset_RxbuffClear(void);
void Wifi_Read_IP(void);
uint8_t Wifi_RESTORE(void);



#endif
