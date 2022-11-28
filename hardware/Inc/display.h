#ifndef DISPLAY_H
#define DISPLAY_H

#include "bsp.h"
#include "fs_protocol.h"

extern uint8_t DeviceRssi;
extern uint32_t ultrasonicValue;

void DisplayDeviceLogo(void);
void DisplayData(void);


#endif
