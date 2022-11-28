#ifndef SWITCH_H
#define SWITCH_H

#include "fs_protocol.h"

#define SWITCHPIN   P1_2


void PWM_Init(void);
void SwitchOnOff(uint8_t type);
void Switch_FAN(uint8 type);
void Switch_BEEP(uint8 type);

#endif

