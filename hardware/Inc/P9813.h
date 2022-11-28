#ifndef P9813_H
#define P9813_H

extern uint8_t jdat;
extern uint8_t rdat;
extern uint8_t gdat;
extern uint8_t bdat;


void P9813_Init(void);
void P9813_flg();
void P9813send_jdat(uint8_t dat);
void P9813colour_dat(uint8_t dat);
uint8_t P9813_get_jdat(uint8_t Red,uint8_t Green,uint8_t Blue);
void P9813_OnOff(uint8_t Red,uint8_t Green,uint8_t Blue);
#endif