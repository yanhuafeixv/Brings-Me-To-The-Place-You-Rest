#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f10x.h"

void Track_Init(void);
uint8_t Track_GetStatus(void);
void Track_Process(void);

extern uint16_t TRACK_THRESHOLD;

#endif