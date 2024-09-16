#ifndef SR04_H
#define SR04_H

#include "stdio.h"
#include "stdint.h"

#define SPEED 340.0f
#define SR04_COUNT_PERIOD 10000

typedef struct
{
	uint32_t IC_Val1;
	uint32_t IC_Val2;
	uint32_t Difference;
	uint8_t Is_First_Captured; // is the first value captured ?
	uint8_t Distance;
} SR04_PulseType;

 extern SR04_PulseType pulse;
 extern SR04_PulseType pulse2;

void SR04_Init();
void SR04_Read();
// void SR04_Calculate();

#endif
