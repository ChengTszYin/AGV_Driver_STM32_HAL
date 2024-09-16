#ifndef SR04_H
#define SR04_H

#include "main.h"
#include "tim.h"
#include "stdio.h"


#define SPEED 				340.0f
#define SR04_COUNT_PERIOD 	10000


typedef struct
{
	uint16_t start;
	uint16_t end;
	uint16_t cnt;
	float distance;
	uint8_t rising_flag;
}SR04_PulseType;


//extern SR04_PulseType pulse;
//extern SR04_PulseType pulse2;

void SR04_Init();
void SR04_Start();
void SR04_Calculate();

#endif
