#ifndef _LMD18200_H_
#define _LMD18200_H_

#include <stm32f1xx_hal.h>

/**
  * @Brief: Break device
  * @Para : @cmd \0 break, \~0 run.
  */
void LMD18200_Break(char cmd);

/**
  * @Brief: ctl dir and speed.
  * @Para : offset is in -1~1
  */
void LMD18200_Drive(float offset);

/**
  * @Brief: read electric I
  */
float LMD18200_I(void);

/**
  * @Brief: read warning Voltage
  */
float LMD18200_T(void);

void LMD18200_Init(void);

static void LMD18200_Dir(char cmd);
static void LMD18200_Dutyc(float dutycycle);

#endif
