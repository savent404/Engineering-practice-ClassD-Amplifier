#include "LMD18200.h"

extern TIM_HandleTypeDef htim1;

void LMD18200_Break(char cmd) {
	if (cmd) {
		HAL_GPIO_WritePin(BREAK_GPIO_Port, BREAK_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(BREAK_GPIO_Port, BREAK_Pin, GPIO_PIN_RESET);
	}
}

void LMD18200_Drive(float offset) {
	if (offset > 0) {
		LMD18200_Dir(1);
		LMD18200_Dutyc(offset);
		LMD18200_Break(0);
	}
	else if (offset < 0) {
		LMD18200_Dir(0);
		LMD18200_Dutyc(-offset);
		LMD18200_Break(0);
	}
	else if (offset < -0.5 || offset > 0.5) {
		LMD18200_Break(0);
	}
	else {
		LMD18200_Break(1);
		LMD18200_Dutyc(0);
	}
}

float LMD18200_I(void) {
	return 0.0f;
}
float LMD18200_T(void) {
	return 0.0f;
}

void LMD18200_Init(void) {
	LMD18200_Break(1);
	LMD18200_Dir(1);
	LMD18200_Dutyc(0);
}
static void LMD18200_Dir(char cmd) {
	if (cmd > 0) {
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	}
}
static void LMD18200_Dutyc(float dutycycle) {
	uint16_t buf = dutycycle * 160;
	if (buf) {
		TIM1->CCR1 = buf - 1;
	}
	else {
		TIM1->CCR1 = buf;
	}
}
