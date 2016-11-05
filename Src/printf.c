#include <stm32f1xx_hal.h>
extern UART_HandleTypeDef huart1;

int stdout_putchar (int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 10);
	return ch;
}
int stderr_putchar (int ch) {
	while (HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 10) != HAL_OK);
	return ch;
}
int stdin_getchar(void) {
	uint8_t buf[1];
	while (HAL_UART_Receive(&huart1, buf, 1, 10) != HAL_OK);
	return *buf;
}
