#include <stm32f1xx_hal.h>
#include <string.h>
extern UART_HandleTypeDef huart1;
static uint8_t tx_buf[30];
static __IO uint8_t rx_buf[30];
static __IO uint8_t rx_cnt = 0;
static __IO uint8_t rx_flag = 0;
int stdout_putchar (int ch) {
	static uint8_t cnt = 0;
	if (ch != '\n')
		tx_buf[cnt++] = ch;
	else {
		tx_buf[cnt++] = ch;
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)&tx_buf, cnt);
		cnt = 0;
	}
	return ch;
}
int stderr_putchar (int ch) {
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)&ch, 1);
	return ch;
}
int usr_puts(char *pt) {
	
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	rx_flag = 1;
}
int stdin_getchar(void) {
	uint8_t buf[1];
	HAL_UART_Receive_IT(&huart1, (uint8_t*)(rx_buf + rx_cnt), 1);
	while (!rx_flag);
	rx_flag = 0;
	return *(rx_buf + rx_cnt);
}
