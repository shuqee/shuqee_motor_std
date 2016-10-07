#include "stm32f1xx_hal.h"
#include "user_config.h"
#include "user_uart.h"

#define UART_BUFF_SIZE 20

extern UART_HandleTypeDef huart1;

struct frame
{
	uint8_t enable;
	uint8_t data;
	uint8_t buff[UART_BUFF_SIZE];
	uint8_t index;
};

struct frame frame = {1};

void user_uart_init(void)
{
	HAL_GPIO_WritePin(OUTPUT_485RW_GPIO_Port, OUTPUT_485RW_Pin, GPIO_PIN_SET);
	HAL_UART_Receive_IT(&huart1, &(frame.data), 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(frame.index == 0 && frame.data == 0xff)
	{
		frame.index++;
		return;
	}
	if(frame.index == 1)
	{
		switch (frame.data)
		{
			case 0xc2:
				frame.index++;
				return;
			case 0xff:
				return;
			default :
				frame.index = 0;
				return;
		}
	}
	if(frame.index >= 8)
	{
		switch (frame.data)
		{
			case 0xee:
				SAFE(motion[0].high.set = frame.buff[2]<<6);
				SAFE(motion[1].high.set = frame.buff[3]<<6);
				SAFE(motion[2].high.set = frame.buff[4]<<6);
				frame.enable = 1;
				frame.index = 0;
				return;
			default :
				frame.index = 0;
				return;
		}
	}
	frame.buff[frame.index] = frame.data;
	frame.index++;

	HAL_UART_Receive_IT(&huart1, &(frame.data), 1);
	//HAL_GPIO_TogglePin(OUTPUT_SEATLED3_GPIO_Port, OUTPUT_SEATLED3_Pin);
}
