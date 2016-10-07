#include "stm32f1xx_hal.h"
#include "user_io.h"

void user_io_init(void)
{
	HAL_GPIO_WritePin(OUTPUT_573LE1_GPIO_Port, OUTPUT_573LE1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_573LE2_GPIO_Port, OUTPUT_573LE2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_573LE3_GPIO_Port, OUTPUT_573LE3_Pin, GPIO_PIN_SET);
}
