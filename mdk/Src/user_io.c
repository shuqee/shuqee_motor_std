#include "stm32f1xx_hal.h"
#include "user_config.h"
#include "user_io.h"

#define ADC_TH 0x03e8

static uint16_t adc[5][4];

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

void user_io_init(void)
{
	HAL_GPIO_WritePin(OUTPUT_573LE1_GPIO_Port, OUTPUT_573LE1_Pin, GPIO_PIN_SET);//使能锁存器
	HAL_GPIO_WritePin(OUTPUT_573LE2_GPIO_Port, OUTPUT_573LE2_Pin, GPIO_PIN_SET);//使能锁存器
	HAL_GPIO_WritePin(OUTPUT_573LE3_GPIO_Port, OUTPUT_573LE3_Pin, GPIO_PIN_SET);//使能锁存器
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc, 20);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint16_t adc_tmp;
	uint16_t seat_num_tmp = 0;
	static int delay_count[4] = {0};
	int i;
	
	for(i=0; i<4; i++)
	{
		adc_tmp = (adc[0][i] + adc[1][i] + adc[2][i] + adc[3][i] + adc[4][i])/5;
		if(adc_tmp < ADC_TH)
			delay_count[i] = 1000;
		else
		{
			if(delay_count[i])
				delay_count[i]--;
		}
	}
	
	for(i=0; i<4; i++)
	{
		if(delay_count[i])
			seat_num_tmp++;
	}
	
	if(delay_count[0])
		LED_SEAT1(1);
	else
		LED_SEAT1(0);
	
	if(delay_count[1])
		LED_SEAT2(1);
	else
		LED_SEAT2(0);
	
	if(delay_count[2])
		LED_SEAT3(1);
	else
		LED_SEAT3(0);
	
	if(delay_count[3])
		LED_SEAT4(1);
	else
		LED_SEAT4(0);
	
	status.seat_num = seat_num_tmp;
}
