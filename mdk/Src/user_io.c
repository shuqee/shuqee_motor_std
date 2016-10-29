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
	
	HAL_GPIO_WritePin(OUTPUT_NUP1_GPIO_Port, OUTPUT_NUP1_Pin, GPIO_PIN_SET);//允许上升
	HAL_GPIO_WritePin(OUTPUT_NDOWN1_GPIO_Port, OUTPUT_NDOWN1_Pin, GPIO_PIN_SET);//允许下降
	HAL_GPIO_WritePin(OUTPUT_NUP2_GPIO_Port, OUTPUT_NUP2_Pin, GPIO_PIN_SET);//允许上升
	HAL_GPIO_WritePin(OUTPUT_NDOWN2_GPIO_Port, OUTPUT_NDOWN2_Pin, GPIO_PIN_SET);//允许下降
	HAL_GPIO_WritePin(OUTPUT_NUP3_GPIO_Port, OUTPUT_NUP3_Pin, GPIO_PIN_SET);//允许上升
	HAL_GPIO_WritePin(OUTPUT_NDOWN3_GPIO_Port, OUTPUT_NDOWN3_Pin, GPIO_PIN_SET);//允许下降
	
	HAL_GPIO_WritePin(OUTPUT_CLR1_GPIO_Port, OUTPUT_CLR1_Pin, GPIO_PIN_SET);//消除警报
	HAL_GPIO_WritePin(OUTPUT_CLR2_GPIO_Port, OUTPUT_CLR2_Pin, GPIO_PIN_SET);//消除警报
	HAL_GPIO_WritePin(OUTPUT_CLR3_GPIO_Port, OUTPUT_CLR3_Pin, GPIO_PIN_SET);//消除警报
	
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



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case EXTI_UPLIMIT1_Pin:
			//if(GET_UPLIMIT1())
			//{
				//if(!status.uplimit1)
				//{
					//status.debug = 1;
					//status.uplimit1 = 1;
					//status.exit_count++;
					//printf("down\r\n");
				//}
			//}
			//else
			//{
				//if(status.uplimit1)
				//{
					//status.debug = 1;
					//status.uplimit1 = 0;
					//status.exit_count--;
					//printf("up\r\n");
				//}
			//}
			break;
		case EXTI_DOWNLIMIT1_Pin:
			//printf("down1\r\n");
			break;
		case EXTI_UPLIMIT2_Pin:
			if(GET_UPLIMIT2())
			{
				HAL_GPIO_WritePin(OUTPUT_NUP2_GPIO_Port, OUTPUT_NUP2_Pin, GPIO_PIN_RESET);//禁止上升
			}
			else
			{
				HAL_GPIO_WritePin(OUTPUT_NUP2_GPIO_Port, OUTPUT_NUP2_Pin, GPIO_PIN_SET);//允许上升
				motion[1].high.now = 0xff << ENV_SPACE;
			}
			break;
		case EXTI_DOWNLIMIT2_Pin:
			if(GET_DOWNLIMIT2())
			{
				HAL_GPIO_WritePin(OUTPUT_NDOWN2_GPIO_Port, OUTPUT_NDOWN2_Pin, GPIO_PIN_RESET);//禁止下降
			}
			else
			{
				HAL_GPIO_WritePin(OUTPUT_NDOWN2_GPIO_Port, OUTPUT_NDOWN2_Pin, GPIO_PIN_SET);//允许下降
				motion[1].high.now = 0 << ENV_SPACE;
			}
			break;
		case EXTI_UPLIMIT3_Pin:
			//printf("up3\r\n");
			break;
		case EXTI_DOWNLIMIT3_Pin:
			//printf("down3\r\n");
			break;
		default:
			break;
	}
}
