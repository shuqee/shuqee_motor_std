#include "stm32f1xx_hal.h"
#include "user_config.h"
#include "user_io.h"

#include <string.h>



#ifdef ENV_24V_SENOR
#define ADC_TH 0x03e8
#define ADC_BUFF_SIZE 6
#define SEAT_COUNT 6
enum adc_item
{
	ADC_ITEM_SEAT1 = 0,
	ADC_ITEM_SEAT2,
	ADC_ITEM_SEAT3,
	ADC_ITEM_SEAT4,
	ADC_ITEM_SEAT5,
	/*加入的24V检测口*/
	ADC_ITEM_SEAT6,
	ADC_ITEM_COUNT
};
#else
#define ADC_TH 0x03e8
#define ADC_BUFF_SIZE 5
#define SEAT_COUNT 5

enum adc_item
{
	ADC_ITEM_SEAT1 = 0,
	ADC_ITEM_SEAT2,
	ADC_ITEM_SEAT3,
	ADC_ITEM_SEAT4,
	ADC_ITEM_SEAT5,
	ADC_ITEM_COUNT
};
	#endif
extern int flag_rst;

static __IO uint16_t adc_buf[ADC_BUFF_SIZE][ADC_ITEM_COUNT] = {0};
static __IO uint16_t adc_result[ADC_ITEM_COUNT] = {0};

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

void user_io_init(void)
{
	HAL_GPIO_WritePin(OUTPUT_SP3_GPIO_Port, OUTPUT_SP3_Pin, GPIO_PIN_SET); /* 关闭特效 */
	HAL_GPIO_WritePin(OUTPUT_SP4_GPIO_Port, OUTPUT_SP4_Pin, GPIO_PIN_SET); /* 关闭特效 */
	HAL_GPIO_WritePin(OUTPUT_SP5_GPIO_Port, OUTPUT_SP5_Pin, GPIO_PIN_SET); /* 关闭特效 */
	HAL_GPIO_WritePin(OUTPUT_SP6_GPIO_Port, OUTPUT_SP6_Pin, GPIO_PIN_SET); /* 关闭特效 */
	HAL_GPIO_WritePin(OUTPUT_SP7_GPIO_Port, OUTPUT_SP7_Pin, GPIO_PIN_SET); /* 关闭特效 */
	HAL_GPIO_WritePin(OUTPUT_SP8_GPIO_Port, OUTPUT_SP8_Pin, GPIO_PIN_SET); /* 关闭特效 */
		/*开启573  OE1，OE2,OE3*/
	HAL_GPIO_WritePin(OE_CE_GPIO_Port, OE_CE_Pin, GPIO_PIN_RESET); /* 开启573的使能端 */
	HAL_GPIO_WritePin(OUTPUT_573LE1_GPIO_Port, OUTPUT_573LE1_Pin, GPIO_PIN_SET);//使能锁存器
	HAL_GPIO_WritePin(OUTPUT_573LE2_GPIO_Port, OUTPUT_573LE2_Pin, GPIO_PIN_SET);//使能锁存器
	HAL_GPIO_WritePin(OUTPUT_573LE3_GPIO_Port, OUTPUT_573LE3_Pin, GPIO_PIN_SET);//使能锁存器
	
	HAL_GPIO_WritePin(OUTPUT_485RW_GPIO_Port, OUTPUT_485RW_Pin, GPIO_PIN_SET);	//485接收
	
	HAL_GPIO_WritePin(OUTPUT_NUP1_GPIO_Port, OUTPUT_NUP1_Pin, GPIO_PIN_SET);//允许上升
	HAL_GPIO_WritePin(OUTPUT_NDOWN1_GPIO_Port, OUTPUT_NDOWN1_Pin, GPIO_PIN_SET);//允许下降
	HAL_GPIO_WritePin(OUTPUT_NUP2_GPIO_Port, OUTPUT_NUP2_Pin, GPIO_PIN_SET);//允许上升
	HAL_GPIO_WritePin(OUTPUT_NDOWN2_GPIO_Port, OUTPUT_NDOWN2_Pin, GPIO_PIN_SET);//允许下降
	HAL_GPIO_WritePin(OUTPUT_NUP3_GPIO_Port, OUTPUT_NUP3_Pin, GPIO_PIN_SET);//允许上升
	HAL_GPIO_WritePin(OUTPUT_NDOWN3_GPIO_Port, OUTPUT_NDOWN3_Pin, GPIO_PIN_SET);//允许下降

	
	HAL_GPIO_WritePin(OUTPUT_CLR1_GPIO_Port, OUTPUT_CLR1_Pin, GPIO_PIN_SET);//消除警报
	HAL_GPIO_WritePin(OUTPUT_CLR2_GPIO_Port, OUTPUT_CLR2_Pin, GPIO_PIN_SET);//消除警报
	HAL_GPIO_WritePin(OUTPUT_CLR3_GPIO_Port, OUTPUT_CLR3_Pin, GPIO_PIN_SET);//消除警报
	
	memset((void *)adc_buf, 0, sizeof(adc_buf));
	memset((void *)adc_result, 0, sizeof(adc_result));
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUFF_SIZE*ADC_ITEM_COUNT);
}

void user_adc_start(void)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUFF_SIZE*ADC_ITEM_COUNT);
}

void user_adc_stop(void)
{
	HAL_ADC_Stop_DMA(&hadc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	int i,j;
	uint32_t sum = 0;
	uint32_t avg;
	static int delay_count[SEAT_COUNT] = {0};
	uint16_t seat_num_tmp = 0;
	
	(void)hadc;
	for(i=0; i<ADC_ITEM_COUNT; i++)
	{
		sum = 0;
		for(j=0; j<ADC_BUFF_SIZE; j++)
		{
			sum = sum + (uint32_t)adc_buf[j][i];
		}
		avg = sum/ADC_BUFF_SIZE;
		adc_result[i] = (uint16_t)avg;
	}
	
	for(i=0; i<SEAT_COUNT; i++)
	{
		if(adc_result[i] < ADC_TH)
			delay_count[i] = 1000;
		else
		{
			if(delay_count[i])
				delay_count[i]--;
		}
	}
	
	for(i=0; i<SEAT_COUNT; i++)
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
	
	if(delay_count[4])
		LED_SEAT5(1);
	else
		LED_SEAT5(0);
	/*加入的24V检测*/
	#ifdef ENV_24V_SENOR
		if(delay_count[5])
		LED_SEAT5(1);
	else
		LED_SEAT5(0);
	#endif
	status.seat_num = seat_num_tmp;
}

#ifdef ENV_NOSENSOR
void down_limit(enum motion_num index)
{
	if(motion[index].high.now < 127 * ENV_SPACE)
		HAL_GPIO_WritePin(motion[index].io.ndown_port, motion[index].io.ndown_pin, GPIO_PIN_RESET);//禁止下降
	else
		HAL_GPIO_WritePin(motion[index].io.nup_port, motion[index].io.nup_pin, GPIO_PIN_RESET);//禁止上升
	if(flag_rst == 0)
	{
		if(motion[index].high.now < 127 * ENV_SPACE)
			motion[index].high.now = (0-motion[index].config.adj) * ENV_SPACE;
		else
			motion[index].high.now = (255+motion[index].config.adj) * ENV_SPACE;
	}
}

void up_limit(enum motion_num index)
{

}
#else
void down_limit(enum motion_num index)
{
	HAL_GPIO_WritePin(motion[index].io.ndown_port, motion[index].io.ndown_pin, GPIO_PIN_RESET);//禁止下降
	if ((flag_rst == 0) && (status.uplimit[index] == 0))
		motion[index].high.now = (0-motion[index].config.adj) * ENV_SPACE;
}

void up_limit(enum motion_num index)
{
	HAL_GPIO_WritePin(motion[index].io.nup_port, motion[index].io.nup_pin, GPIO_PIN_RESET);//禁止上升
	if ((flag_rst == 0) && (status.downlimit[index] == 0))
		motion[index].high.now = (255+motion[index].config.adj) * ENV_SPACE;
}
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case EXTI_UPLIMIT1_Pin:
			if(GET_UPLIMIT1())	//一号缸上限位有效
			{
				if(!status.uplimit[MOTION1])	//消除抖动的判断
				{
					status.uplimit[MOTION1] = 1;	//消除抖动的操作
					up_limit(MOTION1);
					Uplimit1(1);
				}
			}
			else
			{
				if(status.uplimit[MOTION1])
					status.uplimit[MOTION1] = 0;
				 Uplimit1(0);
			}
			break;
		case EXTI_DOWNLIMIT1_Pin:
			if(GET_DOWNLIMIT1())
			{
				if(!status.downlimit[MOTION1])
				{
					status.downlimit[MOTION1] = 1;
					down_limit(MOTION1);
					Downlimit1(1);
				}
			}
			else
			{
				if(status.downlimit[MOTION1])
					status.downlimit[MOTION1] = 0;
				Downlimit1(0);
			}
			break;
		case EXTI_UPLIMIT2_Pin:
			if(GET_UPLIMIT2())
			{
				if(!status.uplimit[MOTION2])
				{
					status.uplimit[MOTION2] = 1;
					up_limit(MOTION2);
					Uplimit2(1);
				}
			}
			else
			{
				if(status.uplimit[MOTION2])
				{
					status.uplimit[MOTION2] = 0;
					Uplimit2(0);
				}
			}
			break;
		case EXTI_DOWNLIMIT2_Pin:
			if(GET_DOWNLIMIT2())
			{
				if(!status.downlimit[MOTION2])
				{
					status.downlimit[MOTION2] = 1;
					down_limit(MOTION2);
					Downlimit2(1);
				}
			}
			else
			{
				if(status.downlimit[MOTION2])
					status.downlimit[MOTION2] = 0;
				 Downlimit2(0);
			}
			break;	
		case EXTI_UPLIMIT3_Pin:
			if(GET_UPLIMIT3())
			{
				if(!status.uplimit[MOTION3])
				{
					status.uplimit[MOTION3] = 1;
					up_limit(MOTION3);
					Uplimit3(1);
				}
			}
			else
			{
				if(status.uplimit[MOTION3])
				{
					status.uplimit[MOTION3] = 0;
					Uplimit3(0);
				}
			}
			break;
		case EXTI_DOWNLIMIT3_Pin:
			if(GET_DOWNLIMIT3())
			{
				if(!status.downlimit[MOTION3])
				{
					status.downlimit[MOTION3] = 1;
					down_limit(MOTION3);
					Downlimit3(1);
				}
			}
			else
			{
				if(status.downlimit[MOTION3])
					status.downlimit[MOTION3] = 0;
				Downlimit3(0);
			}
			break;	
		default:
			break;
	}
}
