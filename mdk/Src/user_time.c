#include "stm32f1xx_hal.h"
#include "user_config.h"
#include "user_time.h"

void user_time_init(void)
{
	__HAL_TIM_SET_AUTORELOAD(&htim1, 999);
	HAL_TIM_Base_Start_IT(&htim1);
	__HAL_TIM_SET_AUTORELOAD(&htim2, 999);
	HAL_TIM_Base_Start_IT(&htim2);
	__HAL_TIM_SET_AUTORELOAD(&htim3, 999);
	HAL_TIM_Base_Start_IT(&htim3);
}

int output_pul(struct motion *pmotion, GPIO_PinState sign)
{
	uint8_t index = pmotion->index;
	GPIO_PinState dir = pmotion->dir;
	static uint8_t status[3] = {0,0,0};
	
	static GPIO_TypeDef *dir_port[3] = {OUTPUT_DIR1_GPIO_Port,OUTPUT_DIR2_GPIO_Port,OUTPUT_DIR3_GPIO_Port};
	static uint16_t dir_pin[3] = {OUTPUT_DIR1_Pin,OUTPUT_DIR2_Pin,OUTPUT_DIR3_Pin};
	
	static GPIO_TypeDef *pul_port[3] = {OUTPUT_PUL1_GPIO_Port,OUTPUT_PUL2_GPIO_Port,OUTPUT_PUL3_GPIO_Port};
	static uint16_t pul_pin[3] = {OUTPUT_PUL1_Pin,OUTPUT_PUL2_Pin,OUTPUT_PUL3_Pin};
	
	//static GPIO_PinState dir = GPIO_PIN_RESET;
	switch(status[index])
	{
		case 0:
			HAL_GPIO_WritePin(dir_port[index], dir_pin[index], sign);
			if(dir != sign)
			{
				pmotion->dir = sign;
				return 0;
			}
			HAL_GPIO_WritePin(pul_port[index], pul_pin[index], GPIO_PIN_RESET);
			status[index]++;
			return 0;
		case 1:
			HAL_GPIO_WritePin(pul_port[index], pul_pin[index], GPIO_PIN_SET);
			status[index] = 0;
			return (dir?-1:1);
		default:
			status[index] = 0;
			return 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	struct motion *pmotion;
	uint32_t now;
	uint32_t set;
	uint32_t speed = 0;
	if(htim->Instance == TIM1)
		pmotion = &motion[0];
	else{if(htim->Instance == TIM2)
		pmotion = &motion[1];
	else{if(htim->Instance == TIM3)
		pmotion = &motion[2];
	else
		return;
	}}
	SAFE(now = pmotion->high.now);
	set = pmotion->high.set;
	if(now == set)
	{
		__HAL_TIM_SET_AUTORELOAD(htim, 999);
		return;
	}
	if(now < set)
		speed = (0xffff)/(set-now);
	else
		speed =  (0xffff)/(now-set);
	speed = (speed<ENV_SPEED_MAX)?ENV_SPEED_MAX:speed;
	__HAL_TIM_SET_AUTORELOAD(htim, speed);
	SAFE(pmotion->high.now += output_pul(pmotion, (now < set)?GPIO_PIN_RESET:GPIO_PIN_SET));
}
