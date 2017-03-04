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

void set_pul(uint8_t index, GPIO_PinState dir, uint16_t speed, uint32_t conut)	//设定输出固定个数、频率、方向、缸号的脉冲
{
	int i =0;
	static GPIO_TypeDef *dir_port[3] = {OUTPUT_DIR1_GPIO_Port,OUTPUT_DIR2_GPIO_Port,OUTPUT_DIR3_GPIO_Port};
	static uint16_t dir_pin[3] = {OUTPUT_DIR1_Pin,OUTPUT_DIR2_Pin,OUTPUT_DIR3_Pin};
	
	static GPIO_TypeDef *pul_port[3] = {OUTPUT_PUL1_GPIO_Port,OUTPUT_PUL2_GPIO_Port,OUTPUT_PUL3_GPIO_Port};
	static uint16_t pul_pin[3] = {OUTPUT_PUL1_Pin,OUTPUT_PUL2_Pin,OUTPUT_PUL3_Pin};
	
	HAL_GPIO_WritePin(dir_port[index], dir_pin[index], dir);
	delay_us(speed);
	for(i=0; i<conut; i++)
	{
		HAL_GPIO_WritePin(pul_port[index], pul_pin[index], GPIO_PIN_RESET);
		delay_us(speed);
		HAL_GPIO_WritePin(pul_port[index], pul_pin[index], GPIO_PIN_SET);
		delay_us(speed);
	}
}

int output_pul(struct motion *pmotion, GPIO_PinState sign)	//脉冲方向输出函数
{
	uint8_t index = pmotion->index;		//获取当前缸索引值
	GPIO_PinState dir = pmotion->dir;	//获取当前缸运动方向
	static uint8_t status[3] = {0,0,0};
	
	static GPIO_TypeDef *dir_port[3] = {OUTPUT_DIR1_GPIO_Port,OUTPUT_DIR2_GPIO_Port,OUTPUT_DIR3_GPIO_Port};
	static uint16_t dir_pin[3] = {OUTPUT_DIR1_Pin,OUTPUT_DIR2_Pin,OUTPUT_DIR3_Pin};
	
	static GPIO_TypeDef *pul_port[3] = {OUTPUT_PUL1_GPIO_Port,OUTPUT_PUL2_GPIO_Port,OUTPUT_PUL3_GPIO_Port};
	static uint16_t pul_pin[3] = {OUTPUT_PUL1_Pin,OUTPUT_PUL2_Pin,OUTPUT_PUL3_Pin};
	
	//static GPIO_PinState dir = GPIO_PIN_RESET;
	switch(status[index])
	{
		case 0:
			HAL_GPIO_WritePin(dir_port[index], dir_pin[index], sign);	//更新运动方向到IO输出
			if(dir != sign)	//如果当前运动方向与设定方向不一致
			{
				pmotion->dir = sign;	//更新运动方向
				return 0;					//退出不输出脉冲
			}
			HAL_GPIO_WritePin(pul_port[index], pul_pin[index], GPIO_PIN_RESET);	//输出脉冲
			status[index]++;	//记录脉冲输出正在进行
			return 0;
		case 1:
			HAL_GPIO_WritePin(pul_port[index], pul_pin[index], GPIO_PIN_SET);	//关闭脉冲
			status[index] = 0;	//记录脉冲输出完成
			return (dir?-1:1);
		default:
			status[index] = 0;
			return 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	struct motion *pmotion;
	int now;
	int set;
	static uint32_t interval = 999;
	if(htim->Instance == TIM1)	//判断是哪个定时器中断
		pmotion = &motion[0];	//取出对应缸状态记录空间的指针
	else{if(htim->Instance == TIM2)
		pmotion = &motion[1];
	else{if(htim->Instance == TIM3)
		pmotion = &motion[2];
	else
		return;
	}}
	SAFE(now = pmotion->high.now);
	set = pmotion->high.set;
	if(now == set)	//当前缸位置与设定缸目标位置一致，不做操作直接返回
	{
		interval = 999;
		__HAL_TIM_SET_AUTORELOAD(htim, interval);
		return;
	}
	if(now < set)
		interval = (ENV_ACCER)/(set-now);
	else
		interval =  (ENV_ACCER)/(now-set);
	interval = (interval<ENV_SPEED_MAX)?ENV_SPEED_MAX:interval;
	__HAL_TIM_SET_AUTORELOAD(htim, interval);
	SAFE(pmotion->high.now += output_pul(pmotion, (now < set)?GPIO_PIN_RESET:GPIO_PIN_SET));	//计算步数
}
