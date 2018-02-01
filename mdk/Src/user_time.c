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

void user_time_stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Stop_IT(&htim3);
}

#ifdef ENV_RESET
void set_pul(enum motion_num index, GPIO_PinState dir, uint16_t speed, uint32_t conut)	//设定输出固定个数、频率、方向、缸号的脉冲
{
	enum motion_num i;

	if (motion[index].config.dir == GPIO_PIN_SET)
		dir = (dir == GPIO_PIN_RESET)?GPIO_PIN_SET:GPIO_PIN_RESET; /* 取反 */
	HAL_GPIO_WritePin(motion[index].io.dir_port, motion[index].io.dir_pin, dir);
	delay_us(speed);
	for(i=MOTION1; i<MOTION_COUNT; i++)
	{
		HAL_GPIO_WritePin(motion[index].io.pul_port, motion[index].io.pul_pin, GPIO_PIN_RESET);
		delay_us(speed);
		HAL_GPIO_WritePin(motion[index].io.pul_port, motion[index].io.pul_pin, GPIO_PIN_SET);
		delay_us(speed);
	}
}
#endif

int output_pul(enum motion_num index, GPIO_PinState sign)	//脉冲方向输出函数
{
	GPIO_PinState dir = motion[index].dir;	//获取当前缸运动方向
	GPIO_PinState out_dir = sign;
	static uint8_t status[MOTION_COUNT] = {0,0,0};
	
	switch(status[index])
	{
		case 0:
			if (motion[index].config.dir == GPIO_PIN_SET)
				out_dir = (sign == GPIO_PIN_RESET)?GPIO_PIN_SET:GPIO_PIN_RESET; /* 取反 */
			HAL_GPIO_WritePin(motion[index].io.dir_port, motion[index].io.dir_pin, out_dir);	//更新运动方向到IO输出
			if(dir != sign)	//如果当前运动方向与设定方向不一致
			{
				motion[index].dir = sign;	//更新运动方向
				return 0;					//退出不输出脉冲
			}
			HAL_GPIO_WritePin(motion[index].io.pul_port, motion[index].io.pul_pin, GPIO_PIN_RESET);	//输出脉冲
			status[index]++;	//记录脉冲输出正在进行
			return 0;
		case 1:
			HAL_GPIO_WritePin(motion[index].io.pul_port, motion[index].io.pul_pin, GPIO_PIN_SET);	//关闭脉冲
			status[index] = 0;	//记录脉冲输出完成
			return (dir?-1:1);
		default:
			status[index] = 0;
			return 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	enum motion_num index;
	int now;
	int set;
	static uint32_t interval = 999;
	if(htim->Instance == TIM1)	//判断是哪个定时器中断
		index = MOTION1;	//取出对应缸状态记录空间的指针
	else{if(htim->Instance == TIM2)
		index = MOTION2;
	else{if(htim->Instance == TIM3)
		index = MOTION3;
	else
		return;
	}}
	SAFE(now = motion[index].high.now);
	set = motion[index].high.set;
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
	SAFE(motion[index].high.now += output_pul(index, (now < set)?GPIO_PIN_RESET:GPIO_PIN_SET));	//计算步数
}
