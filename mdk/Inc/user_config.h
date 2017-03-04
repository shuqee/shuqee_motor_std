#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

#include "stm32f1xx_hal.h"

#define DEBUG_ENV	//调试模式

#define ENV_3DOF	//三自由度平台
//#define ENV_2DOF	//二自由度平台


#define ENV_NOSENSOR	//没有传感器

#ifdef ENV_NOSENSOR
	
#endif

#ifndef DEBUG_ENV
	#define ENV_SPACE 6
	#define ENV_SPEED_MAX 27
	#define ENV_ACCER     (ENV_SPACE * 2000)
#else
	#define ENV_SPACE 31                  //位置扩大倍数为2的ENV_SPACE次方
	#define ENV_SPEED_MAX 37        //最大速度对应的定时器重载值
	#define ENV_ACCER     (ENV_SPACE * (uint32_t)255 * (uint32_t)20)
#endif


#define SAFE(x) do{ \
	__set_PRIMASK(1); \
	x; \
	__set_PRIMASK(0); \
}while(0)	//原子操作

struct high
{
	int now;	//当前缸位置记录
	int set;	//设定的缸目标位置
};

struct motion
{
	uint8_t index;			//缸索引值
	struct high high;	
	GPIO_PinState dir;	//缸当前运动方向
};

struct status
{
	uint8_t id;					//座椅编号
	uint8_t seat_num;		//座椅人数
	uint8_t seat_enable;	//座椅使能位(seat_num||seat_enable为真座椅动作)
	uint8_t spb;				//座椅特效
	uint8_t uplimit[3];
	uint8_t downlimit[3];
};

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern struct motion motion[3];
extern struct status status;

#endif /* __USER_CONFIG_H */
