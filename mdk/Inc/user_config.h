#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

#include "stm32f1xx_hal.h"

#define ENV_3DOF

#ifndef ENV_3DOF
	#define ENV_2DOF
#endif

#define ENV_MY_TEST

#ifndef ENV_MY_TEST
	#define ENV_SPACE 6
	#define ENV_SPEED_MAX 27
#else
	#define ENV_SPACE 4
	#define ENV_SPEED_MAX 35
#endif


#define SAFE(x) do{ \
	__set_PRIMASK(1); \
	x; \
	__set_PRIMASK(0); \
}while(0)

struct high
{
	uint32_t now;
	uint32_t set;
};

struct motion
{
	uint8_t index;
	struct high high;
	GPIO_PinState dir;
};

struct status
{
	uint8_t id;//座椅编号
	uint8_t seat_num;//座椅人数
	uint8_t seat_enable;//座椅使能位(seat_num||seat_enable为真座椅动作)
	uint8_t spb;//座椅特效
	uint8_t uplimit1;
	uint8_t downlimit1;
	uint8_t uplimit2;
	uint8_t downlimit2;
	uint8_t uplimit3;
	uint8_t downlimit3;
	int debug;
	int exit_count;
};

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern struct motion motion[3];
extern struct status status;

#endif /* __USER_CONFIG_H */
