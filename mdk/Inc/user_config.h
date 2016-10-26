#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

#include "stm32f1xx_hal.h"

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
};

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern struct motion motion[3];
extern struct status status;

#endif /* __USER_CONFIG_H */
