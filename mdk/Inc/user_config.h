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

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern struct motion motion[3];

#endif /* __USER_CONFIG_H */
