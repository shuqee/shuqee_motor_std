#ifndef __USER_TIME_H
#define __USER_TIME_H

#include "stm32f1xx_hal.h"
#include "user_config.h"

extern void user_time_init(void);
extern void user_time_stop(void);
extern void delay_ns(uint32_t times);
extern void delay_us(uint32_t times);
extern void delay_ms(uint32_t times);
#ifdef ENV_RESET
extern void set_pul(enum motion_num index, GPIO_PinState dir, uint16_t speed, uint32_t conut);
#endif

extern void user_time_init(void);
extern uint8_t get_tick_flag(void);
extern void clr_tick_flag(void);
#endif /* __USER_TIME_H */
