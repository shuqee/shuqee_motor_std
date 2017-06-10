#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

#include "stm32f1xx_hal.h"

//#define DEBUG_ENV	//调试模式

#define ENV_IWDG

//#define ENV_3DOF_NO_SENSOR	 //三自由度平台不带传感器
//#define ENV_3DOF	           //三自由度平台直线缸式
//#define ENV_3DOF_SWING_LINK  //三自由度平台摆杆式
#define ENV_2DOF	           //二自由度平台摆杆式

#ifdef ENV_3DOF_NO_SENSOR
	#define ENV_NOSENSOR	//没有传感器
	#define ENV_RESET	//复位
	#define MOTION1_ENABLE
	#define MOTION2_ENABLE
	#define MOTION3_ENABLE
	#define MOTION1_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION2_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION3_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION1_CONFIG_ORIGIN	0
	#define MOTION2_CONFIG_ORIGIN	0
	#define MOTION3_CONFIG_ORIGIN	0
	#define MOTION1_CONFIG_ADJ		10
	#define MOTION2_CONFIG_ADJ		10
	#define MOTION3_CONFIG_ADJ		10
	#define ENV_SPACE 46                  //位置扩大倍数为2的ENV_SPACE次方
	#define ENV_SPEED_MAX 37        //最大速度对应的定时器重载值
	#define ENV_ACCER     (ENV_SPACE * (uint32_t)255 * (uint32_t)20)
#endif

#ifdef ENV_3DOF_SWING_LINK
	#define ENV_RESET	//复位
	#define ENV_SWING_LINK
	//#define ENV_SHAKE
	#define MOTION1_ENABLE
	#define MOTION2_ENABLE
	#define MOTION3_ENABLE
	#define MOTION1_CONFIG_DIR	GPIO_PIN_SET
	#define MOTION2_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION3_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION1_CONFIG_ORIGIN	0
	#define MOTION2_CONFIG_ORIGIN	0
	#define MOTION3_CONFIG_ORIGIN	0
	#define MOTION1_CONFIG_ADJ		0
	#define MOTION2_CONFIG_ADJ		0
	#define MOTION3_CONFIG_ADJ		0
	#define ENV_SPACE 25                  //位置扩大倍数为2的ENV_SPACE次方
	#define ENV_SPEED_MAX 37        //最大速度对应的定时器重载值
	#define ENV_ACCER     (ENV_SPACE * (uint32_t)255 * (uint32_t)20)
#endif

#ifdef ENV_2DOF
	#define ENV_RESET	//复位
	#define ENV_SWING_LINK
	#define MOTION1_ENABLE
	#define MOTION3_ENABLE
	#define MOTION1_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION2_CONFIG_DIR	GPIO_PIN_RESET
	#define MOTION3_CONFIG_DIR	GPIO_PIN_SET
	#define MOTION1_CONFIG_ORIGIN	127
	#define MOTION2_CONFIG_ORIGIN	0
	#define MOTION3_CONFIG_ORIGIN	127
	#define MOTION1_CONFIG_ADJ		0
	#define MOTION2_CONFIG_ADJ		0
	#define MOTION3_CONFIG_ADJ		0
	#define ENV_SPACE 25                  //位置扩大倍数为2的ENV_SPACE次方
	#define ENV_SPEED_MAX 37        //最大速度对应的定时器重载值
	#define ENV_ACCER     (ENV_SPACE * (uint32_t)255 * (uint32_t)20)
#endif

#define SPB_AIR_INJECTION_MASK (0x1<<6)


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

enum motion_num
{
	MOTION1 = 0,
	MOTION2,
	MOTION3,
	MOTION_COUNT
};

struct motion_io
{
	GPIO_TypeDef *	dir_port;
	uint16_t				dir_pin;	
	GPIO_TypeDef *	pul_port;
	uint16_t				pul_pin;	
	GPIO_TypeDef *	nup_port;
	uint16_t				nup_pin;
	GPIO_TypeDef *	ndown_port;
	uint16_t				ndown_pin;	
};

struct motion_config
{
	GPIO_PinState dir;	//缸设置的运动方向
	int origin;			//缸起始位置
	int adj;			//复位及上下限位触发时的调整距离
};

struct motion_status
{
	enum motion_num index;			//缸索引值
	struct high high;	
	GPIO_PinState dir;	//缸当前运动方向
	struct motion_io io;
	struct motion_config config;
};

struct status
{
	uint8_t id;					//座椅编号
	uint8_t seat_num;		//座椅人数
	uint8_t seat_enable;	//座椅使能位(seat_num||seat_enable为真座椅动作)
	uint8_t spb;				//座椅特效
	uint8_t uplimit[MOTION_COUNT];
	uint8_t downlimit[MOTION_COUNT];
};

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern struct motion_status motion[MOTION_COUNT];
extern struct status status;

#endif /* __USER_CONFIG_H */
