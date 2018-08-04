#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

#include "stm32f1xx_hal.h"

//#define DEBUG_ENV	//调试模式
#define ENV_IWDG
#define ENV_24V_SENOR


#define ENV_3DOF_NO_SENSOR	 //三自由度平台不带传感器
//#define ENV_3DOF	           //三自由度平台直线缸式
//#define ENV_3DOF_SWING_LINK  //三自由度平台摆杆式
//#define ENV_2DOF	           //二自由度平台摆杆式

//#define ENV_SEAT_PICKING //选座功能(根据ID使能座椅)
//#define ENV_SEND_SEAT_INFO //统计人数功能(根据ID使能反馈座椅人数功能)


#ifdef ENV_3DOF_NO_SENSOR
//  #define SYNERON     //和利时驱动器
	#define DIRNA       //东菱驱动器
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
	#define MOTION1_CONFIG_ADJ		5
	#define MOTION2_CONFIG_ADJ		5
	#define MOTION3_CONFIG_ADJ		5
	#define ENV_RESET_SPEED 800 //ENV_RESET_SPEED越大复位速度越慢	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓和利时驱动电机参数↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/		
	#ifdef SYNERON
	#define ENV_CYLINDER_STROKE 150.0 /* 电动缸行程A:(150mm) B:(170mm)*/
	#define ENV_CYLINDER_REDUCTION_RATIO (1.5/1.0) /* 电动缸减速比A:(1.5:1) B:(1.8:1) */
	#define ENV_CYLINDER_SCREW_LEAD 5.0 /* 电动缸丝杆导程A:(5mm) B:(10mm)*/
	#define ENV_ELECTRONIC_GEAR_RATIO 40.0 /* 驱动器电子齿轮比A:(fn050:40) */
	#define ENV_CYLINDER_STROKE_PERCENT (90.0/100.0) /* 电动缸行程有效使用率(90%)(按百分比计算,不允许使用超过95%的行程,防止撞缸) */
	#define ENV_SPACE ((int)((ENV_CYLINDER_STROKE/ENV_CYLINDER_SCREW_LEAD) \
	                         *ENV_CYLINDER_REDUCTION_RATIO \
										       *10000.0 \
										       /ENV_ELECTRONIC_GEAR_RATIO \
										       /256.0 \
	                         *ENV_CYLINDER_STROKE_PERCENT)) /* 位置扩大倍数 */
	/* 170CM行程的时候EVN_SPACE 取46*/
//	#define ENV_SPACE 46                  //位置扩大倍数为2的ENV_SPACE次方
	/* 150cm行程的时候ENV_SPACE取40 */ 
	
	#define ENV_SPEED_MAX 37        //最大速度对应的定时器重载值
	#define ENV_ACCER     (ENV_SPACE * (uint32_t)256 * (uint32_t)20)
	#endif
	/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓东菱驱动电机参数↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/	
	#ifdef DIRNA
	#define RESET_FILTER_TIME 20            //定义复位的转矩到达时间过滤器；
	#define RUN_TILTER_TIME 500							//定义正常运行的转矩到达时间过滤器；
	#define ENV_CYLINDER_STROKE 95.0 /* 电动缸行程(95mm) */
	#define ENV_CYLINDER_REDUCTION_RATIO (2.5/1.0) /* 电动缸减速比(2.5:1) */
	#define ENV_CYLINDER_SCREW_LEAD 10.0 /* 电动缸丝杆导程(10mm) */
	#define ENV_ELECTRONIC_GEAR_RATIO 50.0 /* 驱动器电子齿轮比(PA205:50) */
	#define ENV_CYLINDER_STROKE_PERCENT (90.0/100.0) /* 电动缸行程有效使用率(90%)(按百分比计算,不允许使用超过95%的行程,防止撞缸) */
	#define ENV_SPACE ((int)((ENV_CYLINDER_STROKE/ENV_CYLINDER_SCREW_LEAD) \
	                         *ENV_CYLINDER_REDUCTION_RATIO \
										       *20000.0 \
										       /ENV_ELECTRONIC_GEAR_RATIO \
										       /256.0 \
	                         *ENV_CYLINDER_STROKE_PERCENT)) /* 位置扩大倍数 */
	/* 170CM行程的时候EVN_SPACE 取46*/
//	#define ENV_SPACE 46                  //位置扩大倍数为2的ENV_SPACE次方
	/* 150cm行程的时候ENV_SPACE取40 */ 
	
	#define ENV_SPEED_MAX 25        //最大速度对应的定时器重载值
	#define ENV_ACCER     (ENV_SPACE * (uint32_t)256 * (uint32_t)20)	
//	#define ENV_SPEED_MAX 32        //最大速度对应的定时器重载值
//	#define ENV_ACCER     (ENV_SPACE * (uint32_t)256 * (uint32_t)6)	
	#endif
	/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/	
#endif

#ifdef ENV_3DOF_SWING_LINK
	#define ENV_RESET	//复位
	#define ENV_SWING_LINK
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
//	#define ENV_SPACE 25                  //位置扩大倍数
  #define ENV_SPACE 70             //东菱驱动器
	#define ENV_SPEED_MAX 37        //最大速度对应的定时器重载值
	/* 中速 */
	#define ENV_ACCER     (ENV_SPACE * (uint32_t)256 * (uint32_t)12)
	/* 高速 */
//	#define ENV_ACCER     (ENV_SPACE * (uint32_t)256 * (uint32_t)8)
	#define ENV_RESET_SPEED 200 //ENV_RESET_SPEED越大复位速度越慢
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
	#define ENV_SPACE 25                  //位置扩大倍数
	#define ENV_SPEED_MAX 37        //最大速度对应的定时器重载值
	#define ENV_ACCER     (ENV_SPACE * (uint32_t)256 * (uint32_t)20)
	#define ENV_RESET_SPEED 200 //ENV_RESET_SPEED越大复位速度越慢	
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
	uint32_t uplimit_count_time[MOTION_COUNT];
	uint32_t downlimit_count_time[MOTION_COUNT]; 
	uint8_t protection_up_flag[MOTION_COUNT];
	uint8_t protection_down_flag[MOTION_COUNT];
	uint8_t record_point_position[MOTION_COUNT];
};

struct shark
{
	uint8_t am;   //振幅
	uint8_t real_am_high[3];
	uint8_t period;
	uint8_t shark_flag;
	uint8_t time_count;
	uint8_t rec_high_flag;
	uint8_t route_flag;
	uint8_t motion_dir[3];
	uint8_t error_high[3];
	uint8_t mark_before_am;
	uint8_t mark_befor_period;
};
extern struct shark shark;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6; 
extern CAN_HandleTypeDef hcan;
extern struct motion_status motion[MOTION_COUNT];
extern struct status status;

#endif /* __USER_CONFIG_H */
