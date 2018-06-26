/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "user_config.h"
#include "user_io.h"
#include "user_time.h"
#include "user_uart.h"
#include "user_can.h"
#include "sw_timer.h"

#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

struct motion_status motion[MOTION_COUNT] = {MOTION1};
struct status status = {0};
int flag_rst = 0;	//reset flag


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void exti_interrupt_filter(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void delay_ns(uint32_t times)
{
	while(times--);
}

void delay_us(uint32_t times)
{
	int i;
	uint32_t count;
	for(i=0; i<10; i++)
	{
		count = times;
		while(count--);
	}
}

void delay_ms(uint32_t times)
{
	uint32_t count;
	count = times;
	while(count--)
		delay_us(1000);
}

/*�ⲿ�ж�����ģ��*/
void exti_mask(void)
{
	EXTI->IMR &=~(1<<1);  //1�ж�����ģ
	EXTI->IMR &=~(1<<3);	 //3�ж�����ģ
	EXTI->IMR &=~(1<<5);  //5�ж�����ģ
}	

/*�ⲿ�жϿ���ģ��*/
void exti_open(void)
{
	EXTI->IMR |=1<<1;  //1��������ģ
	EXTI->IMR |=1<<3;	 //3��������ģ
	EXTI->IMR |=1<<5;  //5��������ģ
}

#ifdef ENV_RESET
void find_origin(void) /* reset function */
{
	enum motion_num i;
	uint8_t downlimit_temp = 0;
	int def_high[MOTION_COUNT] = {0};
	int find_origin_step[MOTION_COUNT] = {0};
	uint8_t not_reset_num = 0; /* δ��λ�׵����� */
	for(i=MOTION1; i<MOTION_COUNT; i++)
	flag_rst |= 1<<i; /* ��ʼ����λ��־(�׶�Ӧλ��ʼֵΪ1,��λ��׶�ӦλΪ0) */
#ifndef MOTION1_ENABLE
	flag_rst &= ~(1<<MOTION1);
#endif
#ifndef MOTION2_ENABLE
	flag_rst &= ~(1<<MOTION2);
#endif
#ifndef MOTION3_ENABLE
	flag_rst &= ~(1<<MOTION3);
#endif
	while(flag_rst)	/* ���и�δ��λ */
	{
		not_reset_num = 0;
		for(i=MOTION1; i<MOTION_COUNT; i++)
		{
			if((flag_rst&(1<<i)) != 0) /* δ��λ */
			{
				not_reset_num += 1;
			}
		}
		for(i=MOTION1; i<MOTION_COUNT; i++)
		{
			if((flag_rst&(1<<i)) != 0) /* δ��λ */
			{
				SAFE(downlimit_temp = status.downlimit[i]);
				switch (find_origin_step[i])
				{
					case 0:
						if (i == MOTION1)
						{
							LED_SEAT1(1);
							LED_SEAT2(1);
						}
						if (i == MOTION3)
						{
							LED_SEAT3(1);
							LED_SEAT4(1);
						}
						if (downlimit_temp == 0) /* ��δ���� */
						{
							HAL_GPIO_WritePin(motion[i].io.ndown_port, motion[i].io.ndown_pin, GPIO_PIN_SET); /* �����½� */
							set_pul(i, (GPIO_PinState)1, (ENV_RESET_SPEED*MOTION_COUNT/not_reset_num), 1); /* �����˶� */
						}
						else
						{
							++find_origin_step[i];
						}
						break;
					case 1:
						if (i == MOTION1)
						{
							LED_SEAT1(0);
							LED_SEAT2(1);
						}
						if (i == MOTION3)
						{
							LED_SEAT3(0);
							LED_SEAT4(1);
						}
						if (downlimit_temp == 1) /* �׵��� */
						{
							HAL_GPIO_WritePin(motion[i].io.nup_port, motion[i].io.nup_pin, GPIO_PIN_SET); /* �������� */
							set_pul(i, (GPIO_PinState)0, (ENV_RESET_SPEED*MOTION_COUNT/not_reset_num), 1); /* �����˶� */
						}
						else
						{
							++find_origin_step[i];
						}
						break;
					case 2:
						if (i == MOTION1)
						{
							LED_SEAT1(1);
							LED_SEAT2(0);
						}
						if (i == MOTION3)
						{
							LED_SEAT3(1);
							LED_SEAT4(0);
						}
						if (downlimit_temp == 0) /* ��δ���� */
						{
							HAL_GPIO_WritePin(motion[i].io.ndown_port, motion[i].io.ndown_pin, GPIO_PIN_SET); /* �����½� */
							set_pul(i, (GPIO_PinState)1, (ENV_RESET_SPEED*MOTION_COUNT/not_reset_num), 1); /* �����˶� */
						}
						else
						{
							HAL_GPIO_WritePin(motion[i].io.nup_port, motion[i].io.nup_pin, GPIO_PIN_SET); /* �������� */
							++find_origin_step[i];
						}
						break;
					case 3:
						if (i == MOTION1)
						{
							LED_SEAT1(0);
							LED_SEAT2(0);
						}
						if (i == MOTION3)
						{
							LED_SEAT3(0);
							LED_SEAT4(0);
						}
						if (motion[i].config.adj == 0) /* ����ҪУ�� */
							flag_rst &= ~(1<<i); /* ��־��λ��� */
						else
						{
							def_high[i] = motion[i].config.adj * ENV_SPACE;	/* ��ʼ��У���߶� */
							++find_origin_step[i];
						}
						break;
					case 4:
						if (i == MOTION1)
						{
							LED_SEAT1(0);
							LED_SEAT2(0);
						}
						if (i == MOTION3)
						{
							LED_SEAT3(0);
							LED_SEAT4(0);
						}
						if(def_high[i] != 0)
						{
							set_pul(i, (GPIO_PinState)0, (ENV_RESET_SPEED*MOTION_COUNT/not_reset_num), 1); /* �����˶� */
							def_high[i]--;
							if(def_high[i] == 0) /* �˶���ָ��λ�� */
							{
								flag_rst &= ~(1<<i); /* ��־��λ��� */
							}
						}
						break;
					default:
						find_origin_step[i] = 0;
						break;
				}
			}
		}
#ifdef ENV_IWDG
		HAL_IWDG_Refresh(&hiwdg); /* have to refresh the iwdg */
#endif
#ifdef DIRNA		
		exti_interrupt_filter();
#endif		
	}
}
#endif

void exchange_nup_ndown(enum motion_num index)
{
	GPIO_TypeDef * temp_port;
	uint16_t temp_pin;
	temp_port = motion[index].io.nup_port;
	temp_pin = motion[index].io.nup_pin;
	motion[index].io.nup_port = motion[index].io.ndown_port;
	motion[index].io.nup_pin = motion[index].io.ndown_pin;
	motion[index].io.ndown_port = temp_port;
	motion[index].io.ndown_pin = temp_pin;
}

void user_motion_init(void)
{
	enum motion_num i;
	
	motion[MOTION1].io.dir_port = OUTPUT_DIR1_GPIO_Port;
	motion[MOTION1].io.dir_pin = OUTPUT_DIR1_Pin;
	motion[MOTION1].io.pul_port = OUTPUT_PUL1_GPIO_Port;
	motion[MOTION1].io.pul_pin = OUTPUT_PUL1_Pin;
	motion[MOTION1].io.nup_port = OUTPUT_NUP1_GPIO_Port;
	motion[MOTION1].io.nup_pin = OUTPUT_NUP1_Pin;
	motion[MOTION1].io.ndown_port = OUTPUT_NDOWN1_GPIO_Port;
	motion[MOTION1].io.ndown_pin = OUTPUT_NDOWN1_Pin;
	motion[MOTION2].io.dir_port = OUTPUT_DIR2_GPIO_Port;
	motion[MOTION2].io.dir_pin = OUTPUT_DIR2_Pin;
	motion[MOTION2].io.pul_port = OUTPUT_PUL2_GPIO_Port;
	motion[MOTION2].io.pul_pin = OUTPUT_PUL2_Pin;
	motion[MOTION2].io.nup_port = OUTPUT_NUP2_GPIO_Port;
	motion[MOTION2].io.nup_pin = OUTPUT_NUP2_Pin;
	motion[MOTION2].io.ndown_port = OUTPUT_NDOWN2_GPIO_Port;
	motion[MOTION2].io.ndown_pin = OUTPUT_NDOWN2_Pin;
	motion[MOTION3].io.dir_port = OUTPUT_DIR3_GPIO_Port;
	motion[MOTION3].io.dir_pin = OUTPUT_DIR3_Pin;
	motion[MOTION3].io.pul_port = OUTPUT_PUL3_GPIO_Port;
	motion[MOTION3].io.pul_pin = OUTPUT_PUL3_Pin;
	motion[MOTION3].io.nup_port = OUTPUT_NUP3_GPIO_Port;
	motion[MOTION3].io.nup_pin = OUTPUT_NUP3_Pin;
	motion[MOTION3].io.ndown_port = OUTPUT_NDOWN3_GPIO_Port;
	motion[MOTION3].io.ndown_pin = OUTPUT_NDOWN3_Pin;
	
	motion[MOTION1].config.dir = MOTION1_CONFIG_DIR;
	motion[MOTION2].config.dir = MOTION2_CONFIG_DIR;
	motion[MOTION3].config.dir = MOTION3_CONFIG_DIR;
	
	motion[MOTION1].config.origin = MOTION1_CONFIG_ORIGIN;
	motion[MOTION2].config.origin = MOTION2_CONFIG_ORIGIN;
	motion[MOTION3].config.origin = MOTION3_CONFIG_ORIGIN;
	
	motion[MOTION1].config.adj = MOTION1_CONFIG_ADJ;
	motion[MOTION2].config.adj = MOTION2_CONFIG_ADJ;
	motion[MOTION3].config.adj = MOTION3_CONFIG_ADJ;
	
	for (i=MOTION1; i<MOTION_COUNT; i++)
	{
		motion[i].index = i;
		motion[i].high.set = motion[i].config.origin * ENV_SPACE;
		if (motion[i].config.dir == GPIO_PIN_SET) /* ������巽��ȡ�� */
			exchange_nup_ndown(i); /* ����ת��ֹ��Ӧ����ȡ�� */
	}
#ifdef ENV_RESET
	find_origin();
#endif
}

#ifdef ENV_NOSENSOR
void free_ndown(void)
{
	enum motion_num i;
	for (i=MOTION1; i<MOTION_COUNT; i++)
	{
		if (motion[i].high.now >= 0 * ENV_SPACE)
			HAL_GPIO_WritePin(motion[i].io.ndown_port, motion[i].io.ndown_pin, GPIO_PIN_SET); /* �����½� */
	}
}
void free_nup(void)
{
	enum motion_num i;
	for (i=MOTION1; i<MOTION_COUNT; i++)
	{
		if (motion[i].high.now <= 255 * ENV_SPACE)
			HAL_GPIO_WritePin(motion[i].io.nup_port, motion[i].io.nup_pin, GPIO_PIN_SET); /* �������� */
	}
}
#else
void free_ndown(void)
{
	enum motion_num i;
	for (i=MOTION1; i<MOTION_COUNT; i++)
	{
		if (status.downlimit[i] == 0)
			HAL_GPIO_WritePin(motion[i].io.ndown_port, motion[i].io.ndown_pin, GPIO_PIN_SET); /* �����½� */
	}
}
void free_nup(void)
{
	enum motion_num i;
	for (i=MOTION1; i<MOTION_COUNT; i++)
	{
		if (status.uplimit[i] == 0)
			HAL_GPIO_WritePin(motion[i].io.nup_port, motion[i].io.nup_pin, GPIO_PIN_SET); /* �������� */
	}
}
#endif

#ifdef DIRNA
uint16_t recognize_limit_time(enum motion_num index)
{
	if((motion[index].high.now<=0x10)&&(motion[index].high.now>=0)&&(motion[index].dir==(GPIO_PinState)1))  //0x00-0x10
	{
		return RESET_FILTER_TIME;
	}	
	if((motion[index].high.now<=0xff)&&(motion[index].high.now>=0xef)&&(motion[index].dir==(GPIO_PinState)0))  //0xef-0xff
	{
		return RESET_FILTER_TIME;
	}	
	if((motion[index].high.now<0xef)&&(motion[index].high.now>0x10))  //0x00-0x10
	{
		return RUN_TILTER_TIME;
	}	
 return RUN_TILTER_TIME;	
}	
void exti_interrupt_filter(void)
{  
	enum motion_num index;
	uint32_t temp;
	for(index=MOTION1;index<MOTION_COUNT;index++)
	{
		if(flag_rst )//��λ��
		{
			if(status.uplimit[index]==1)
			{
				if (HAL_GetTick() - status.uplimit_count_time[index]>=RESET_FILTER_TIME)
				{
					if(index==MOTION2)	//������
					Uplimit1(1);				//������
				}
			}	
			else
			{
				status.uplimit_count_time[index]=  HAL_GetTick();				
			}
			if(status.downlimit[index]==1)
			{
				if (HAL_GetTick() - status.downlimit_count_time[index]>=RESET_FILTER_TIME)
				{
					if(index==MOTION2)	//������
					Downlimit1(1);			//������
				}
			}	
			else
			{
				status.downlimit_count_time[index]=  HAL_GetTick();
			}	
		}
	if(flag_rst==0 )//��������;
	{
		if(status.uplimit[index]==1)
		{
			if(HAL_GetTick()<recognize_limit_time(index))  //ȷ���δ�ʱ���Ƿ�����ʱ��LIMIT
			{ 
				status.protection_up_flag[index]=1; 
			}
			if (HAL_GetTick() - status.uplimit_count_time[index]>=recognize_limit_time(index))
			{
				if(status.protection_up_flag[index])
				{
					temp=status.uplimit_count_time[index];
					status.uplimit_count_time[index]=  HAL_GetTick()-temp;
					status.protection_up_flag[index]=0;  
					continue;
				}	
				if(index==MOTION2)		//������
				Uplimit2(1);					//������
				up_limit(index);
			}
		}	
		else
		{
			status.uplimit_count_time[index]=  HAL_GetTick();
		}
		if(status.downlimit[index]==1)
		{
			if(HAL_GetTick()<recognize_limit_time(index))  //ȷ���δ�ʱ���Ƿ�����ʱ��LIMIT
			{
				status.protection_down_flag[index]=1;
			}			
			if (HAL_GetTick() - status.downlimit_count_time[index]>=recognize_limit_time(index))
			{
				if(status.protection_down_flag[index])
				{
					temp=status.uplimit_count_time[index];
					status.uplimit_count_time[index]=  HAL_GetTick()-temp;
					status.protection_down_flag[index]=0;					
					continue;
				}					
				if(index==MOTION2)  //������
				Downlimit2(1);	    //������
				down_limit(index);				
			}
		}	
		else
		{
			status.downlimit_count_time[index]=  HAL_GetTick();
		}		
	}
 }	
}
#endif
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
	static int led_count = 0;
#ifdef ENV_SEAT_PICKING
	static uint8_t seat_picking = 0;
#endif
#ifdef ENV_SEND_SEAT_INFO
	static uint8_t send_seat = 0;
	static uint8_t send_buf[4] = {0xff,0xc1}; /* �ظ�֡ͷ */
	static int send_index = 0;
#endif
	 /* �������ݸ��±�־ */
	uint8_t update_485;
	uint8_t update_can;
	uint8_t init_flag = 0; /* ��ʼ����׼λ */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#ifdef ENV_IWDG
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();	
  MX_CAN_Init();

  /* USER CODE BEGIN 2 */
#else

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
	MX_CAN_Init();
  /* MX_IWDG_Init(); */

#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  
	memset((void *)motion, 0 ,sizeof(motion));
	status.id = 0;
	status.seat_num = 0;
	status.seat_enable = 0;
	status.spb = 0;
	led_count = 0;
#ifdef ENV_SEAT_PICKING
	seat_picking = 0;
#endif
#ifdef ENV_SEND_SEAT_INFO
	send_seat = 0;
	send_index = 0;
#endif
	user_io_init();
	user_motion_init(); 
  user_adc_start();
	user_time_init();
	user_uart_init();
	user_can_init();
	sw_timer_init(); 
	init_flag = 1;
	while (init_flag != 0)
	{
#ifdef DIRNA		
		exti_interrupt_filter();
#endif		
		sw_timer_handle();	
#ifdef ENV_IWDG
		HAL_IWDG_Refresh(&hiwdg);
#endif
		status.seat_enable = GET_SEAT_ENABLE();
		SAFE(status.seat_enable += status.seat_num);
		SAFE(update_485 = frame.enable);
		SAFE(free_ndown());
		SAFE(free_nup());
    SAFE(update_can=get_update_flag()); 
		if((update_485)||(update_can)) /* �������ݸ��� */
		{
			SAFE(frame.enable = 0);
			SAFE(clr_update_flag()); 
			/*LED_START*/
			led_count++;
			led_count = led_count%10;
			if(led_count == 0)
			{
				LED_TOGGLE(); /* ��˸ָʾ�� */
			}
			/*LED_END*/
			/*SEAT_START*/
#ifdef ENV_SEAT_PICKING
			if (seat_picking == 0) /* ����δ��ѡ�� */
			{
				status.seat_enable = 0;
			}
#endif
///////////////////////////////////////////////////////////////////
			uint8_t msg_buff_l[8]={0};
			uint8_t a,b,c;
			get_high_speed_date(HIGHT_MSG_P,msg_buff_l);          //ʾ����
			get_high_speed_date(ENV_SP_P,&a);
			get_high_speed_date(SEAT_ID_P,&b);
			get_high_speed_date(SEAT_SP_P,&c);
      
			frame.buff[4]=msg_buff_l[0];
			frame.buff[3]=msg_buff_l[1];
			frame.buff[2]=msg_buff_l[2];
			
			frame.buff[5]=c;
			frame.buff[7]=b;
			////////////////////////////////////////////////////////////
		
			if(status.seat_enable) /* ����ʹ�� */
			{
				SAFE(status.spb = frame.buff[5]); /* ������Ч */
#ifdef MOTION1_ENABLE
				SAFE(motion[MOTION1].high.set = frame.buff[4] * ENV_SPACE); /* ����Ŀ��λ�� */
#else
				SAFE(motion[MOTION1].high.set = motion[MOTION1].config.origin * ENV_SPACE); /* �ָ�Ŀ��λ�� */
#endif
#ifdef MOTION2_ENABLE
				SAFE(motion[MOTION2].high.set = frame.buff[3] * ENV_SPACE); /* ����Ŀ��λ�� */
#else
				SAFE(motion[MOTION2].high.set = motion[MOTION2].config.origin * ENV_SPACE); /* �ָ�Ŀ��λ�� */
#endif
#ifdef MOTION3_ENABLE
				SAFE(motion[MOTION3].high.set = frame.buff[2] * ENV_SPACE); /* ����Ŀ��λ�� */
#else
				SAFE(motion[MOTION3].high.set = motion[MOTION3].config.origin * ENV_SPACE); /* �ָ�Ŀ��λ�� */
#endif
			}
			else
			{
				SAFE(motion[MOTION1].high.set = motion[MOTION1].config.origin * ENV_SPACE); /* �ָ�Ŀ��λ�� */
				SAFE(motion[MOTION2].high.set = motion[MOTION2].config.origin * ENV_SPACE); /* �ָ�Ŀ��λ�� */
				SAFE(motion[MOTION3].high.set = motion[MOTION3].config.origin * ENV_SPACE); /* �ָ�Ŀ��λ�� */
				SAFE(status.spb = frame.buff[5]); /* ������Ч */
#ifdef ENV_SWING_LINK
				SAFE(status.spb &= SPB_AIR_INJECTION_MASK); /* �ָ���Ч */
#else
				SAFE(status.spb = 0); /* �ָ���Ч */
#endif
			}
			status.id = 0; /* ����id */
			if(GET_ID_1())
				status.id = status.id + 1;
			if(GET_ID_2())
				status.id = status.id + 2;
			if(GET_ID_4())
				status.id = status.id + 4;
			if(GET_ID_8())
				status.id = status.id + 8;
			if(GET_ID_10())
				status.id = status.id + 10;
			if(GET_ID_20())
				status.id = status.id + 20;
			if(GET_ID_40())
				status.id = status.id + 40;
			if(GET_ID_80())
				status.id = status.id + 80;
#ifdef ENV_SEAT_PICKING
			if (frame.buff[7] == 0xAA) /* ѡ���������� */
			{
				seat_picking = 1;
			}
			else if (frame.buff[7] == 0x00) /* ȡ��ѡ���������� */
			{
				seat_picking = 0;
			}
			else if (frame.buff[7] == status.id) /* ����IDѡ������ */
			{
				seat_picking = 1;
			}
			else
			{
				; /* ά�ֵ�ǰѡ�л�δѡ��״̬ */
			}
#endif
#ifdef ENV_SEND_SEAT_INFO
			if(frame.buff[7] == status.id) /* �ж����α�� */
			{
				if (frame.buff[7] != 0x00)
				{
					send_seat = 1;
					send_buf[2] = status.id;
					SAFE(send_buf[3] = status.seat_num);
				}
			}
#endif
			/*SEAT_END*/
		}
		/*SEND_SEAT_START*/
#ifdef ENV_SEND_SEAT_INFO
		if(send_seat)
		{
			HAL_GPIO_WritePin(OUTPUT_485RW_GPIO_Port, OUTPUT_485RW_Pin, GPIO_PIN_RESET); /* 485���� */
			if(send_index == 0 || __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) != RESET)
			{
				huart1.Instance->DR = send_buf[send_index];
				send_index++;
			}
			if(send_index == 4)
			{
				send_index = 0;
				send_seat = 0;
			}
		}
		else
		{
			if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) != RESET)
			HAL_GPIO_WritePin(OUTPUT_485RW_GPIO_Port, OUTPUT_485RW_Pin, GPIO_PIN_SET); /* 485���� */
		}
#endif
		/*SEND_SEAT_END*/
		/*SPB_START*/
		SPB3(status.spb&(1<<2)); /* ������Ч��IO��� */
		SPB4(status.spb&(1<<3));
		SPB5(status.spb&(1<<4));
		SPB6(status.spb&(1<<5));
		SPB7(status.spb&(1<<6));
		SPB8(status.spb&(1<<7));
		/*SPB_END*/
		/*RST_START*/
		if (status.spb&0x01)
		{
			user_adc_stop();
			user_time_stop();
			user_uart_stop();
			init_flag = 0;
		}
		else
		{
			/*RST_END*/
			HAL_UART_Receive_IT(&huart1, (uint8_t *)&(frame.data), 1); /* ��ֹ���ڳ��� */
		}
	}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 5, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_5TQ;
  hcan.Init.BS2 = CAN_BS2_6TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OUTPUT_SEATLED5_Pin|OUTPUT_SEATLED4_Pin|OUTPUT_SEATLED3_Pin|OUTPUT_573LE1_Pin 
                          |OE_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OUTPUT_SEATLED2_Pin|OUTPUT_SEATLED1_Pin|OUTPUT_CLR1_Pin|OUTPUT_DIR3_Pin 
                          |OUTPUT_DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUTPUT_PUL1_Pin|OUTPUT_CLR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUTPUT_PUL2_Pin|OUTPUT_LED0_Pin|OUTPUT_LED1_Pin|OUTPUT_PUL3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Uplimit1_Pin|Downlimit1_Pin|Uplimit2_Pin|Downlimit2_Pin 
                          |Uplimit3_Pin|Downlimit3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OUTPUT_573LE2_Pin|OUTPUT_573LE3_Pin|OUTPUT_DIR1_Pin|OUTPUT_CLR3_Pin 
                          |OUTPUT_NUP3_Pin|OUTPUT_NDOWN3_Pin|OUTPUT_NUP2_Pin|OUTPUT_NDOWN2_Pin 
                          |OUTPUT_NUP1_Pin|OUTPUT_NDOWN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUTPUT_485RW_GPIO_Port, OUTPUT_485RW_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUTPUT_SP8_Pin|OUTPUT_SP7_Pin|OUTPUT_SP6_Pin|OUTPUT_SP5_Pin 
                          |OUTPUT_SP4_Pin|OUTPUT_SP3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : OUTPUT_SEATLED5_Pin OUTPUT_SEATLED4_Pin OUTPUT_SEATLED3_Pin OUTPUT_573LE1_Pin 
                           Uplimit1_Pin Downlimit1_Pin Uplimit2_Pin Downlimit2_Pin 
                           Uplimit3_Pin Downlimit3_Pin */
  GPIO_InitStruct.Pin = OUTPUT_SEATLED5_Pin|OUTPUT_SEATLED4_Pin|OUTPUT_SEATLED3_Pin|OUTPUT_573LE1_Pin 
                          |Uplimit1_Pin|Downlimit1_Pin|Uplimit2_Pin|Downlimit2_Pin 
                          |Uplimit3_Pin|Downlimit3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_SEATLED2_Pin OUTPUT_SEATLED1_Pin OUTPUT_CLR1_Pin OUTPUT_DIR3_Pin 
                           OUTPUT_DIR2_Pin */
  GPIO_InitStruct.Pin = OUTPUT_SEATLED2_Pin|OUTPUT_SEATLED1_Pin|OUTPUT_CLR1_Pin|OUTPUT_DIR3_Pin 
                          |OUTPUT_DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI_UPLIMIT1_Pin EXTI_DOWNLIMIT1_Pin EXTI_UPLIMIT2_Pin EXTI_DOWNLIMIT2_Pin 
                           EXTI_UPLIMIT3_Pin EXTI_DOWNLIMIT3_Pin */
  GPIO_InitStruct.Pin = EXTI_UPLIMIT1_Pin|EXTI_DOWNLIMIT1_Pin|EXTI_UPLIMIT2_Pin|EXTI_DOWNLIMIT2_Pin 
                          |EXTI_UPLIMIT3_Pin|EXTI_DOWNLIMIT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_PUL1_Pin OUTPUT_485RW_Pin OUTPUT_CLR2_Pin */
  GPIO_InitStruct.Pin = OUTPUT_PUL1_Pin|OUTPUT_485RW_Pin|OUTPUT_CLR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_PUL2_Pin OUTPUT_LED0_Pin OUTPUT_LED1_Pin OUTPUT_SP8_Pin 
                           OUTPUT_SP7_Pin OUTPUT_SP6_Pin OUTPUT_SP5_Pin OUTPUT_SP4_Pin 
                           OUTPUT_PUL3_Pin OUTPUT_SP3_Pin */
  GPIO_InitStruct.Pin = OUTPUT_PUL2_Pin|OUTPUT_LED0_Pin|OUTPUT_LED1_Pin|OUTPUT_SP8_Pin 
                          |OUTPUT_SP7_Pin|OUTPUT_SP6_Pin|OUTPUT_SP5_Pin|OUTPUT_SP4_Pin 
                          |OUTPUT_PUL3_Pin|OUTPUT_SP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT_SW_Pin */
  GPIO_InitStruct.Pin = INPUT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INPUT_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_573LE2_Pin OUTPUT_573LE3_Pin OUTPUT_DIR1_Pin OUTPUT_CLR3_Pin 
                           OUTPUT_NUP3_Pin OUTPUT_NDOWN3_Pin OUTPUT_NUP2_Pin OUTPUT_NDOWN2_Pin 
                           OUTPUT_NUP1_Pin OUTPUT_NDOWN1_Pin */
  GPIO_InitStruct.Pin = OUTPUT_573LE2_Pin|OUTPUT_573LE3_Pin|OUTPUT_DIR1_Pin|OUTPUT_CLR3_Pin 
                          |OUTPUT_NUP3_Pin|OUTPUT_NDOWN3_Pin|OUTPUT_NUP2_Pin|OUTPUT_NDOWN2_Pin 
                          |OUTPUT_NUP1_Pin|OUTPUT_NDOWN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_BCD1_1_Pin INPUT_BCD2_1_Pin INPUT_BCD4_1_Pin INPUT_BCD8_1_Pin */
  GPIO_InitStruct.Pin = INPUT_BCD1_1_Pin|INPUT_BCD2_1_Pin|INPUT_BCD4_1_Pin|INPUT_BCD8_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_BCD1_2_Pin INPUT_BCD2_2_Pin INPUT_BCD4_2_Pin INPUT_BCD8_2_Pin */
  GPIO_InitStruct.Pin = INPUT_BCD1_2_Pin|INPUT_BCD2_2_Pin|INPUT_BCD4_2_Pin|INPUT_BCD8_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OE_CE_Pin */
  GPIO_InitStruct.Pin = OE_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OE_CE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
