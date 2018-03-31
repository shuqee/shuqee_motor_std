#include "stm32f1xx_hal.h"
#include "user_io.h"
#include "user_can.h"
#include "string.h"

typedef void(*p_fun_t)(void);

typedef enum 
{
	STATUS_MSG_ID = 0x0,
	
	HIGHT_MSG_ID = 0x100,    //高度ID
	SPEED_MSG_ID = 0x101,		 //速度ID
	SP_MSG_ID  = 0x102, //特效ID	
	
	NM_MSG_ID = 0x400,
} can_msg_id_t;

typedef enum 
{
	HIGHT_MSG=0,  //高度ID
	SPEED_MSG,					//速度ID
	SP_MSG,					  //特效ID	
} can_rx_msg_t;

typedef struct 
{
	uint8_t data[8];
} can_tx_msg_data_t;

typedef enum 
{
	NM_MSG = 0,
	STATUS_MSG,
	CAN_TX_MAX_NUM
} can_tx_msg_t;

can_tx_msg_data_t can_tx_buf[CAN_TX_MAX_NUM] = {0};

typedef struct
{
	can_msg_id_t msg_id;
	uint8_t cycle;
	uint8_t count;
	p_fun_t can_tx;
} can_tx_item_t;

void can_tx_handle(void);

static void can_tx_nm(void)
{
	//send STATUS_MSG_ID into can bus
	LED_UP_LIMIT2_TOGGLE();
	can_send(NM_MSG_ID, can_tx_buf[NM_MSG].data, 8);
}

static void can_tx_status(void)
{
	//send NM_MSG_ID into can bus
	LED_UP_LIMIT3_TOGGLE();
	can_send(STATUS_MSG_ID, can_tx_buf[STATUS_MSG].data, 8);
}

void set_status_msg(uint8_t *tx_data)
{
	memcpy(can_tx_buf[STATUS_MSG].data, tx_data, 8);
}

void set_nm_msg(uint8_t *tx_data)
{
	memcpy(can_tx_buf[NM_MSG].data, tx_data, 8);
}

can_tx_item_t can_tx_table[CAN_TX_MAX_NUM];

can_msg_id_t tx_msg_id_maping[CAN_TX_MAX_NUM] =
{
	NM_MSG_ID,
	STATUS_MSG_ID
};

uint8_t tx_msg_cycle_init[CAN_TX_MAX_NUM] =
{
	50, /* 0 NM_MSG:500ms */
	5 /* 1 STATUS_MSG:50ms */
};

p_fun_t can_tx_fun_init[CAN_TX_MAX_NUM] =
{
	&can_tx_nm, /* 0 NM_MSG */
	&can_tx_status /* 1 STATUS_MSG */
};

void can_tx_servet_init(void)
{
	can_tx_msg_t index = 0;
	for (index = 0; index < CAN_TX_MAX_NUM; index++)
	{
		can_tx_table[index].msg_id = tx_msg_id_maping[index];
		can_tx_table[index].cycle = tx_msg_cycle_init[index];
		can_tx_table[index].count = 0;
		can_tx_table[index].can_tx = can_tx_fun_init[index];
	}
}

void task_can_tx(void)
{
	LED_UP_LIMIT1_TOGGLE();
	can_tx_handle();
}

void can_tx_handle(void)
{
	can_tx_msg_t index = 0;
	for (index = 0; index < CAN_TX_MAX_NUM; index++)
	{
		if (can_tx_table[index].count == 0)
		{
			can_tx_table[index].count = can_tx_table[index].cycle;
			can_tx_table[index].can_tx();
		}
		if (can_tx_table[index].count > 0)
		{
			can_tx_table[index].count--;
		}
	}
}



