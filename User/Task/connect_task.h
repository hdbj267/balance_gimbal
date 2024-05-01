#ifndef CONNECT_TASK_H
#define CONNECT_TASK_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "can1_app.h"
#include "remote_app.h"
#include "can.h"


#define CONNECT_TASK_TIME_1MS                  (1)   
#define my_abs(x) ((x)>0? (x):(-(x)))

	typedef	union 
	{
		float aaa;
		uint8_t bbb[4];
	}communication_t;

typedef __packed struct 
{
	uint8_t robot_color;
	uint8_t robot_mode;
	uint8_t robot_id;
	int16_t 	gyro_x;
	int16_t   gyro_y;
	float 	yaw;
	float 	pitch;
	short   gyro_accle;
	uint8_t bullet_speed;
}_tx2_tx_data;	
	
typedef struct   //can2传输的rc数据
{
	uint8_t control_mode;
	uint8_t work_mode;
	struct 
	{
		int16_t ch2;
		int16_t ch3;
	}rc;
	struct
	{
		int16_t key;
	}mouse;

}can2_rc_ctrl_t;


typedef struct
{
	RC_ctrl_t *rc_ctrl;
	
	can2_rc_ctrl_t can2_rc_ctrl;
	uint8_t receive_success_flag;
	
	uint16_t cm1_encode;
	uint16_t cm2_encode;
	uint16_t cm3_encode;
	uint16_t cm4_encode;
	
	int16_t cm1_rate;
	int16_t cm2_rate;
	int16_t cm3_rate;
	int16_t cm4_rate;
	
}connect_t;

extern connect_t connect_data;

void send_gyro_data_to_chassis(void); 

connect_t *get_connect_data_point(void);
#endif





