#ifndef _barrier_carry_task_H
#define _barrier_carry_task_H

#include "stm32f4xx.h"

#define BARRIER_ON      GPIO_SetBits(GPIOC,GPIO_Pin_3);
#define BARRIER_OFF     GPIO_ResetBits(GPIOC,GPIO_Pin_3);
typedef enum 
{
	
	BARRIER_CARRYED = 0,
	BARRIER_CARRYING,//正在搬运
	
}barrier_carry_flag_t;

typedef struct
{
	uint8_t flag;
  uint8_t state;//记录是否初始化
  uint8_t last_state;
  
  uint8_t barrier_carry_cmd;
	
	barrier_carry_flag_t barrier_carrry_flag;
	
	uint8_t upraise_updown_flag;
  
  int16_t spd_ref[2];
  int16_t spd_fdb[2];
	
	int16_t current[2];//发送电流值
  
  int32_t angle_ref[2];
  int32_t angle_fdb[2];
  
  int32_t init_angle[2];//初始化角度
	
	int32_t rc_ctrl_angle;//通过遥控控制对位/放下高度
	
	
}barrier_carry_t;

extern barrier_carry_t barrier_carry;
extern uint32_t barrier_carry_angle_ref;
extern uint8_t  barrier_carry_init_state  ;
extern uint8_t barrier_time;
extern uint8_t barrier_angle_switch;
extern uint8_t barrier_rescue_angle;
void barrier_carry_task(void *parm);
void barrier_carry_param_init(void);

void barrier_carry_init_handler(void);
void barrier_carry_enable_handler(void);
void barrier_carry_power_on_initialization(void);
#endif


