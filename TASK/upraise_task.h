#ifndef _upraise_task_H
#define _upraise_task_H


#include "stm32f4xx.h"

//抬升使用按键压触则置1，否则堵转置零
#define UPRISE_KET_MODE 1

/*****抬升量标志位*****/
#define INITIAL 0
#define SMALL		1
#define MEDIUM	2	
#define LARGE		3

#define UPRISE_UP +
#define ANGLE_HIGHT 2100/220

typedef enum
{
  DOWN = 0,
  UP,
  FALL,
  RAISE,
  UPRAISE_INIT,
}updown_flag_t;

typedef struct
{
  uint8_t flag;
  uint8_t state;
  
  updown_flag_t updown_flag;
  
  int16_t spd_ref[2];
  int16_t spd_fdb[2];
	
	int16_t current[2];
  
  int32_t angle_ref[2];
  int32_t angle_fdb[2];
  
  int32_t init_angle[2];
    
}upraise_t;

extern upraise_t upraise;
extern uint8_t upraise_angle_ctrl_state[2] ;
extern uint8_t upraise_minimum_state;

void upraise_motor_angle(void);
void upraise_task(void *parm);
void upraise_init_handler(void);
void upraise_param_init(void);
void upraise_change_angle_ref(int16_t upraise_mode_angle,int16_t small,int16_t medium,int16_t large);

#endif

