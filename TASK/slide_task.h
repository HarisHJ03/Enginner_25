#ifndef _slide_task_H
#define _slide_task_H

#include "stm32f4xx.h"
#include "ramp.h"
#include "User.h"

/*****伸出量标志位*****/
#define INITIAL 0
#define SMALL		1
#define MEDIUM	2	
#define LARGE		3

#define FRONT_MODE	0
#define SIDE_MODE		1

typedef enum
{
	SLIDING,
	SLIDED,
	SLIDE_INIT,
}slide_flag_t;

typedef struct
{
	uint8_t init_flag;
	uint8_t state;
	uint8_t last_state;
	
	slide_flag_t slide_flag;
	
	int16_t spd_ref[SLIDE_NUMBER];
	int16_t spd_fdb[SLIDE_NUMBER];
	
	int16_t current[SLIDE_NUMBER];
	
	int32_t angle_ref[SLIDE_NUMBER];
	int32_t angle_fdb[SLIDE_NUMBER];
	
	int32_t init_angle[SLIDE_NUMBER];
}slide_t;

extern slide_t slide;
extern uint8_t slide_angle_ctrl_state[SLIDE_NUMBER];
extern uint8_t slide_minimum_state;


void slide_motor_angle(void);
void slide_task(void *parm);
void slide_init_handler(void);
void slide_param_init(void);
void slide_change_angle_ref(int16_t slide_mode_angle,int16_t small,int16_t medium,int16_t large,uint8_t mode_flag);

#endif
