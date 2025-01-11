#ifndef _comm_8010_task_H
#define _comm_8010_task_H

#include "stm32f4xx.h"
#include "User.h"
#include "motor_task.h"

void coom_8010_task(void *parm);
void go8010_task_1(void);
void go8010_task_2(void);
static void get_relative(motor_t* Motor,int16_t want_speed,int16_t final_speed[]);
void Centering_task(void);
void go_8010_test_tesk1(int id,float T,float Pos,float W,float K_P,float K_W);
#endif 
