#ifndef info_get_task_H
#define info_get_task_H

/*3508�ļ��ٱ� */
#define DECELE_RATIO_3508 (19.0f/1.0f)
/*����3508  */
#define DECELE_RATIO_3508_2 (51.0f/1.0f)
/*2006�ļ��ٱ�*/
#define DECELE_RATIO_2006 (36.0f/1.0f)
/* �������ֵ �� �Ƕȣ��ȣ� �ı��� */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)



#include "stm32f4xx.h"
#include "motor_task.h"

void info_get_task(void *parm);

//static void get_gimbal_info(void);
static void get_chassis_info(void);
static void get_rescue_info(void);
static void get_clamp_info(void);
static void get_upraise_info(void);
static void get_slide_info(void);
static void get_barrier_carry_info(void);
static void get_structure_param(void);
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);
static void get_global_last_info(void);
static void get_manipulator_info(void);
int16_t judge_motor_type(motor_t* Motor);
static void Get_Motor_info(void);

#endif
