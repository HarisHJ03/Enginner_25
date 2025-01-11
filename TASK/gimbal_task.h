#ifndef _gimbal_task_H
#define _gimbal_task_H


#include "stm32f4xx.h"
#include "ramp.h"

typedef struct
{
  /* Ïà¶Ô½Ç¶È£¬ÍÓÂÝÒÇÅ·À­½Ç */
  float pit_relative_angle;
  float yaw_relative_angle;
  float pit_gyro_angle;
  float yaw_gyro_angle;
  /* ½ÇËÙ¶È */
  float yaw_palstance;
  float pit_palstance;
} gim_sensor_t;

typedef struct
{
  uint16_t PIT_ONE_REF;//¸©Ñö
	uint16_t YAW_ONE_REF;//Æ«º½
	
 	uint16_t YAW_TWO_REF;//Æ«º½
	uint16_t PIT_TWO_REF;//¸©Ñö
	
	int16_t pit_v;
	int16_t pit1_v;
	gim_sensor_t sensor;
//	uint16_t yaw_v;//½ö¿ØÖÆ¸©Ñö
	
}gimbal_t;




extern gimbal_t gimbal;

void gimbal_task(void *parm);
void gimbal_param_init(void);

static void gimbal_nomarl_handler(void);
static void gimbal_engineer_handler(void);
static void gimbal_release_handler(void);

#endif

