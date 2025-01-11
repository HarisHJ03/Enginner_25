#ifndef _chassis_task_H
#define _chassis_task_H

#define MODEL_TO_XANGLE_RATIO 1

#define ANGLE_RATIO_FR 1
#define ANGLE_RATIO_FL 1
#define ANGLE_RATIO_BR 1
#define ANGLE_RATIO_BL 1

#define NOT_AUTO_TURN 1

#include "stm32f4xx.h"

typedef struct
{
  uint8_t         turn_back;
  float           vx; // forward/back
  float           vy; // left/right
  float           vw; // 
  int16_t         rotate_x_offset;
  int16_t         rotate_y_offset;
  
  int16_t         wheel_spd_fdb[4];
  int16_t         wheel_spd_ref[4];
	int16_t         wheel_angle_fdb[4];
  int16_t         wheel_angle_ref[4];
	int16_t         wheel_angle_offset[4];
  int16_t         current[4];
  
  int16_t         turn_back_angle;  //记录转头前的总编码角度
  
}chassis_t;

typedef enum
{
	AHEAD_FRONT=0,
	AHEAD_BACK,
}Direction_t;


extern chassis_t chassis;
extern uint8_t chassis_exchange_flag;
extern uint8_t chassis_exchange_action;
/*小猫步*/


extern uint32_t chassis_exchange_action_time;
//extern uint8_t  chassis_push_the_ore_down_done_state;    //兑换矿石时，已将其推落标志位。为1已推落
extern uint8_t exchange_ctrl_chassis_state;
extern int16_t exchange_chassis_spd;


void chassis_task(void *parm);
void chassis_param_init(void);

static void chassis_ahead_to_back(void);
static void chassis_ahead_to_front(void);
static void chassis_engineer_handler(void);
static void chassis_exchange_handler(void);
static void chassis_exchange_angle_handler(void);

static void chassis_small_auto_mode_handler(void);//小资源岛视觉辅助连夹三箱
static void chassis_auto_clamp_one_mode_handler(void);//大/小资源岛视觉辅助夹取一箱
//static void chassis_clamp_ordinary_mode_handler(void);
//static void chassis_big_island_move_andler(int left_or_right);

static void chassis_dodge_handler(void);
static void chassis_stop_handler(void);
static void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);

#endif
