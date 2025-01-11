#ifndef _keyboard_H
#define _keyboard_H

#include "stm32f4xx.h"
#include "User.h"

#define kb_chassis_control_flag 0

#ifdef kb_chassis_control_flag

#define FORWARD    (rc.kb.bit.W)
#define BACK       (rc.kb.bit.S)
#define LEFT       (rc.kb.bit.A)
#define RIGHT      (rc.kb.bit.D)

#endif

#define KB_SLIDE_FORWARD (!rc.kb.bit.SHIFT && rc.kb.bit.W)
#define KB_SLIDE_BACK (!rc.kb.bit.SHIFT && rc.kb.bit.S)
#define KB_TRAVERSE_LEFT (!rc.kb.bit.SHIFT && rc.kb.bit.A)
#define KB_TRAVERSE_RIGHT (!rc.kb.bit.SHIFT && rc.kb.bit.D)
#define KB_UPRAISE_UP (!rc.kb.bit.SHIFT && rc.kb.bit.F)
#define KB_UPRAISE_DN (!rc.kb.bit.SHIFT && rc.kb.bit.C)

#define KB_YAW_LEFT (rc.kb.bit.SHIFT && rc.kb.bit.A)
#define KB_YAW_RIGHT (rc.kb.bit.SHIFT && rc.kb.bit.D)
#define KB_PITCH_UP (rc.kb.bit.SHIFT && rc.kb.bit.W)
#define KB_PITCH_DN (rc.kb.bit.SHIFT && rc.kb.bit.S)
#define KB_ROLL_LEFT (rc.kb.bit.SHIFT && rc.kb.bit.Q)
#define KB_ROLL_RIGHT (rc.kb.bit.SHIFT && rc.kb.bit.E)

#define KB_CLAMP_SLIDE_DEL (rc.kb.bit.B)
/*兑换箱数人为更改*/
#define KB_HAVE_BOX_DEC (rc.kb.bit.Z && rc.kb.bit.CTRL) // 已夹取零箱
#define KB_HAVE_BOX_INC (rc.kb.bit.X && rc.kb.bit.CTRL) // 已夹取一箱
// #define KB_HAVE_CLAMP_TWO     (!rc.kb.bit.SHIFT && rc.kb.bit.C && rc.kb.bit.CTRL) //已夹取两箱


#define KB_RESET_INIT (rc.kb.bit.R && rc.kb.bit.CTRL && rc.kb.bit.SHIFT) // 软件初始化
/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
// #define W 			0x0001		//bit 0
// #define S 			0x0002
// #define A 			0x0004
// #define D 			0x0008
// #define SHIFT 	0x0010
// #define CTRL 		0x0020
// #define Q 			0x0040
// #define E				0x0080
// #define R 			0x0100
// #define F 			0x0200
// #define G 			0x0400
// #define Z 			0x0800
// #define X 			0x1000
// #define C 			0x2000
// #define V 			0x4000
// #define B				0x8000	//bit 15
/******************************************************/

typedef enum
{
  MOUSE_RELEASE,
  MOUSE_PRESS,
  MOUSE_DONE,
  MOUSE_ONCE,
  MOUSE_LONG,
} MOUSE_STATUS;

typedef struct
{
  MOUSE_STATUS l_mouse_sta;
  MOUSE_STATUS r_mouse_sta;
  uint16_t l_cnt;
  uint16_t r_cnt;

  uint8_t kb_enable;

  float vx_limit_speed;
  float vy_limit_speed;
  float vw_limit_speed;

  /*底盘方向*/
  float vx;
  float vy;
  float vw;
  /*云台方向*/
  float pit_v;
  float yaw_v;

} kb_ctrl_t;

typedef struct
{
  int16_t angle;
  uint32_t delay_times;
} kb_adjust_t;

extern kb_ctrl_t km;

extern int big_island_chassis_move;                   // 在大资源岛夹取时，控制底盘向左还是向右及移动距离大小
extern uint8_t big_island_chassis_move_done;          // 为1时，证明已经移动到指定位置
extern uint8_t KB_BIG_ISLAND_AUTO_SINGLE_CLAMP_STATE; // 键盘优先级高于遥控（仅大资源岛夹取，不代表所有）
extern uint8_t keyborad_rescue_view_flag;

////////////////////删////////////////////////
extern int16_t kb_upraise_angle[2];
extern int16_t kb_slide_angle[2];
extern int16_t kb_exchange_angle;
extern int16_t kb_manipulator_adjust_angle[7]; // 机械臂电机调节角度
extern uint8_t looking_screen_flag;
////////////////////删////////////////////////

extern kb_adjust_t kb_adrust[6];
extern kb_adjust_t kb_slide_clamp_del;
void keyboard_global_hook(void);
void keyboard_chassis_hook(void);
void keyboard_clamp_hook(void);
void keyboard_supply_hook(void);
void keyboard_barrier_carry_hook(void);
void keyboard_exchange_hook(void);
void keyboard_ore_adjust_hook(void);
void keyboard_init_hook(void);
void kb_adrust_angle_ctrl(void);
#endif
