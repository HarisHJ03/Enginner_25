#ifndef _modeswitch_task_H
#define _modeswitch_task_H


#include "stm32f4xx.h"
#include "string.h"

#define MEMSET(flag,type) (memset((type*)flag,0,sizeof(type)))

typedef enum
{
  INIT_NEVER = 0,
  INIT_DONE,
}global_state;

typedef enum
{
  CLAMP_VIEW = 0, //矿石夹取视野
  ODJIUST_VIEW,   //姿态调整视�?
}view_switch_t;

/*主要控制模式*/
typedef enum
{
  RELEASE_CTRL,
  MANUAL_CTRL,
//  SEMI_AUTOMATIC_CTRL,     //半自动控�?
  ENGINEER_CTRL,           //夹取，救援，障碍块搬运都基于工程模式，即底盘在任何模式都是正常使用的
}global_status;
/*云台控制模式*/
typedef enum
{
  GIMBAL_RELEASE = 0,
  GIMBAL_NORMAL_MODE,
  GIMBAL_ENGINEER_MODE,
}gimbal_status;


/*底盘控制模式*/
/**************************2021.6*************************
*** @第四代车添加夹取地上去除补给模式，因此将进入补给模式
     改为夹取地上的模�?
*** @第四代车添加姿态调整（现打算姿态调整和兑换是同一模式�?
***********************************************************
*****************实现姿态调整的思路************************
*** @1、当进入兑换且SW1并未改变时、此时可通过摇杆CH调整储矿
        机构�?
*** @2、当SW1改变且保持在MI时，此时可通过摇杆CH调整夹取机构
        的舵机�?
**********************************************************/
typedef enum
{
  CHASSIS_RELEASE,
  CHASSIS_NORMAL_MODE,
	
  CHASSIS_RESCUE_MODE,
	CHASSIS_SUPPLY_MODE,         //补给模式
	CHASSIS_BARRIER_CARRY_MODE,  //障碍块搬�?
	CHASSIS_EXCHANGE_MODE,       //兑换金币   
	CHASSIS_ANGLE_MODE,
		
	CHASSIS_CLAMP_SMALL_ISLAND_MODE,
	CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE,//夹取大资源岛中间笔直矿石模式
	CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE,//夹取大资源岛两边倾斜矿石模式

	CHASSIS_CLAMP_CATCH_MODE,
	CHASSIS_GROUND_MODE,            //夹取地上模式
  CHASSIS_STOP_MODE,  
	
	CHASSIS_DEFEND_MODE,          //防守前哨战模�?
	CHASSIS_CHECK_MODE,						//为了检录最大伸展尺寸专门的模式
}chassis_status;

typedef enum
{
  RESCUE_INIT = 0,
  RESCUE_ENABLE, 
}rescue_status;
/*兑换金币控制*/
typedef enum
{
  EXCHANGE_INIT = 0,
  EXCHANGE_ENABLE_MODE, 
	
}exchange_status;
/*夹取控制*/
typedef enum
{
  CLAMP_INIT = 0,
	
	
  SMALL_ISLAND,            //小资源岛夹取模式
  BIG_ISLAND_STRAIGHT,              //大资源岛夹取模式
	BIG_ISLAND_SLANTED,              //大资源岛夹取模式
	GROUND_MODE,             //夹取地上模式
	CATCH_MODE,							 //空接模式
	
	EXCHANGE_MODE,           //兑换模式 
	TEST_MODE,
	
	DEFEND_MODE,//防守前哨�?
	
	
}clamp_status;

typedef enum
{
	SMALL_ISLAND_ORDINARY_MODE,            //小资源岛普通夹取模�?
	SMALL_ISLAND_AUTOMATIC_CLAMP_ONE_MODE, //小资源岛视觉辅助夹取一�?
	SMALL_ISLAND_AUTOMATIC_MODE,           //小资源岛视觉辅助夹取三箱
	
}small_island_mode_t;

typedef enum
{
  BIG_ISLAND_ORDINARY_MODE,            //大资源岛普通夹取模�?
	BIG_ISLAND_AUTOMATIC_CLAMP_ONE_MODE, //大资源岛视觉辅助夹取一�?
	
}big_island_mode_t;

typedef enum
{
  BARRIER_CARRY_INIT = 0,
  BARRIER_CARRY_ENABLE, 
}barrier_carry_status;

/*补给控制模式*/
typedef enum
{
  SUPPLY_INIT = 0,
  SUPPLY_TO_HERO,  //给英雄补�?
}supply_status;

extern global_status global_mode; 
extern global_status last_global_mode;

//extern gimbal_status gimbal_mode;
//extern gimbal_status last_gimbal_mode;

extern chassis_status chassis_mode;
extern chassis_status last_chassis_mode;

extern rescue_status rescue_mode;
extern rescue_status last_rescue_mode;

extern clamp_status clamp_mode;
extern clamp_status last_clamp_mode;

extern barrier_carry_status barrier_carry_mode;
extern barrier_carry_status last_barrier_carry_mode;

extern supply_status supply_mode;
extern supply_status last_supply_mode;


extern small_island_mode_t small_island_mode;
extern big_island_mode_t   big_island_mode;

extern gimbal_status gimbal_mode;

extern view_switch_t view_switch;

//extern uint8_t start_exexchange_cmd_state;
extern uint8_t start_big_island_mode_state ;

void mode_switch_task(void *parm);
void get_last_mode(void);
void get_main_mode(void);
void get_chassis_mode(void);

//void get_rescue_mode(void);
void get_clamp_mode(void);
void get_barrier_carry_mode(void);
void get_supply_mode(void);
void get_gimbal_mode(void);
//void get_shoot_mode(void);

void get_monitor_display_mode(void);
void get_client_layer_mode(void);
void get_view_switch(void);

#endif






