#include "remote_ctrl.h"
#include "stdlib.h"
#include "string.h"
#include "sys_config.h"
#include "STM32_TIM_BASE.h"

#include "supply_task.h"
#include "rescue_task.h"
#include "clamp_task.h"
#include "supply_task.h"
#include "gimbal_task.h"
#include "upraise_task.h"
#include "barrier_carry_task.h"
#include "clamp_task.h"
#include "keyboard.h"
#include "modeswitch_task.h"
#include "clamp.h"

rc_ctrl_t rm;
sw_record_t glb_sw;
clamp_action_status clamp_action;
exchange_action_status exchange_action;

//存矿与兑换使能用
uint8_t	change_mode_flag=4;
uint8_t clamp_action_ReadyToChange_state=0;
uint8_t exchange_action_ReadyToChange_state=0;

void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf)//遥控数据解算
{
	/*遥感数据解算，-1024是为了摇杆在中间时值为0*/
	rc->ch1 = (dbus_buf[0] | dbus_buf[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (dbus_buf[1] >> 3 | dbus_buf[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (dbus_buf[2] >> 6 | dbus_buf[3] << 2 | dbus_buf[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (dbus_buf[4] >> 1 | dbus_buf[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;
  
  /* prevent remote control zero deviation */
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;
  
	/*拨杆数据*/
  rc->sw1 = ((dbus_buf[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (dbus_buf[5] >> 4) & 0x0003;
  rc->iw = (dbus_buf[16] | dbus_buf[17] << 8) & 0x07FF;
	
	/**/
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_info_t));
    return ;
  }
	/*鼠标数据，z是滑轮*/
  rc->mouse.x = dbus_buf[6] | (dbus_buf[7] << 8); // x axis
  rc->mouse.y = dbus_buf[8] | (dbus_buf[9] << 8);
  rc->mouse.z = dbus_buf[10] | (dbus_buf[11] << 8);

  rc->mouse.l = dbus_buf[12];
  rc->mouse.r = dbus_buf[13];
	/*键盘*/
  rc->kb.key_code = ((dbus_buf[14] | dbus_buf[15] << 8)); // key borad code| (dbus_buf[16] | dbus_buf[17] << 8) << 8
}

/*
* @ RC_RESOLUTION :摇杆最大值 660 
* @ CHASSIS_RC_MAX_SPEED_X
    CHASSIS_RC_MAX_SPEED_Y
    CHASSIS_RC_MAX_SPEED_R ：平移和旋转的速度最大值
  @ CHASSIS_RC_MOVE_RATIO_X
    CHASSIS_RC_MOVE_RATIO_Y
    CHASSIS_RC_MOVE_RATIO_R : 数值方向
*/
int rocker_maximum_continuous_times;//如控制旋转时，摇杆持续打在最大档位将放开限制
static void chassis_operation_func(int16_t forward_back, int16_t left_right, int16_t rotate)//底盘运动控制函数
{
  rm.vx =  forward_back / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X * CHASSIS_RC_MOVE_RATIO_X;
  rm.vy = -left_right / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y * CHASSIS_RC_MOVE_RATIO_Y;
	
	/*因为在模拟小陀螺时限制了旋转速度会出现达不到想要的转速*/
	if(!(rotate == RC_RESOLUTION || rotate == - RC_RESOLUTION))//左侧摇杆横移，小陀螺
	{
		rocker_maximum_continuous_times = HAL_GetTick();//如果摇杆不是最大值就一直刷新rocker_maximum_continuous_times
	}
	if(HAL_GetTick() - rocker_maximum_continuous_times > 2000)//放开限制，如果摇杆最大值达到两秒，小陀螺速度放开限制
		rm.vw = rotate / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_R * CHASSIS_RC_MOVE_RATIO_R;
	else//无限制小陀螺速度
		rm.vw = rotate / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_R * CHASSIS_RC_MOVE_RATIO_R*0.5;//乘以0.5是因为操作时感觉转弯过于灵敏

}

void remote_ctrl_chassis_hook(void)
{
    chassis_operation_func(rc.ch1, rc.ch2, rc.ch3);
}
static void gimbal_operation_func( int32_t pit_ctrl)//云台控制函数，工程来说没什么用
{
	gimbal.pit_v += pit_ctrl/RC_RESOLUTION*3;
	
	gimbal.pit1_v += pit_ctrl/RC_RESOLUTION*3;
	
	/*限制控制角度*/
	if(gimbal.pit_v <= -300)
		gimbal.pit_v = -300;
	else if(gimbal.pit_v >= 2000)
		gimbal.pit_v = 2000;
	
	if(gimbal.pit1_v <= -2000)
		gimbal.pit1_v = -2000;
	else if(gimbal.pit1_v >= 2000)
		gimbal.pit1_v = 2000;
}

uint32_t ctrl_time       = 0;
uint32_t gimbal_ctrl_pit = 0;
uint8_t  gimbal_ctrl     =0;
uint8_t  ctrl_flag       = 0;

void remote_ctrl_gimbal_hook(void)
{
	//当左摇杆向上或向下到一定值时，且作用一定时间后进入控制云台
	if(rc.ch4 >= 250 || rc.ch4 <= (-250))
		ctrl_time++;
	//当摇杆归中一定时间后退出控制云台
	else if(rc.ch4 == 0 && ctrl_time > 0)
		ctrl_time--;
	
	if(ctrl_time > 500)
	{
		ctrl_flag = 1;			
		ctrl_time = 500;//当ctrl_time大于1000后定为1000，控制响应时间
	}
	else if(ctrl_time == 0)
	{
		ctrl_flag = 0;
	}
	
	if(ctrl_flag == 1)
	{	
		gimbal_ctrl = 1;//控制底盘vw能否转动
		
		if(rc.ch4 >= 0)//摇杆向上
		{
			if(rc.ch4 - 250 > 0)//将响应位置重置为起点
				gimbal_ctrl_pit = rc.ch4 - 250;
			else
				gimbal_ctrl_pit = 0;
		}	
		else//摇杆向下
		{				
			if(rc.ch4 + 250 < 0)
			{	
				gimbal_ctrl_pit = rc.ch4 + 250;
			}
			else
				gimbal_ctrl_pit = 0;
		}
		//gimbal_operation_func(gimbal_ctrl_pit);
	}	
	else if(ctrl_flag == 0)
	{
		gimbal_ctrl = 0;
	}	 
	
	gimbal_operation_func(rc.ch4);
}

/*补给使能*/
static void supply_operation_func(uint8_t supply_cmd1,uint8_t supply_cmd2)
{
  supply_mode = SUPPLY_TO_HERO;
  	
  if(supply_cmd1 && !supply_behind_state &&(HAL_GetTick() - supply_action_times < supply_continuous_time + 20))//补给前使能
  {
    supply.supply_cmd1 = 1;
  }
  else
  {
    supply.supply_cmd1 = 0;
  }
	if(supply_cmd2 && supply_behind_state && (HAL_GetTick() - supply_action_times < supply_continuous_time + 20))//补给后使能
  {
    supply.supply_cmd2 = 1;
  }
  else
  {
    supply.supply_cmd2 = 0;
  }
}
void remote_ctrl_supply_hook(void)
{
  supply_operation_func(RC_SUPPLY_CMD1,RC_SUPPLY_CMD2);
	
	if(RC_UPRAISE_INIT)
  {
    upraise.state = INIT_NEVER;
  }
}
//                      /*救援使能控制函数*/
///*******************************************日期：20.12.20********************
//********************************************经调试验证满足要求*****************
//***** @防止一次救援时未能抓住其他机器人
//*******所以就必须要可 独立 多次 进行对位
//***** @现象：第一次操作rescue_ctrl使能救援，第二次操作失能，第三次操作又使能。。。
//*****************************************************************************
//***************************编写调试注释者：张某秀 交流+QQ：3155460945**********/
//uint8_t rescue_cmd_state = 0;
//extern uint8_t rescue_over;
//static void rc_rescue_cmd(uint8_t rescue_ctrl)//该函数编辑调试耗时8小时
//{
//  if(rescue_ctrl && rescue_cmd_state == 0)//使能操作
//  {
//		rescue.rescue_cmd = 1;	
//  }
//	if(rescue.rescue_cmd == 1 && rescue_ctrl == 0)//判断到上次是使能操作，同时SW1键此时被切换了
//	{
//		rescue_cmd_state =1;
//	}
//	if(rescue_cmd_state == 1 && rescue_ctrl)//rescue_cmd_state == 1 证明当前是使能状态。 && rescue_ctrl表示再次操作rescue_ctrl，则失能救援
//	{
//		rescue.rescue_cmd = 0;	
//		rescue_over = 1;
//	}
//	if(rescue.rescue_cmd == 0 && rescue_ctrl == 0)//当前是失能状态，同时不处于失能操作
//	{
//		rescue_cmd_state = 0;
////		rescue_over = 1;
//	}
//}

///*复活使能控制函数*/
//static void rc_rescue_resurrection_cmd(uint8_t rescue_resurrection_ctrl)
//{
//  if(rescue_resurrection_ctrl)
//  {
//    rescue.rescue_resurrection_cmd = 1;//这里只是置1，在rescue_task.c里面会经判断置0，同时在rescue_task.c里面判断已经抓取成功复活才有效
//  }
//}
//void remote_ctrl_rescue_hook(void)
//{
//  if(chassis_mode == CHASSIS_RESCUE_MODE)
//	{
//		rc_rescue_cmd(RC_RESCUE_MODE_CMD);                         //救援使能
//		rc_rescue_resurrection_cmd(RC_RESCUE_RESURGENCE_MODE_CMD); //复活使能
//	}
//	else
//	{
//		
//	}
//	
//	if(RC_UPRAISE_INIT)
//  {
//    upraise.state = INIT_NEVER;
//  }
//}

/*搬运障碍块 使能/失能 控制函数*/
static void rc_barrier_carry_cmd(uint8_t barrier_carry_cmd_ctrl,uint8_t barrier_carry__disabilty_ctrl)
{
  if(barrier_carry_cmd_ctrl && chassis_mode == CHASSIS_BARRIER_CARRY_MODE)//使能操作
  {
		barrier_carry.barrier_carry_cmd = 1;	
		barrier_carry.barrier_carrry_flag = BARRIER_CARRYING;
  }
	else if(barrier_carry__disabilty_ctrl && chassis_mode == CHASSIS_BARRIER_CARRY_MODE)
	{
		barrier_carry.barrier_carry_cmd = 0;

    barrier_carry.barrier_carrry_flag = BARRIER_CARRYED;		
	}
	
}
/*遥控控制障碍块高度*/
static void barrier_carry_operation_func( int16_t barrier_carry_angle_ctrl)
{
  barrier_carry.rc_ctrl_angle += (barrier_carry_angle_ctrl/RC_RESOLUTION)*2;        //角度累计
	
	/*限制可调范围*/
	if(barrier_carry.rc_ctrl_angle >= 1000)
		barrier_carry.rc_ctrl_angle = 1000;
	else if(barrier_carry.rc_ctrl_angle <= -500)
		barrier_carry.rc_ctrl_angle = -500;
	
	/*使下次搬运时 该控制为0*/
	if(barrier_carry.barrier_carry_cmd)
	{
		barrier_carry.rc_ctrl_angle = 0;
	}
}
void remote_ctrl_barrier_carry_hook(void)
{
  rc_barrier_carry_cmd(RC_BARRIER_CARRY_MODE_CMD,RC_BARRIER_CARRY_MODE_UNCMD);//障碍块搬运使能
	
//	if(chassis_mode == CHASSIS_BARRIER_CARRY_MODE)
//	{
//		barrier_carry_operation_func(rc.ch4);
//	}
}

static void kb_enable_hook(void)
{
	km.kb_enable = 1;
}
uint8_t min_cmd_state;
uint8_t big_island_cmd_state   = 0; //保证一次操作仅使能一次
/*控制假如是第一次进入大资源岛，遥控没有变化则不能认为是使能操作*/
uint8_t enter_big_island_state      = 0; 
uint8_t big_island_cmd_change_state = 0;
uint8_t min_state            =0;

uint32_t  asdasdasd=0;
/****************删除以下****************************/
uint8_t throw_away_cmd_state            = 0;

uint8_t ground_cmd_state   = 0; //保证一次操作仅使能一次
/*控制假如是第一次进入夹取地上，遥控没有变化则不能认为是使能操作*/
uint8_t enter_ground_state      = 0; 
uint8_t ground_cmd_change_state = 0;
uint8_t catch_cmd_state   = 0; //保证一次操作仅使能一次
/*控制假如是第一次进入夹取地上，遥控没有变化则不能认为是使能操作*/
uint8_t enter_catch_state      = 0; 
uint8_t catch_cmd_change_state = 0;
static void rc_clamp_catch_cmd(uint8_t catch_clamp_single_ctrl,uint8_t catch_clamp_single_disability_ctrl)
{
//	if(chassis_mode != CHASSIS_CLAMP_CATCH_MODE)
//	{
//		enter_catch_state      = 0;
//		catch_cmd_change_state = 0;
//	}
//	else 
//	{
//		if(!catch_cmd_change_state)
//		  enter_catch_state = 1;
//	}
//	
//	if(chassis_mode == CHASSIS_CLAMP_CATCH_MODE)
//	{
//		if(catch_clamp_single_ctrl && !catch_cmd_state)//使能
//		{
//			clamp.clamp_cmd = 1;
//			
//			catch_cmd_state = 1;
//		}
//		else if(catch_clamp_single_disability_ctrl)//失能
//		{
//			clamp.clamp_cmd = 0;
//			
//			catch_cmd_state = 0;
//		}
//	}
}
/*使能/失能兑换动作*/
/**************************************
**************出现的问题****************
****@1.遥控进入兑换模式时 因为和使能的操作一致 导致直接有不合理的动作
*/
//uint8_t enter_exchange_state         = 0;  
//uint8_t exchange_cmd_change_state    = 0; 
//static void get_exchange_enter(void)  //因为遥控进入兑换模式时，rc.sw1的状态是随机的，导致可能误认为是使能操作
//{
//	if(chassis_mode != CHASSIS_EXCHANGE_MODE)
//	{
//		enter_exchange_state      = 0;
//		exchange_cmd_change_state = 0;
//	}
//	else 
//	{
//		if(!exchange_cmd_change_state)
//		  enter_exchange_state = 1;
//	}	
//}	
uint8_t exchange_cmd_state          ;
uint8_t exchange_pick_cmd_state     ;
uint8_t exhange_push_down_cmd_state ;

/****************删除以上****************************/

static void rc_exchange_ore_cmd(uint8_t exchange_cmd_ctrl,uint8_t exchange_pick_cmd_ctrl)
{
	/*避免一进入兑换操作 误认为是使能操作*/
//	get_exchange_enter();
	
	if(chassis_mode == CHASSIS_EXCHANGE_MODE)
	{
		if(exchange_cmd_ctrl)
		{
			if(exchange_cmd_state)
			{
				clamp.exchange_cmd = 1;
				exchange_cmd_state = 0;
				
			}
		}
		else if(exchange_pick_cmd_ctrl)
		{
			if(exchange_pick_cmd_state)
			{
				clamp.exchange_pick_cmd = 1;
				exchange_pick_cmd_state = 0;
			}
		}
		else
		{
			exchange_pick_cmd_state     = 1;
			exchange_cmd_state          = 1;

		}
		if(rc.sw1 == RC_MI)
		{
			clamp.exchange_cmd=0;
			clamp.exchange_pick_cmd=0;
		}
	}
	else
	{
		exchange_cmd_state          = 1;
		exhange_push_down_cmd_state = 0;
	}		
}

/*获取遥控控制夹取机构姿态数据*/
int32_t adjustment_slide_v;
int32_t adjustment_upraise_v;
int32_t adjustment_pit_v;
int32_t rc_delay_time;
static void ecxchange_operation_func( int32_t ctrl_x,int32_t ctrl_z,int32_t ctrl_pit)
{	
	if(HAL_GetTick() - rc_delay_time >200)
	{
		adjustment_pit_v += ctrl_pit/RC_RESOLUTION ;
		rc_delay_time = HAL_GetTick();
	}
	adjustment_upraise_v += ctrl_z/RC_RESOLUTION;

	adjustment_slide_v += ctrl_x/RC_RESOLUTION*3;

	
	/*限制控制角度*/
	if(adjustment_slide_v <= -800)
		adjustment_slide_v = -800;
	else if(adjustment_slide_v >= 800)
		adjustment_slide_v = 800;
	
		/*限制控制角度*/
	if(adjustment_upraise_v <= -500)
		adjustment_upraise_v = -500;
	else if(adjustment_upraise_v >= 500)
		adjustment_upraise_v = 500;
	
			/*限制控制角度*/
	if(adjustment_pit_v <= -45)
		adjustment_pit_v = -45;
	else if(adjustment_pit_v >= 45)
		adjustment_pit_v = 45;
}


//24赛季动作使能判断
void rc_clamp_store_cmd()
{
	uint8_t action_max_number=1;
	//这不比原来那一堆函数和宏定义简洁高效多了
	if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE 
		|| chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE
		||chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE 
		||chassis_mode == CHASSIS_GROUND_MODE )
	{
		action_max_number=1;
		if(chassis_mode==CHASSIS_CLAMP_SMALL_ISLAND_MODE)
			action_max_number=2;
		if(rc.sw1==RC_MI)
			clamp_action_ReadyToChange_state=1;
		if(clamp_action_ReadyToChange_state)
		{
			if(RC_SW1_DN)
			{
				
				if(clamp_action<=action_max_number)clamp_action++;
				clamp_action_ReadyToChange_state=0;
			}
			if(RC_SW1_UP)
			{
				if(clamp_action>=1)clamp_action--;
				clamp_action_ReadyToChange_state=0;
				if(have_box_number>=1)have_box_number--;
			}
		}
	}
}

void rc_exchange_action_cmd()
{
	if(chassis_mode == CHASSIS_EXCHANGE_MODE || chassis_mode == CHASSIS_ANGLE_MODE)
	{
		if(rc.sw1==RC_MI)
		{
			exchange_action_ReadyToChange_state=1;
			exchange_action=EXCHANGE_UN_CMD;
		}
		if(exchange_action_ReadyToChange_state)
		{
			if(RC_SW1_DN)//放
			{
				exchange_action=RELESE_BOX_ACTION;
				exchange_action_ReadyToChange_state=0;
			}
			if(RC_SW1_UP)//取
			{
				if(exchange_action_ReadyToChange_state)
					exchange_action=PICK_ACTION;
				exchange_action_ReadyToChange_state=0;
			}
		}
	}
}

void remote_ctrl_clamp_hook(void)
{
	rc_clamp_store_cmd();//超级无敌之一个更比5个强（替代原有的5个函数）
	rc_exchange_action_cmd();
	//存矿使能
//	rc_store_cmd(RC_ORDINARY_SINGLE_STORE_CMD,RC_ORDINARY_SINGLE_STORE_UNCMD,RC_ORDINARY_MIN_STORE_CMD);
//	/*夹取使能*/
//	rc_clamp_small_island_cmd (RC_SMALL_ORDINARY_SINGLE_CLAMP_CMD,RC_SMALL_ORDINARY_SINGLE_CLAMP_UNCMD);//使能/失能 普通夹取，视觉夹取时由算法数据使能
//  rc_clamp_big_island_cmd(RC_BIG_ORDINARY_SINGLE_CLAMP_CMD,RC_BIG_ORDINARY_SINGLE_CLAMP_UNCMD);
//	/*丢弃使能*/
//	rc_clamp_throw_away_cmd(RC_CLAMP_THROW_AWAY_CMD,RC_CLAMP_THROW_AWAY_UNCMD);
//	/*夹取地上使能*/
//	rc_clamp_ground_cmd(RC_GROUND_SINGLE_CLAMP_CMD,RC_GROUND_SINGLE_CLAMP_UNCMD);
//	rc_clamp_catch_cmd(RC_CATCH_SINGLE_CLAMP_CMD,RC_CATCH_SINGLE_CLAMP_UNCMD);
	/*调整舵机控制*/
	if(exchange_start_push_flag)
	{
		ecxchange_operation_func(rc.ch1,rc.ch2,rc.ch3);
	}
	else
  {
		adjustment_slide_v = 0;
		adjustment_upraise_v = 0;
		adjustment_pit_v = 0;
	}
	
	/*兑换使能*/
	rc_exchange_ore_cmd(RC_EXCHANGE_ORDINARY_SINGLE_CMD,RC_EXCHANGE_PICK_ORDINARY_SINGLE_CMD);
	
  /*使能键盘鼠标*/
  kb_enable_hook();
}



