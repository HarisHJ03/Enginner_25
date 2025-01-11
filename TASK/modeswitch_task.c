#include "modeswitch_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "remote_ctrl.h"
#include "keyboard.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "barrier_carry_task.h"
#include "upraise_task.h"
#include "clamp_task.h"
#include "rescue_task.h"
#include "supply_task.h"
#include "controller.h"

#include "pc_tx_data.h"
#include "judge_tx_data.h"

#include "bsp_can.h"
#include "motor_8010.h"
/**********************************************************************************
*******本源代码控制逻辑于20.12.19张某秀修改（若有建议及疑问请联系QQ：3155460945）******
***********************************************************************************
//1.首先获取全局状态，然后经操作选定底盘模式
//2.底盘模式包含其他工程模式的功能：补给、夹取、障碍块搬运、救援、兑换金币模式
//3.所以判断进入工程模式的哪种模式可通过查看底盘的模式
*********************21.1.31开始全部修改了夹取模式的控制逻辑**************************

***********************************************************************************/

UBaseType_t mode_switch_stack_surplus;

extern TaskHandle_t info_get_Task_Handle;

global_status global_mode; 
global_status last_global_mode = RELEASE_CTRL;

chassis_status chassis_mode;
chassis_status last_chassis_mode = CHASSIS_RELEASE;

rescue_status rescue_mode;
rescue_status last_rescue_mode = RESCUE_INIT;

clamp_status clamp_mode;
clamp_status last_clamp_mode = CLAMP_INIT;

barrier_carry_status barrier_carry_mode;
barrier_carry_status last_barrier_carry_mode = BARRIER_CARRY_INIT;

supply_status supply_mode;
supply_status last_supply_mode;

gimbal_status gimbal_mode;

/*小/大资源岛模式具体的操作模式不需要处理上次的模式*/
small_island_mode_t small_island_mode;
big_island_mode_t   big_island_mode;

void mode_switch_task(void *parm)
{
	uint32_t mode_switch_wake_time = osKernelSysTick();
	while(1)
	{
		
		//go8010_task();
		get_last_mode();           //获取所有任务上次的模式
		get_main_mode();
		get_chassis_mode();
//		get_rescue_mode();
		get_supply_mode();
		get_clamp_mode();          //注意要包含读取矿石抬升
		get_barrier_carry_mode();
		get_gimbal_mode();
		get_monitor_display_mode();//获取显示屏的显示视野
		get_view_switch();
		
//    /*发送遥控数据*/
//    send_rc_data1();
//    send_rc_data2();
//    send_rc_data3();
    
    xTaskGenericNotify( (TaskHandle_t) info_get_Task_Handle, 
                        (uint32_t) MODE_SWITCH_INFO_SIGNAL, 
                        (eNotifyAction) eSetBits, 
                        (uint32_t *)NULL );
    
    mode_switch_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
    
    vTaskDelayUntil(&mode_switch_wake_time, 6);//绝对延时函数
  }
}
/*************************************************************
*****1.首先判断状态有没有变化
*****2.状态发送变化时再赋值上次状态
*****3.上次状态和当前状态保持不一样，即当前状态切换时再更新上次状态

********该思路只在工程代码更改，其他或可能是沿用新框架的思路
***************************************************************/
uint8_t	global_change_state = 1;
	/*中间变化模式缓冲*/
uint8_t	global_middle_change_mode;
uint8_t chassis_middle_change_mode;
uint8_t rescue_middle_change_mode;
uint8_t clamp_middle_change_mode ;
uint8_t barrier_carry_middle_change_mode;
uint8_t supply_middle_change_mode;

void get_last_mode(void)
{	
	if(global_change_state)//只运行一次，初始化就好
	{		
		global_middle_change_mode        = global_mode;
		last_global_mode                 = (global_status)global_middle_change_mode;
		
		chassis_middle_change_mode       = chassis_mode;
		last_chassis_mode                = (chassis_status)chassis_middle_change_mode;
		
		rescue_middle_change_mode        = rescue_mode;
		last_rescue_mode                 = (rescue_status)rescue_middle_change_mode;
		
		clamp_middle_change_mode         = clamp_mode;
		last_clamp_mode                  = (clamp_status)clamp_middle_change_mode;
		
    	barrier_carry_middle_change_mode = barrier_carry_mode;
		last_barrier_carry_mode          = (barrier_carry_status)barrier_carry_middle_change_mode;
		
		supply_middle_change_mode        = supply_mode;
		last_supply_mode                 = (supply_status)supply_middle_change_mode;
		
		global_change_state = 0;
	}
	if((global_status)global_middle_change_mode != global_mode )                      //获取上一次全局状态
	{
		last_global_mode                = (global_status)global_middle_change_mode;
		global_middle_change_mode       = global_mode;
	}
	if((chassis_status)chassis_middle_change_mode !=chassis_mode )                    //获取上一次底盘状态
	{
		last_chassis_mode               = (chassis_status)chassis_middle_change_mode;
		chassis_middle_change_mode      = chassis_mode;
		
		/*控制抬升 发生模式变化时 抬升的变动都需用单环控制好升降速度*/
		upraise_angle_ctrl_state[0] = 0;
		upraise_angle_ctrl_state[1] = 0;
		upraise.updown_flag        = RAISE; //抬升标志
	}
	if(rescue_middle_change_mode != rescue_mode)                                      //获取上一次救援状态
	{
		last_rescue_mode                 = (rescue_status)rescue_middle_change_mode;
		rescue_middle_change_mode        = rescue_mode;
	}
	if((clamp_status)clamp_middle_change_mode != clamp_mode)                          //获取上一次夹取状态
	{
		last_clamp_mode                  = (clamp_status)clamp_middle_change_mode;
		clamp_middle_change_mode         = clamp_mode;
	}
	if((barrier_carry_status)barrier_carry_middle_change_mode != barrier_carry_mode)   //获取上一次搬运障碍块状态
	{
		last_barrier_carry_mode          = (barrier_carry_status)barrier_carry_middle_change_mode;
		barrier_carry_middle_change_mode = barrier_carry_mode;
	}
	if((supply_status)supply_middle_change_mode != supply_mode)                        //获取上一次补给状态
	{
		last_supply_mode                 = (supply_status)supply_middle_change_mode;
		supply_middle_change_mode        = supply_mode;
	}
}

void get_main_mode(void)
{   
  switch(rc.sw2)
  {
    case RC_UP:
    {
    	global_mode = MANUAL_CTRL;   			//正常模式
    }
    break;
    case RC_MI:
    {
    	global_mode = ENGINEER_CTRL;           //工程模式（夹取兑换等）
    }break;
    case RC_DN:
    {
    	global_mode = RELEASE_CTRL;						//断电状态
    }
    break;
    default:
    {
		global_mode = RELEASE_CTRL;
    }break;
  }
}
//extern uint8_t rescue_over;
uint8_t start_big_island_mode_state = 0;//为了控制刚进入大资源岛操作时，抬头查看灯效添加进入模式标志位

void get_chassis_mode(void)
{
  switch(global_mode)
  {
    case RELEASE_CTRL:
		{
    	chassis_mode = CHASSIS_RELEASE;
			PUMP_OFF
		}
    break;  
		
		/*因特殊情况在此添加防守模式*/
    case MANUAL_CTRL:
    {
			if((glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP) &&  rc.sw1 == RC_UP)
			{
				chassis_mode = CHASSIS_BARRIER_CARRY_MODE;
			}
			else if((glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP) &&  rc.sw1 == RC_DN)
			{
				chassis_mode = CHASSIS_CHECK_MODE;
			}
			else if(chassis_mode != CHASSIS_BARRIER_CARRY_MODE && chassis_mode != CHASSIS_CHECK_MODE)
			{
        		chassis_mode = CHASSIS_NORMAL_MODE;
			}
			
			if((glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN) && chassis_mode == CHASSIS_BARRIER_CARRY_MODE)
			{
				chassis_mode = CHASSIS_NORMAL_MODE;
			}
			else if((glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN) && chassis_mode == CHASSIS_CHECK_MODE)
			{
				chassis_mode = CHASSIS_NORMAL_MODE;				
			}
    }break;
    
	/***********************************************************
	***************global_mode = ENGINEER_CTRL******************
	******************该模式下切换其他模式说明*********************
	**** @1.rc.sw1 = RC_UP && glb_sw.last_iw向上 切换到笔直大资源岛模式
	**** @1.rc.sw1 = RC_UP && glb_sw.last_iw向下 切换到倾斜大资源岛模式

	**** @4.rc.sw1 = RC_MI && glb_sw.last_iw向上 切换到兑换模式
	**** @5.rc.sw1 = RC_MI && glb_sw.last_iw向下 切换到检录模式

	**** @6.rc.sw1 = RC_DN && glb_sw.last_iw向上 切换到侧臂小资源岛夹取模式
	**** @7.rc.sw1 = RC_DN && glb_sw.last_iw向上 切换到夹地面模式
	****************************************************************/
    case ENGINEER_CTRL:
    {
		/*************工程模式——遥控切换模式**************/
		if(RC_IW_UP && rc.sw1 == RC_UP)
    {          
       chassis_mode = CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE;    //笔直大资源岛模式   
		}
		else if(RC_IW_DN && rc.sw1 == RC_UP)
		{
			chassis_mode = CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE;  	  //倾斜大资源岛夹取模式			
    }
		else if(RC_IW_UP && rc.sw1 == RC_MI)
		{
			chassis_mode = CHASSIS_EXCHANGE_MODE;     				  //兑换模式	
//				start_exexchange_cmd_state = 1;
		}
		else if(RC_IW_DN && rc.sw1 == RC_MI)
		{
			chassis_mode = CHASSIS_ANGLE_MODE;     						//前臂小资源岛夹取模式
		}

		else if(RC_IW_UP && rc.sw1 == RC_DN)
		{
			chassis_mode = CHASSIS_CLAMP_SMALL_ISLAND_MODE;    		  //侧臂小资源岛夹取模式
			
			start_big_island_mode_state = 1;
		}

		else if(RC_IW_DN && rc.sw1 == RC_DN)
     	{        
        	chassis_mode = CHASSIS_CHECK_MODE;		        	 	  //夹取地面模式
      	}				
			/*键盘鼠标操作切换模式*/
//      if(KB_CLAMP_SMALL_ISLAND_MODE)                              
//        chassis_mode = CHASSIS_CLAMP_SMALL_ISLAND_MODE;
//			else if(KB_CLAMP_BIG_ISLAND_MODE )                         
//        chassis_mode = CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE;
//			else if(KB_EXCHANGE_MODE )                         
//        chassis_mode = CHASSIS_EXCHANGE_MODE;
//      else if(KB_SUPPLY_MODE)
//        chassis_mode = CHASSIS_RESCUE_MODE;
//      else if(KB_RESCUE_MODE)
//        chassis_mode = CHASSIS_SUPPLY_MODE;
//			else if(KB_BARRIER_CARRY_MODE)
//        chassis_mode = CHASSIS_BARRIER_CARRY_MODE;  	
    }break;    			
    default:
    {
		chassis_mode = CHASSIS_RELEASE;
    }break;
  }
}

//void get_rescue_mode(void)
//{ 
//	/*使能失能救援*/
//	remote_ctrl_rescue_hook();
////  keyboard_rescue_hook();
//	/*救援时不需要抬升时只需下面一行*/
// // upraise.updown_flag = FALL; //下降标志
//	
//	if(chassis_mode != CHASSIS_RESCUE_MODE)
//	{
//    	rescue.upraise_updown_flag = 0;
//	}
//	/*因调试需要将救援模式下需要抬升*/
// 	if(chassis_mode == CHASSIS_RESCUE_MODE)
//	{
//		if(!rescue.upraise_updown_flag)
//		{				
//			rescue.upraise_updown_flag = 1;
//			upraise.updown_flag        = RAISE; //抬升标志
//		}  
//	}
//	else if( chassis_mode != CHASSIS_RESCUE_MODE && chassis_mode != CHASSIS_BARRIER_CARRY_MODE && chassis_mode != CHASSIS_SUPPLY_MODE
//      		&& chassis_mode != CHASSIS_CLAMP_SMALL_ISLAND_MODE  && chassis_mode != CHASSIS_EXCHANGE_MODE 
//	        && chassis_mode != CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE && chassis_mode != CHASSIS_DEFEND_MODE && chassis_mode != CHASSIS_GROUND_MODE
//  	    	&& chassis_mode != CHASSIS_CLAMP_CATCH_MODE && chassis_mode != CHASSIS_CHECK_MODE && upraise.updown_flag != DOWN )
//	{
//		upraise.updown_flag = FALL;
//	}
//}

/*小资源岛模式选择*/
/******************************************************
*****************1.小资源岛夹取模式分三种**************
 @普通夹取         -- 操作手对位，操作遥控使能夹取一次
 @视觉辅助夹取一箱 -- 只由算法夹取一箱便自动中断夹取 
 @视觉辅助夹取全部 -- 全程交由算法数据处理
******************************************************
******** 1.默认进入普通夹取
******** 2.经不同操作分别进入三种不同模式*/
uint8_t enter_clamp_state              = 0;  //为1证明进入了夹取模式
uint8_t small_island_mode_change_state = 0;  //为1证明进入了夹取模式，同时已操控切换了模式

/*因为遥控进入夹取模式时，rc.sw1的状态是随机的，导致可能直接进入视觉模式*/
static void get_clamp_enter_mode(void)       
{
	if(chassis_mode != CHASSIS_CLAMP_SMALL_ISLAND_MODE)
	{
		enter_clamp_state              = 0;
		small_island_mode_change_state = 0;
	}
	else 
	{
		if(!small_island_mode_change_state)
		enter_clamp_state = 1;
	}	
}	
static void get_clamp_small_island_mode(uint8_t samll_single_automatic_clamp_one_mode
	                                    ,uint8_t samll_single_automatic_mode
                                        ,uint8_t samll_single_ordinary_mode)
{

	if(clamp_mode == SMALL_ISLAND)
	{
////		if(samll_single_automatic_clamp_one_mode)
////		{
////			small_island_mode = SMALL_ISLAND_AUTOMATIC_CLAMP_ONE_MODE;				
////		}
////		else if(samll_single_automatic_mode)
////		{
////			small_island_mode = SMALL_ISLAND_AUTOMATIC_MODE;			
////		}
////		else if(samll_single_ordinary_mode)
////		{
////			small_island_mode = SMALL_ISLAND_ORDINARY_MODE;  
////		}
////		else
////		{
////			small_island_mode = SMALL_ISLAND_ORDINARY_MODE;         //默认进入小资源岛模式时进入普通模式				
////		}
////		if(enter_clamp_state)//第一次进入防止进入视觉模式
////		{
////			if(rc.sw1 == RC_MI)
////				small_island_mode = SMALL_ISLAND_ORDINARY_MODE; 
////			else
////			{
////				small_island_mode_change_state = 1;
////				enter_clamp_state              = 0;
////			}
////		}
////	}
////	else
////	{
		small_island_mode = SMALL_ISLAND_ORDINARY_MODE; 
	}
}
/*大资源岛夹取模式选择*/
/***********************
**** 1.大资源岛夹取模式分为两种
     @普通夹取
     @视觉辅助夹取一箱
**** 2.大资源岛的底盘移动将由键盘按键控制
**** 3.移动至矿石大致位置时，可操作手操作对位，操作夹取（键盘可使能该模式下的夹取）
**** 4.可经遥控操作进入视觉辅助夹取一箱               （键盘也可操作进入视觉辅助模式）
*/
static void get_clamp_big_island_mode(uint8_t big_single_automatic_clamp_one_mode,uint8_t big_island_ordinary_mode)
{
//	if(clamp_mode == BIG_ISLAND)
//	{		
//		if(big_single_automatic_clamp_one_mode)      //视觉辅助夹取一箱
//		{
//			big_island_mode = BIG_ISLAND_AUTOMATIC_CLAMP_ONE_MODE;				
//		}
//		if((big_island_ordinary_mode && !KB_BIG_ISLAND_AUTO_SINGLE_CLAMP_STATE)|| big_island_mode != BIG_ISLAND_AUTOMATIC_CLAMP_ONE_MODE)
//		{
//			big_island_mode = BIG_ISLAND_ORDINARY_MODE;//默认进入大资源岛模式时进入普通模式			
//		}
//	}
//	/*不是大资源岛夹取模式时，将其模式切换回普通夹取模式，防止下次直接进入视觉辅助夹取模式*/
//	else
//	{
		big_island_mode = BIG_ISLAND_ORDINARY_MODE;
//	}	
}
/**************夹取模式介绍********日期：2021.1.15*********/
/**** @1.夹取分为普通夹取和算法的视觉辅助夹取
***** @2.两种模式默认进入的都是小/大资源岛的普通夹取模式

*************将夹取分为大资源岛夹取和小资源岛夹取*****日期：2021.1.31*****
*/
/*控制抬升*/
static void get_clamp_upraise_updown_flag(void)
{
	if(chassis_mode != CHASSIS_CLAMP_SMALL_ISLAND_MODE)
	{
		clamp.small_island_upraise_updown_flag = 0;
	}
	if(chassis_mode != CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE)
	{
		clamp.big_island_upraise_updown_flag = 0;
	}
	if(chassis_mode != CHASSIS_EXCHANGE_MODE )
	{
		clamp.exchange_upraise_updown_flag = 0;	
	}
	if(chassis_mode != CHASSIS_DEFEND_MODE)
	{
		clamp.defend_upraise_updown_flag = 0;
	}
	if(chassis_mode != CHASSIS_GROUND_MODE)
	{
		clamp.ground_upraise_updown_flag = 0;
	}
	if(chassis_mode != CHASSIS_CLAMP_CATCH_MODE)
	{
		clamp.catch_upraise_updown_flag = 0;
	}	
}

void get_clamp_mode(void)
{  
	remote_ctrl_clamp_hook();
	keyboard_clamp_hook();
	
	get_clamp_enter_mode();//处理好，一进入夹取模式时，不能出现进入视觉模式
	
	/*夹取模式选择*/
	if(chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE)		
	{
//		if(RC_SMALL_SINGLE_ORDINARY_MODE)
//		{
			clamp_mode = SMALL_ISLAND;		
//		}
	}
	else if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE)
	{
		clamp_mode = BIG_ISLAND_STRAIGHT;
	}
	else if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE)
	{
		clamp_mode = BIG_ISLAND_SLANTED;
	}
	else if(chassis_mode == CHASSIS_GROUND_MODE)
	{
		clamp_mode = GROUND_MODE;
	}
	else if(chassis_mode == CHASSIS_EXCHANGE_MODE)
	{
		clamp_mode = EXCHANGE_MODE;
	}
	else if(chassis_mode == CHASSIS_DEFEND_MODE)
	{
		clamp_mode = DEFEND_MODE;
	}
	else if(chassis_mode == CHASSIS_CLAMP_CATCH_MODE)
	{
		clamp_mode = CATCH_MODE;
	}
	else
	{
		clamp_mode = CLAMP_INIT;
		if(chassis_mode == CHASSIS_RELEASE)
		{
			clamp_init_times = HAL_GetTick();
		}
	}
	
	/*获取小资源岛的夹取模式*/
	get_clamp_small_island_mode(RC_SMALL_SINGLE_AUTOMATIC_CLAMP_ONE_MODE,RC_SMALL_SINGLE_AUTOMATIC_MODE,RC_SMALL_SINGLE_ORDINARY_MODE);
	/*获取大资源岛的夹取模式*/
	get_clamp_big_island_mode(RC_BIG_SINGLE_AUTOMATIC_CLAMP_ONE_MODE,RC_BIG_SINGLE_ORDINARY_CLAMP_MODE);
	
	/*控制抬升*/
	get_clamp_upraise_updown_flag();
	/*控制抬升代码*/
  	if( chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE)
	{
		if(!clamp.small_island_upraise_updown_flag)
		{				
		  	clamp.small_island_upraise_updown_flag = 1;
		 	upraise.updown_flag = RAISE; //抬升标志
		}  
	}
	else if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE)
	{
		if(!clamp.big_island_upraise_updown_flag)
		{
			clamp.big_island_upraise_updown_flag = 1;
		  	upraise.updown_flag = RAISE; //抬升标志
		}	
	}
	else if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE)
	{
		if(!clamp.big_island_upraise_updown_flag)
		{
			clamp.big_island_upraise_updown_flag = 1;
		  	upraise.updown_flag = RAISE; //抬升标志
		}	
	}
	else if(chassis_mode == CHASSIS_EXCHANGE_MODE)
	{
		if(!clamp.exchange_upraise_updown_flag)
		{
			clamp.exchange_upraise_updown_flag = 1;
			upraise.updown_flag = RAISE; //抬升标志
		}
	}
	else if(chassis_mode == CHASSIS_GROUND_MODE)
	{
		if(!clamp.ground_upraise_updown_flag)
		{
			clamp.ground_upraise_updown_flag = 1;
			upraise.updown_flag = RAISE; //抬升标志
		}		
	}
	else if(chassis_mode == CHASSIS_DEFEND_MODE)
	{
		if(!clamp.defend_upraise_updown_flag)
		{
			clamp.defend_upraise_updown_flag = 1;
			upraise.updown_flag = RAISE; //抬升标志
		}
	}
	else if(chassis_mode == CHASSIS_CLAMP_CATCH_MODE)
	{
		if(!clamp.catch_upraise_updown_flag)
		{
			clamp.catch_upraise_updown_flag = 1;
			upraise.updown_flag = RAISE; //抬升标志
		}
	}
	else if(chassis_mode != CHASSIS_RESCUE_MODE && chassis_mode != CHASSIS_BARRIER_CARRY_MODE 
		    && chassis_mode != CHASSIS_SUPPLY_MODE && chassis_mode != CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE      
  	      	&& chassis_mode != CHASSIS_EXCHANGE_MODE && chassis_mode != CHASSIS_CLAMP_SMALL_ISLAND_MODE 
	        && upraise.updown_flag != DOWN  && chassis_mode != CHASSIS_DEFEND_MODE && chassis_mode != CHASSIS_GROUND_MODE 
			&& chassis_mode != CHASSIS_CLAMP_CATCH_MODE && chassis_mode != CHASSIS_CHECK_MODE)
	{
		upraise.updown_flag = FALL;
	}
}

void get_supply_mode(void)
{
  	remote_ctrl_supply_hook();
//  keyboard_supply_hook();
	
  	if(chassis_mode != CHASSIS_SUPPLY_MODE)
	{
		supply.upraise_updown_flag = 0;
	}
	
  	if(chassis_mode == CHASSIS_SUPPLY_MODE)
	{
		if(!supply.upraise_updown_flag)
		{
			supply.upraise_updown_flag = 1;
			upraise.updown_flag = RAISE; //抬升标志
		}   
	}
	else if(chassis_mode != CHASSIS_RESCUE_MODE && chassis_mode != CHASSIS_BARRIER_CARRY_MODE 
		    && chassis_mode != CHASSIS_SUPPLY_MODE && chassis_mode != CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE      
  	        && chassis_mode != CHASSIS_EXCHANGE_MODE && chassis_mode != CHASSIS_CLAMP_SMALL_ISLAND_MODE 
	        && upraise.updown_flag != DOWN && chassis_mode != CHASSIS_DEFEND_MODE && chassis_mode != CHASSIS_GROUND_MODE
			&& chassis_mode != CHASSIS_GROUND_MODE && chassis_mode != CHASSIS_CLAMP_CATCH_MODE && chassis_mode != CHASSIS_CHECK_MODE)
	{
		upraise.updown_flag = FALL;
	}
}

void get_barrier_carry_mode(void)
{
	remote_ctrl_barrier_carry_hook();
//  keyboard_barrier_carry_hook();
  	if(chassis_mode != CHASSIS_BARRIER_CARRY_MODE)
	{
		barrier_carry.upraise_updown_flag = 0;
	}
  	if(chassis_mode == CHASSIS_BARRIER_CARRY_MODE)
	{
		if(!barrier_carry.upraise_updown_flag)
		{		
			barrier_carry.upraise_updown_flag = 1;
			upraise.updown_flag = RAISE; //抬升标志
		}
	}
	else if(chassis_mode != CHASSIS_RESCUE_MODE && chassis_mode != CHASSIS_BARRIER_CARRY_MODE 
		    && chassis_mode != CHASSIS_SUPPLY_MODE && chassis_mode != CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE      
  	      	&& chassis_mode != CHASSIS_EXCHANGE_MODE && chassis_mode != CHASSIS_CLAMP_SMALL_ISLAND_MODE 
	        && upraise.updown_flag != DOWN  && chassis_mode != CHASSIS_DEFEND_MODE &&  chassis_mode != CHASSIS_GROUND_MODE
			&& chassis_mode != CHASSIS_GROUND_MODE && chassis_mode != CHASSIS_CLAMP_CATCH_MODE && chassis_mode != CHASSIS_CHECK_MODE)
	{
		upraise.updown_flag = FALL;
	}
}

void get_gimbal_mode(void)
{
	remote_ctrl_gimbal_hook();
	
	if( chassis_mode == CHASSIS_RELEASE)
	{
		gimbal_mode = GIMBAL_RELEASE;
	}
	else if( chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE 
			||(chassis_mode == CHASSIS_SUPPLY_MODE  && supply_behind_state)
			|| chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE 
			|| chassis_mode == CHASSIS_EXCHANGE_MODE 
			|| chassis_mode == CHASSIS_GROUND_MODE
			|| chassis_mode==CHASSIS_CLAMP_CATCH_MODE
	  		|| chassis_mode == CHASSIS_RESCUE_MODE)
	{
		gimbal_mode = GIMBAL_ENGINEER_MODE;
	}
	else
	{
		gimbal_mode = GIMBAL_NORMAL_MODE;
	}
}

void get_view_switch(void)
{
//	if(chassis_mode==CHASSIS_CLAMP_SMALL_ISLAND_MODE||chassis_mode==CHASSIS_GROUND_MODE||chassis_mode==
//		CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE || (chassis_mode == CHASSIS_EXCHANGE_MODE && exchange_view_switch_flag == 2))
//	{
//		view_switch = CLAMP_VIEW;
//	}
//	else
//	{
//		view_switch = ODJIUST_VIEW;
//	}
	view_switch = CLAMP_VIEW;
}

view_switch_t view_switch = CLAMP_VIEW;
void get_monitor_display_mode(void)
{ /*控制显示器切屏*/
	if(view_switch == CLAMP_VIEW)
	{
		GPIO_ResetBits(GPIOF,GPIO_Pin_0);
//	   GPIO_ResetBits(GPIOC,GPIO_Pin_1);
	}
	else if(view_switch == ODJIUST_VIEW)
	{	
		GPIO_SetBits(GPIOF,GPIO_Pin_0);
//	   GPIO_SetBits(GPIOC,GPIO_Pin_1);
	}
}







