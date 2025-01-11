#include "gimbal_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "comm_task.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "rescue_task.h"
#include "bsp_can.h"
#include "upraise_task.h"
#include "clamp_task.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "sys_config.h"
#include "pc_rx_data.h"
#include "pid.h"
#include "stdlib.h"
#include "stdlib.h" //abs()函数
#include "math.h"   //fabs()函数

/*************************************************
******************第NEW代工程**********************
********该占比和舵机的装配时角度有关**************
*******************云台 1 YAW*********************
**** @TIM2_CH1  PA0  S

 看屏幕 570
 看前方 2550
 
 *******************云台 1 PIT*********************
**** @TIM2_CH2  PA1  T

 最大1464   平视 1156   最小 890
 
*******************云台 2 YAW********************* 
**** @TIM2_CH3  PA2  U

看矿石 1600   看后方 660

**************************************************
*******************云台 2 PIT*********************
***  @TIM2_CH4  PA3  V

最高  1100  平视 1350  最低 1700

*********************************日期：2022年6月30日*/


UBaseType_t gimbal_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

#if (INFANTRY_NUM == INFANTRY_1)

/*云台1软件限位*/
#define PIT_ONE_MAX      0  	// 1140 平视
#define PIT_ONE_MIN      0

#define YAW_ONE_MAX      0  
#define YAW_ONE_MIN      0

/*特定角度控制*/

#define PIT_ONE_LEVEL    1100   //云台1平视给定
#define PIT_ONE_CLAMP    1100   //云台1夹取大小资源岛给定
#define PIT_ONE_CATCH  	 800  	//云台1看空接给定		增大向下，减小向上
#define PIT_ONE_GROUND   1500  	//云台1看夹取地面给定
#define PIT_ONE_SCREEN	 1350  	//云台看显示屏给定

#define PIT_ONE_CHECK = 0		//检录

#define YAW_ONE_FORWARD  1055	//云台1看前方给定	增大向左，减小向右
#define YAW_ONE_CLAMP  	 1055	//云台1夹取大小资源岛给定
#define YAW_ONE_ORE		 1055   //云台1看矿石给定
#define YAW_ONE_SCREEN   1700   //云台1看显示器给定

#endif

gimbal_t gimbal;

int16_t yaw_debug_angle;
int16_t pit_debug_angle;
void gimbal_task(void *parm)
{
  	uint32_t Signal;
	BaseType_t STAUS;
  
  	while(1)
   	{
		/*此函数用于获取通知值和清除通知值的指定位值*/
    	STAUS = xTaskNotifyWait((uint32_t) NULL, 					//等待前清零指定任务通知值的比特位（旧值对应bit清0）
								(uint32_t) INFO_GET_GIMBAL_SIGNAL,  //成功等待后清零指定的任务通知值比特位（新值对应bit清0）
								(uint32_t *)&Signal, 				//用来取出通知值（如果不需要取出，可设为NULL）
								(TickType_t) portMAX_DELAY );		//设置阻塞时间
    	if(STAUS == pdTRUE)
		{
			if(Signal & INFO_GET_GIMBAL_SIGNAL)
			{
				switch(gimbal_mode)
				{
					case GIMBAL_NORMAL_MODE:
					{
						gimbal_nomarl_handler();
					}break;

					case GIMBAL_ENGINEER_MODE:
					{
						gimbal_engineer_handler();
					}break;

					case GIMBAL_RELEASE:
					{
						gimbal_release_handler();
					}break;

					default:
					{								
					}break; 
				}

    		/*软件限位*/
//			VAL_LIMIT(gimbal.PIT_ONE_REF,PIT_ONE_MIN,PIT_ONE_MAX);
//			VAL_LIMIT(gimbal.YAW_ONE_REF,0,YAW_ONE_MAX);

		    /*输出 PWM*/		
			TIM2->CCR1 = gimbal.YAW_ONE_REF+yaw_debug_angle;	//PA0  S
			TIM2->CCR2 = gimbal.PIT_ONE_REF+pit_debug_angle;  	//PA1  T 				
			
			/*通用发送函数*/
    		xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle,	//接收任务通知的任务句柄（指定接收方）
    	                	    (uint32_t) GIMBAL_MOTOR_MSG_SIGNAL, 		//任务通知值
    	                	    (eNotifyAction) eSetBits, 					//通知更新方式
    	                	    (uint32_t *)NULL );							//用于保存更新前的任务通知值（为NULL则不保存）
			/*查询自身堆栈的高水位  判断堆栈是否溢出*/
//			gimbal_stack_surplus = uxTaskGetStackHighWaterMark(NULL);//会占用时间 一般在调试的时候使用就好
    		}
    	}
  	}
}

void gimbal_param_init(void)
{
	gimbal.PIT_ONE_REF = PIT_ONE_LEVEL;
	gimbal.YAW_ONE_REF = YAW_ONE_FORWARD;
}

static void gimbal_nomarl_handler(void)
{
//	if(rescue.recuse_flag == RECUSEING && chassis_mode != CHASSIS_RESCUE_MODE)
//	{
//		gimbal.PIT_ONE_REF = PIT_ONE_LEVEL - gimbal.pit_v ;
//		gimbal.YAW_ONE_REF = YAW_ONE_FORWARD;
//	}
//	
	if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE || chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE)
	{
		gimbal.PIT_ONE_REF = PIT_ONE_CLAMP;
		gimbal.YAW_ONE_REF = YAW_ONE_CLAMP;
	}
	else
	{
		gimbal.PIT_ONE_REF = PIT_ONE_LEVEL ;
		gimbal.YAW_ONE_REF = YAW_ONE_FORWARD ;
	}
}
static void gimbal_engineer_handler(void)
{
	if(chassis_mode == CHASSIS_EXCHANGE_MODE)
	{
		gimbal.PIT_ONE_REF = PIT_ONE_LEVEL ;
		gimbal.YAW_ONE_REF = YAW_ONE_ORE ;
	}
	else if(chassis_mode == CHASSIS_GROUND_MODE || chassis_mode == CHASSIS_RESCUE_MODE)
	{
		gimbal.PIT_ONE_REF = PIT_ONE_GROUND;
		gimbal.YAW_ONE_REF = YAW_ONE_ORE ;
	}
	else if(chassis_mode == CHASSIS_CLAMP_CATCH_MODE)
	{
		gimbal.PIT_ONE_REF = PIT_ONE_CATCH ;
		gimbal.YAW_ONE_REF = YAW_ONE_ORE ;
	}
	else
	{
		gimbal.PIT_ONE_REF = PIT_ONE_LEVEL ;
		gimbal.YAW_ONE_REF = YAW_ONE_ORE ;
	}
}

static void gimbal_release_handler(void)
{
	gimbal.PIT_ONE_REF = PIT_ONE_LEVEL;
	gimbal.YAW_ONE_REF = YAW_ONE_ORE;
}
