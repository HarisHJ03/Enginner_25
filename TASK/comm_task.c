#include "comm_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_can.h"
#include "pid.h"
#include "motor_task.h"
#include "motor_8010.h"
/***********************************

利用CAN发送函数向各个单元发送控制电流

***********************************/
UBaseType_t can_stack_surplus;

//motor_current_t glb_cur;
int16_t CAN1_current[9];
int16_t CAN2_current[9];

uint8_t transfer_8010_flag=0; 

void can_msg_send_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
							(uint32_t) CHASSIS_MOTOR_MSG_SIGNAL | \
		                    			CLAMP_MOTOR_MSG_SIGNAL | \
		                                BARRIER_CARRY_MOTOR_MSG_SIGNAL | \
                                    MODE_SWITCH_MSG_SIGNAL | \
		                                UPRAISE_MOTOR_MSG_SIGNAL | \
																		INFO_SEND_MOTOR_SIGNAL | \
										SLIDE_MOTOR_MSG_SIGNAL,
							(uint32_t *)&Signal, 
							(TickType_t) portMAX_DELAY );
		if(STAUS == pdTRUE)
		{	
		
				
			/********************************************************
			can1底盘   can2云台
			
			can1:
			1234——底盘		56——抬升					7——侧臂伸出
			can2：
			12——云台伸出 	 3——存矿机构伸出 4——侧面夹取臂伸出   56——正前方兑换臂
			********************************************************/
			
			//重新整合发送函数，结构更加合理并更好搭配motor.c
			if(Signal & CHASSIS_MOTOR_MSG_SIGNAL)//发送底盘电流
			{
				send_can1_low_cur(CAN1_current[1],CAN1_current[2],CAN1_current[3],CAN1_current[4]);
			}
			if(Signal & INFO_SEND_MOTOR_SIGNAL)//发送底盘电流
			{
				send_can1_high_cur(CAN1_current[5],CAN1_current[6],CAN1_current[7]);
				send_can2_low_cur(CAN2_current[1],CAN2_current[2],CAN2_current[3],CAN2_current[4]);
				send_can2_high_cur(CAN2_current[5],CAN2_current[6],CAN2_current[7]);
			}
					if(Signal & MODE_SWITCH_MSG_SIGNAL)//关闭遥控 -- 发送电流为零
				{      
					send_can1_low_cur(CAN1_current[1],CAN1_current[2],CAN1_current[3],CAN1_current[4]);
					send_can1_high_cur(CAN1_current[5],CAN1_current[6],CAN1_current[7]);
					send_can2_low_cur(CAN2_current[1],CAN2_current[2],CAN2_current[3],CAN2_current[4]);
					send_can2_high_cur(CAN2_current[5],CAN2_current[6],CAN2_current[7]);
				}
				
				//vTaskDelayUntil(&mode_switch_wake_time, 100);
			/*查询自身堆栈的高水位  判断堆栈是否溢出*/			
			can_stack_surplus = uxTaskGetStackHighWaterMark(NULL);//会占用时间 一般在调试的时候使用就好

		}
  }	
}
