#include "barrier_carry_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "comm_task.h"
#include "detect_task.h"
#include "upraise_task.h"
#include "pid.h"
#include "bsp_can.h"
#include "math.h"
#include "sys_config.h"
#include "remote_ctrl.h"
#include "rescue_task.h"
#include "keyboard.h"
/*************************************
************障碍块搬运*****************


**** @GPIOI:PIN5（W）  头  
**** @GPIOI:PIN6（X）  尾  
*************************************/
int32_t  BARRIER_CARRY_M1_MAX,BARRIER_CARRY_M2_MAX;
uint8_t barrier_rescue_angle;
uint8_t barrier_mode_frist_in_flag;
extern TaskHandle_t can_msg_send_Task_Handle;

UBaseType_t barrier_carry_stack_surplus;

barrier_carry_t barrier_carry;

void barrier_carry_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_BARRIER_CARRY_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
		if(STAUS == pdTRUE)
		{
			if((Signal & INFO_GET_BARRIER_CARRY_SIGNAL) && chassis_mode != CHASSIS_RELEASE)
			{			
				if(chassis_mode == CHASSIS_BARRIER_CARRY_MODE)
				{
					if(barrier_carry.state == INIT_NEVER)
					{
						barrier_carry_mode = BARRIER_CARRY_INIT;
					}
					
					switch(barrier_carry_mode)
          {
						case BARRIER_CARRY_INIT:
						{
							barrier_carry_init_handler();
						}break;
						case BARRIER_CARRY_ENABLE:
						{						
							barrier_carry_enable_handler();
						}break;
						
						 default:
            {              
            }break;
					}
				}
				else
				{
					barrier_angle_switch = 0;
					barrier_mode_frist_in_flag = 0;
					for(uint8_t i = 0; i < 3;i++)
					{
						BARRIER_OFF
					}
				}
				if((!barrier_carry_is_controllable()))   //没有模块离线
        {		    
        }  
				
				
			  xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                          (uint32_t) BARRIER_CARRY_MOTOR_MSG_SIGNAL, 
                          (eNotifyAction) eSetBits, 
                          (uint32_t *)NULL );
				
				/*调试使用 查看堆栈情况*/
//				barrier_carry_stack_surplus = uxTaskGetStackHighWaterMark(NULL); 
			}
		}
	}
}

void barrier_carry_param_init(void)
{ 
	memset(&barrier_carry, 0, sizeof(barrier_carry_t));//分配内存并初始化
	
	barrier_carry.state               = INIT_NEVER;
	barrier_carry.barrier_carrry_flag = BARRIER_CARRYED;
	
	/*失能*/
   BARRIER_OFF
	
}
void barrier_carry_init_handler(void)
{
		barrier_carry.state = INIT_DONE;
		
		barrier_carry_mode  = BARRIER_CARRY_ENABLE;
}
uint8_t barrier_time;
uint8_t barrier_angle_switch;
//在对障碍快做角度控制时需要通过救援机构的角度辅助控制
void barrier_carry_cmd_time()
{
	
	if(rc.sw1 == RC_MI)
	{
		barrier_angle_switch = 1;
	}
	
	if(!barrier_carry.barrier_carry_cmd )
	{ 
			barrier_time = 0;

				BARRIER_OFF		
	}
	else
	{
		if(barrier_time == 0)
		{
			barrier_time = 1;
			barrier_angle_switch = 0;
		}
		else if(barrier_time == 1 && barrier_angle_switch && rc.sw1 == RC_DN)
		{
			barrier_time = 2;
			barrier_angle_switch = 0;
		}
		else if(barrier_time == 2 && barrier_angle_switch && rc.sw1 == RC_DN)
		{
			barrier_time = 1;
			barrier_angle_switch = 0;
		}
	}

}

/*搬起/放下控制函数*/
void barrier_carry_enable_handler(void)
{
	barrier_carry_cmd_time();
//	/*机构放下对位*/
	if(!barrier_carry.barrier_carry_cmd && barrier_mode_frist_in_flag)
	{
		barrier_rescue_angle = 2;
		BARRIER_OFF
	}
	
	if(barrier_time == 1)
	{
		barrier_mode_frist_in_flag = 1;
		barrier_rescue_angle = 1;//10度
		BARRIER_ON
	}
	else if(barrier_time == 2)
	{
		barrier_mode_frist_in_flag = 1;
		barrier_rescue_angle = 0;
		BARRIER_ON
	}
}


