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
************�ϰ������*****************


**** @GPIOI:PIN5��W��  ͷ  
**** @GPIOI:PIN6��X��  β  
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
				if((!barrier_carry_is_controllable()))   //û��ģ������
        {		    
        }  
				
				
			  xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                          (uint32_t) BARRIER_CARRY_MOTOR_MSG_SIGNAL, 
                          (eNotifyAction) eSetBits, 
                          (uint32_t *)NULL );
				
				/*����ʹ�� �鿴��ջ���*/
//				barrier_carry_stack_surplus = uxTaskGetStackHighWaterMark(NULL); 
			}
		}
	}
}

void barrier_carry_param_init(void)
{ 
	memset(&barrier_carry, 0, sizeof(barrier_carry_t));//�����ڴ沢��ʼ��
	
	barrier_carry.state               = INIT_NEVER;
	barrier_carry.barrier_carrry_flag = BARRIER_CARRYED;
	
	/*ʧ��*/
   BARRIER_OFF
	
}
void barrier_carry_init_handler(void)
{
		barrier_carry.state = INIT_DONE;
		
		barrier_carry_mode  = BARRIER_CARRY_ENABLE;
}
uint8_t barrier_time;
uint8_t barrier_angle_switch;
//�ڶ��ϰ������Ƕȿ���ʱ��Ҫͨ����Ԯ�����ĽǶȸ�������
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

/*����/���¿��ƺ���*/
void barrier_carry_enable_handler(void)
{
	barrier_carry_cmd_time();
//	/*�������¶�λ*/
	if(!barrier_carry.barrier_carry_cmd && barrier_mode_frist_in_flag)
	{
		barrier_rescue_angle = 2;
		BARRIER_OFF
	}
	
	if(barrier_time == 1)
	{
		barrier_mode_frist_in_flag = 1;
		barrier_rescue_angle = 1;//10��
		BARRIER_ON
	}
	else if(barrier_time == 2)
	{
		barrier_mode_frist_in_flag = 1;
		barrier_rescue_angle = 0;
		BARRIER_ON
	}
}


