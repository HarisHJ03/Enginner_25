#include "supply_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "remote_ctrl.h"
#include "upraise_task.h"
#include "comm_task.h"
#include "pid.h"
#include "math.h"
#include "bsp_can.h"
/*******************************************
	PH11 -------> TIM5_CH2   C 补给前
	PH10 -------> TIM5_CH1   D 补给后
	
	*******************************************************
	**** @1.因为有两个补给弹仓，默认第一次补给用的是前面的
	        即，第一次补给不需要控制自动转180度进行补给
	**** @2.第二次补给的时候刚进入补给模式就需要控制旋转180度。     
	*******************************************************/
extern TaskHandle_t can_msg_send_Task_Handle;
UBaseType_t supply_stack_surplus;

supply_t supply;

uint8_t supply_ahead_finish = 0;//为1时证明前面的弹仓已经补给完毕
uint8_t supply_behind_state = 0;//为1时说明下次补给的控制将是后面的弹仓

uint32_t supply_action_times;           //记录补给时间，一定时间后不经操作自动缩回
uint32_t supply_continuous_time = 2500; //控制补给持续时间后关闭
uint8_t  supplying_state;               //刷新supply_action_times标志位

void supply_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_SUPPLY_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
		if(STAUS == pdTRUE)
		{
			if(Signal & INFO_GET_SUPPLY_SIGNAL)
			{
        if(chassis_mode == CHASSIS_SUPPLY_MODE && upraise.updown_flag == UP)
        {          
          switch(supply_mode)
          {
						case SUPPLY_INIT:
						{
							supply_init_handler();
						}
            
            case SUPPLY_TO_HERO:
            {
              supply_to_hero_handler();
            }break;
            
            default:
            {
              
            }break;           
          }         
        }
				else
				  supply_param_init();
				
        /*配合底盘转180度的控制逻辑*/
        if(supply_ahead_finish && chassis_mode != CHASSIS_SUPPLY_MODE )//前面的弹仓已补给完毕
				{
					supply_behind_state = 1;  // 下次进入补给时需要补给的是后面的弹仓 即需要旋转180°
				}
				
//        supply_stack_surplus = uxTaskGetStackHighWaterMark(NULL);       
      }
    }

  }
}


void supply_param_init(void)
{
	//补给前不补给配置：  
	//TIM5 ->CCR2 = 2500;
	
	//补给后不补给配置：
	//TIM5 ->CCR1 = 1500;  
}

void supply_init_handler(void)
{
	supply_mode = SUPPLY_TO_HERO;//仅给英雄补给
}
/*一定时间内自动收回*/
void supply_to_hero_handler(void)
{
	/*控制补给时间的刷新*/
	if(RC_SUPPLY_START_TIMER)
	{
		supplying_state = 1;
	}
	else
	{
		supplying_state = 0;
	}
	/*满足条件下的时间刷新*/
  if(!supplying_state)
	{
		supply_action_times = HAL_GetTick();
	}
	
  if(supply.supply_cmd1)//前面补给开始补给
  {
		if( HAL_GetTick() - supply_action_times < supply_continuous_time)//一定时间后将自动关闭补给
		{
			//TIM5 ->CCR2 = 1250;
		}
		else
		{
			/*补给动作 已完成*/
			supply.supply_cmd1  = 0;
			/*重新开始刷新时间*/
			supplying_state     = 0;
			/*前面的弹仓 补给完毕*/
			supply_ahead_finish = 1;
		}
  }
  else
  {
    //TIM5 ->CCR2 = 2500;
  }
	if(supply.supply_cmd2)//后面补给开始补给
	{
		if( HAL_GetTick() - supply_action_times < supply_continuous_time)//一定时间后将自动关闭补给
		{
		//	TIM5 ->CCR1 = 2500;
		}
		else
		{
			//TIM5 ->CCR1 = 1500;
			
			supply.supply_cmd2  = 0;
			supplying_state     = 0;
		} 
	}
	else
	{  
		// TIM5 ->CCR1 = 1500;
	}
} 



