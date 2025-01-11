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
	PH11 -------> TIM5_CH2   C ����ǰ
	PH10 -------> TIM5_CH1   D ������
	
	*******************************************************
	**** @1.��Ϊ�������������֣�Ĭ�ϵ�һ�β����õ���ǰ���
	        ������һ�β�������Ҫ�����Զ�ת180�Ƚ��в���
	**** @2.�ڶ��β�����ʱ��ս��벹��ģʽ����Ҫ������ת180�ȡ�     
	*******************************************************/
extern TaskHandle_t can_msg_send_Task_Handle;
UBaseType_t supply_stack_surplus;

supply_t supply;

uint8_t supply_ahead_finish = 0;//Ϊ1ʱ֤��ǰ��ĵ����Ѿ��������
uint8_t supply_behind_state = 0;//Ϊ1ʱ˵���´β����Ŀ��ƽ��Ǻ���ĵ���

uint32_t supply_action_times;           //��¼����ʱ�䣬һ��ʱ��󲻾������Զ�����
uint32_t supply_continuous_time = 2500; //���Ʋ�������ʱ���ر�
uint8_t  supplying_state;               //ˢ��supply_action_times��־λ

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
				
        /*��ϵ���ת180�ȵĿ����߼�*/
        if(supply_ahead_finish && chassis_mode != CHASSIS_SUPPLY_MODE )//ǰ��ĵ����Ѳ������
				{
					supply_behind_state = 1;  // �´ν��벹��ʱ��Ҫ�������Ǻ���ĵ��� ����Ҫ��ת180��
				}
				
//        supply_stack_surplus = uxTaskGetStackHighWaterMark(NULL);       
      }
    }

  }
}


void supply_param_init(void)
{
	//����ǰ���������ã�  
	//TIM5 ->CCR2 = 2500;
	
	//�����󲻲������ã�
	//TIM5 ->CCR1 = 1500;  
}

void supply_init_handler(void)
{
	supply_mode = SUPPLY_TO_HERO;//����Ӣ�۲���
}
/*һ��ʱ�����Զ��ջ�*/
void supply_to_hero_handler(void)
{
	/*���Ʋ���ʱ���ˢ��*/
	if(RC_SUPPLY_START_TIMER)
	{
		supplying_state = 1;
	}
	else
	{
		supplying_state = 0;
	}
	/*���������µ�ʱ��ˢ��*/
  if(!supplying_state)
	{
		supply_action_times = HAL_GetTick();
	}
	
  if(supply.supply_cmd1)//ǰ�油����ʼ����
  {
		if( HAL_GetTick() - supply_action_times < supply_continuous_time)//һ��ʱ����Զ��رղ���
		{
			//TIM5 ->CCR2 = 1250;
		}
		else
		{
			/*�������� �����*/
			supply.supply_cmd1  = 0;
			/*���¿�ʼˢ��ʱ��*/
			supplying_state     = 0;
			/*ǰ��ĵ��� �������*/
			supply_ahead_finish = 1;
		}
  }
  else
  {
    //TIM5 ->CCR2 = 2500;
  }
	if(supply.supply_cmd2)//���油����ʼ����
	{
		if( HAL_GetTick() - supply_action_times < supply_continuous_time)//һ��ʱ����Զ��رղ���
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



