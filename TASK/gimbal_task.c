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
#include "stdlib.h" //abs()����
#include "math.h"   //fabs()����

/*************************************************
******************��NEW������**********************
********��ռ�ȺͶ����װ��ʱ�Ƕ��й�**************
*******************��̨ 1 YAW*********************
**** @TIM2_CH1  PA0  S

 ����Ļ 570
 ��ǰ�� 2550
 
 *******************��̨ 1 PIT*********************
**** @TIM2_CH2  PA1  T

 ���1464   ƽ�� 1156   ��С 890
 
*******************��̨ 2 YAW********************* 
**** @TIM2_CH3  PA2  U

����ʯ 1600   ���� 660

**************************************************
*******************��̨ 2 PIT*********************
***  @TIM2_CH4  PA3  V

���  1100  ƽ�� 1350  ��� 1700

*********************************���ڣ�2022��6��30��*/


UBaseType_t gimbal_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

#if (INFANTRY_NUM == INFANTRY_1)

/*��̨1�����λ*/
#define PIT_ONE_MAX      0  	// 1140 ƽ��
#define PIT_ONE_MIN      0

#define YAW_ONE_MAX      0  
#define YAW_ONE_MIN      0

/*�ض��Ƕȿ���*/

#define PIT_ONE_LEVEL    1100   //��̨1ƽ�Ӹ���
#define PIT_ONE_CLAMP    1100   //��̨1��ȡ��С��Դ������
#define PIT_ONE_CATCH  	 800  	//��̨1���սӸ���		�������£���С����
#define PIT_ONE_GROUND   1500  	//��̨1����ȡ�������
#define PIT_ONE_SCREEN	 1350  	//��̨����ʾ������

#define PIT_ONE_CHECK = 0		//��¼

#define YAW_ONE_FORWARD  1055	//��̨1��ǰ������	�������󣬼�С����
#define YAW_ONE_CLAMP  	 1055	//��̨1��ȡ��С��Դ������
#define YAW_ONE_ORE		 1055   //��̨1����ʯ����
#define YAW_ONE_SCREEN   1700   //��̨1����ʾ������

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
		/*�˺������ڻ�ȡֵ֪ͨ�����ֵ֪ͨ��ָ��λֵ*/
    	STAUS = xTaskNotifyWait((uint32_t) NULL, 					//�ȴ�ǰ����ָ������ֵ֪ͨ�ı���λ����ֵ��Ӧbit��0��
								(uint32_t) INFO_GET_GIMBAL_SIGNAL,  //�ɹ��ȴ�������ָ��������ֵ֪ͨ����λ����ֵ��Ӧbit��0��
								(uint32_t *)&Signal, 				//����ȡ��ֵ֪ͨ���������Ҫȡ��������ΪNULL��
								(TickType_t) portMAX_DELAY );		//��������ʱ��
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

    		/*�����λ*/
//			VAL_LIMIT(gimbal.PIT_ONE_REF,PIT_ONE_MIN,PIT_ONE_MAX);
//			VAL_LIMIT(gimbal.YAW_ONE_REF,0,YAW_ONE_MAX);

		    /*��� PWM*/		
			TIM2->CCR1 = gimbal.YAW_ONE_REF+yaw_debug_angle;	//PA0  S
			TIM2->CCR2 = gimbal.PIT_ONE_REF+pit_debug_angle;  	//PA1  T 				
			
			/*ͨ�÷��ͺ���*/
    		xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle,	//��������֪ͨ����������ָ�����շ���
    	                	    (uint32_t) GIMBAL_MOTOR_MSG_SIGNAL, 		//����ֵ֪ͨ
    	                	    (eNotifyAction) eSetBits, 					//֪ͨ���·�ʽ
    	                	    (uint32_t *)NULL );							//���ڱ������ǰ������ֵ֪ͨ��ΪNULL�򲻱��棩
			/*��ѯ�����ջ�ĸ�ˮλ  �ж϶�ջ�Ƿ����*/
//			gimbal_stack_surplus = uxTaskGetStackHighWaterMark(NULL);//��ռ��ʱ�� һ���ڵ��Ե�ʱ��ʹ�þͺ�
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
