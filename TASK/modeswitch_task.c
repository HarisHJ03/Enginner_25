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
*******��Դ��������߼���20.12.19��ĳ���޸ģ����н��鼰��������ϵQQ��3155460945��******
***********************************************************************************
//1.���Ȼ�ȡȫ��״̬��Ȼ�󾭲���ѡ������ģʽ
//2.����ģʽ������������ģʽ�Ĺ��ܣ���������ȡ���ϰ�����ˡ���Ԯ���һ����ģʽ
//3.�����жϽ��빤��ģʽ������ģʽ��ͨ���鿴���̵�ģʽ
*********************21.1.31��ʼȫ���޸��˼�ȡģʽ�Ŀ����߼�**************************

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

/*С/����Դ��ģʽ����Ĳ���ģʽ����Ҫ�����ϴε�ģʽ*/
small_island_mode_t small_island_mode;
big_island_mode_t   big_island_mode;

void mode_switch_task(void *parm)
{
	uint32_t mode_switch_wake_time = osKernelSysTick();
	while(1)
	{
		
		//go8010_task();
		get_last_mode();           //��ȡ���������ϴε�ģʽ
		get_main_mode();
		get_chassis_mode();
//		get_rescue_mode();
		get_supply_mode();
		get_clamp_mode();          //ע��Ҫ������ȡ��ʯ̧��
		get_barrier_carry_mode();
		get_gimbal_mode();
		get_monitor_display_mode();//��ȡ��ʾ������ʾ��Ұ
		get_view_switch();
		
//    /*����ң������*/
//    send_rc_data1();
//    send_rc_data2();
//    send_rc_data3();
    
    xTaskGenericNotify( (TaskHandle_t) info_get_Task_Handle, 
                        (uint32_t) MODE_SWITCH_INFO_SIGNAL, 
                        (eNotifyAction) eSetBits, 
                        (uint32_t *)NULL );
    
    mode_switch_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
    
    vTaskDelayUntil(&mode_switch_wake_time, 6);//������ʱ����
  }
}
/*************************************************************
*****1.�����ж�״̬��û�б仯
*****2.״̬���ͱ仯ʱ�ٸ�ֵ�ϴ�״̬
*****3.�ϴ�״̬�͵�ǰ״̬���ֲ�һ��������ǰ״̬�л�ʱ�ٸ����ϴ�״̬

********��˼·ֻ�ڹ��̴�����ģ�����������������¿�ܵ�˼·
***************************************************************/
uint8_t	global_change_state = 1;
	/*�м�仯ģʽ����*/
uint8_t	global_middle_change_mode;
uint8_t chassis_middle_change_mode;
uint8_t rescue_middle_change_mode;
uint8_t clamp_middle_change_mode ;
uint8_t barrier_carry_middle_change_mode;
uint8_t supply_middle_change_mode;

void get_last_mode(void)
{	
	if(global_change_state)//ֻ����һ�Σ���ʼ���ͺ�
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
	if((global_status)global_middle_change_mode != global_mode )                      //��ȡ��һ��ȫ��״̬
	{
		last_global_mode                = (global_status)global_middle_change_mode;
		global_middle_change_mode       = global_mode;
	}
	if((chassis_status)chassis_middle_change_mode !=chassis_mode )                    //��ȡ��һ�ε���״̬
	{
		last_chassis_mode               = (chassis_status)chassis_middle_change_mode;
		chassis_middle_change_mode      = chassis_mode;
		
		/*����̧�� ����ģʽ�仯ʱ ̧���ı䶯�����õ������ƺ������ٶ�*/
		upraise_angle_ctrl_state[0] = 0;
		upraise_angle_ctrl_state[1] = 0;
		upraise.updown_flag        = RAISE; //̧����־
	}
	if(rescue_middle_change_mode != rescue_mode)                                      //��ȡ��һ�ξ�Ԯ״̬
	{
		last_rescue_mode                 = (rescue_status)rescue_middle_change_mode;
		rescue_middle_change_mode        = rescue_mode;
	}
	if((clamp_status)clamp_middle_change_mode != clamp_mode)                          //��ȡ��һ�μ�ȡ״̬
	{
		last_clamp_mode                  = (clamp_status)clamp_middle_change_mode;
		clamp_middle_change_mode         = clamp_mode;
	}
	if((barrier_carry_status)barrier_carry_middle_change_mode != barrier_carry_mode)   //��ȡ��һ�ΰ����ϰ���״̬
	{
		last_barrier_carry_mode          = (barrier_carry_status)barrier_carry_middle_change_mode;
		barrier_carry_middle_change_mode = barrier_carry_mode;
	}
	if((supply_status)supply_middle_change_mode != supply_mode)                        //��ȡ��һ�β���״̬
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
    	global_mode = MANUAL_CTRL;   			//����ģʽ
    }
    break;
    case RC_MI:
    {
    	global_mode = ENGINEER_CTRL;           //����ģʽ����ȡ�һ��ȣ�
    }break;
    case RC_DN:
    {
    	global_mode = RELEASE_CTRL;						//�ϵ�״̬
    }
    break;
    default:
    {
		global_mode = RELEASE_CTRL;
    }break;
  }
}
//extern uint8_t rescue_over;
uint8_t start_big_island_mode_state = 0;//Ϊ�˿��Ƹս������Դ������ʱ��̧ͷ�鿴��Ч��ӽ���ģʽ��־λ

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
		
		/*����������ڴ���ӷ���ģʽ*/
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
	******************��ģʽ���л�����ģʽ˵��*********************
	**** @1.rc.sw1 = RC_UP && glb_sw.last_iw���� �л�����ֱ����Դ��ģʽ
	**** @1.rc.sw1 = RC_UP && glb_sw.last_iw���� �л�����б����Դ��ģʽ

	**** @4.rc.sw1 = RC_MI && glb_sw.last_iw���� �л����һ�ģʽ
	**** @5.rc.sw1 = RC_MI && glb_sw.last_iw���� �л�����¼ģʽ

	**** @6.rc.sw1 = RC_DN && glb_sw.last_iw���� �л������С��Դ����ȡģʽ
	**** @7.rc.sw1 = RC_DN && glb_sw.last_iw���� �л����е���ģʽ
	****************************************************************/
    case ENGINEER_CTRL:
    {
		/*************����ģʽ����ң���л�ģʽ**************/
		if(RC_IW_UP && rc.sw1 == RC_UP)
    {          
       chassis_mode = CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE;    //��ֱ����Դ��ģʽ   
		}
		else if(RC_IW_DN && rc.sw1 == RC_UP)
		{
			chassis_mode = CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE;  	  //��б����Դ����ȡģʽ			
    }
		else if(RC_IW_UP && rc.sw1 == RC_MI)
		{
			chassis_mode = CHASSIS_EXCHANGE_MODE;     				  //�һ�ģʽ	
//				start_exexchange_cmd_state = 1;
		}
		else if(RC_IW_DN && rc.sw1 == RC_MI)
		{
			chassis_mode = CHASSIS_ANGLE_MODE;     						//ǰ��С��Դ����ȡģʽ
		}

		else if(RC_IW_UP && rc.sw1 == RC_DN)
		{
			chassis_mode = CHASSIS_CLAMP_SMALL_ISLAND_MODE;    		  //���С��Դ����ȡģʽ
			
			start_big_island_mode_state = 1;
		}

		else if(RC_IW_DN && rc.sw1 == RC_DN)
     	{        
        	chassis_mode = CHASSIS_CHECK_MODE;		        	 	  //��ȡ����ģʽ
      	}				
			/*�����������л�ģʽ*/
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
//	/*ʹ��ʧ�ܾ�Ԯ*/
//	remote_ctrl_rescue_hook();
////  keyboard_rescue_hook();
//	/*��Ԯʱ����Ҫ̧��ʱֻ������һ��*/
// // upraise.updown_flag = FALL; //�½���־
//	
//	if(chassis_mode != CHASSIS_RESCUE_MODE)
//	{
//    	rescue.upraise_updown_flag = 0;
//	}
//	/*�������Ҫ����Ԯģʽ����Ҫ̧��*/
// 	if(chassis_mode == CHASSIS_RESCUE_MODE)
//	{
//		if(!rescue.upraise_updown_flag)
//		{				
//			rescue.upraise_updown_flag = 1;
//			upraise.updown_flag        = RAISE; //̧����־
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

/*С��Դ��ģʽѡ��*/
/******************************************************
*****************1.С��Դ����ȡģʽ������**************
 @��ͨ��ȡ         -- �����ֶ�λ������ң��ʹ�ܼ�ȡһ��
 @�Ӿ�������ȡһ�� -- ֻ���㷨��ȡһ����Զ��жϼ�ȡ 
 @�Ӿ�������ȡȫ�� -- ȫ�̽����㷨���ݴ���
******************************************************
******** 1.Ĭ�Ͻ�����ͨ��ȡ
******** 2.����ͬ�����ֱ�������ֲ�ͬģʽ*/
uint8_t enter_clamp_state              = 0;  //Ϊ1֤�������˼�ȡģʽ
uint8_t small_island_mode_change_state = 0;  //Ϊ1֤�������˼�ȡģʽ��ͬʱ�Ѳٿ��л���ģʽ

/*��Ϊң�ؽ����ȡģʽʱ��rc.sw1��״̬������ģ����¿���ֱ�ӽ����Ӿ�ģʽ*/
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
////			small_island_mode = SMALL_ISLAND_ORDINARY_MODE;         //Ĭ�Ͻ���С��Դ��ģʽʱ������ͨģʽ				
////		}
////		if(enter_clamp_state)//��һ�ν����ֹ�����Ӿ�ģʽ
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
/*����Դ����ȡģʽѡ��*/
/***********************
**** 1.����Դ����ȡģʽ��Ϊ����
     @��ͨ��ȡ
     @�Ӿ�������ȡһ��
**** 2.����Դ���ĵ����ƶ����ɼ��̰�������
**** 3.�ƶ�����ʯ����λ��ʱ���ɲ����ֲ�����λ��������ȡ�����̿�ʹ�ܸ�ģʽ�µļ�ȡ��
**** 4.�ɾ�ң�ز��������Ӿ�������ȡһ��               ������Ҳ�ɲ��������Ӿ�����ģʽ��
*/
static void get_clamp_big_island_mode(uint8_t big_single_automatic_clamp_one_mode,uint8_t big_island_ordinary_mode)
{
//	if(clamp_mode == BIG_ISLAND)
//	{		
//		if(big_single_automatic_clamp_one_mode)      //�Ӿ�������ȡһ��
//		{
//			big_island_mode = BIG_ISLAND_AUTOMATIC_CLAMP_ONE_MODE;				
//		}
//		if((big_island_ordinary_mode && !KB_BIG_ISLAND_AUTO_SINGLE_CLAMP_STATE)|| big_island_mode != BIG_ISLAND_AUTOMATIC_CLAMP_ONE_MODE)
//		{
//			big_island_mode = BIG_ISLAND_ORDINARY_MODE;//Ĭ�Ͻ������Դ��ģʽʱ������ͨģʽ			
//		}
//	}
//	/*���Ǵ���Դ����ȡģʽʱ������ģʽ�л�����ͨ��ȡģʽ����ֹ�´�ֱ�ӽ����Ӿ�������ȡģʽ*/
//	else
//	{
		big_island_mode = BIG_ISLAND_ORDINARY_MODE;
//	}	
}
/**************��ȡģʽ����********���ڣ�2021.1.15*********/
/**** @1.��ȡ��Ϊ��ͨ��ȡ���㷨���Ӿ�������ȡ
***** @2.����ģʽĬ�Ͻ���Ķ���С/����Դ������ͨ��ȡģʽ

*************����ȡ��Ϊ����Դ����ȡ��С��Դ����ȡ*****���ڣ�2021.1.31*****
*/
/*����̧��*/
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
	
	get_clamp_enter_mode();//����ã�һ�����ȡģʽʱ�����ܳ��ֽ����Ӿ�ģʽ
	
	/*��ȡģʽѡ��*/
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
	
	/*��ȡС��Դ���ļ�ȡģʽ*/
	get_clamp_small_island_mode(RC_SMALL_SINGLE_AUTOMATIC_CLAMP_ONE_MODE,RC_SMALL_SINGLE_AUTOMATIC_MODE,RC_SMALL_SINGLE_ORDINARY_MODE);
	/*��ȡ����Դ���ļ�ȡģʽ*/
	get_clamp_big_island_mode(RC_BIG_SINGLE_AUTOMATIC_CLAMP_ONE_MODE,RC_BIG_SINGLE_ORDINARY_CLAMP_MODE);
	
	/*����̧��*/
	get_clamp_upraise_updown_flag();
	/*����̧������*/
  	if( chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE)
	{
		if(!clamp.small_island_upraise_updown_flag)
		{				
		  	clamp.small_island_upraise_updown_flag = 1;
		 	upraise.updown_flag = RAISE; //̧����־
		}  
	}
	else if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE)
	{
		if(!clamp.big_island_upraise_updown_flag)
		{
			clamp.big_island_upraise_updown_flag = 1;
		  	upraise.updown_flag = RAISE; //̧����־
		}	
	}
	else if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE)
	{
		if(!clamp.big_island_upraise_updown_flag)
		{
			clamp.big_island_upraise_updown_flag = 1;
		  	upraise.updown_flag = RAISE; //̧����־
		}	
	}
	else if(chassis_mode == CHASSIS_EXCHANGE_MODE)
	{
		if(!clamp.exchange_upraise_updown_flag)
		{
			clamp.exchange_upraise_updown_flag = 1;
			upraise.updown_flag = RAISE; //̧����־
		}
	}
	else if(chassis_mode == CHASSIS_GROUND_MODE)
	{
		if(!clamp.ground_upraise_updown_flag)
		{
			clamp.ground_upraise_updown_flag = 1;
			upraise.updown_flag = RAISE; //̧����־
		}		
	}
	else if(chassis_mode == CHASSIS_DEFEND_MODE)
	{
		if(!clamp.defend_upraise_updown_flag)
		{
			clamp.defend_upraise_updown_flag = 1;
			upraise.updown_flag = RAISE; //̧����־
		}
	}
	else if(chassis_mode == CHASSIS_CLAMP_CATCH_MODE)
	{
		if(!clamp.catch_upraise_updown_flag)
		{
			clamp.catch_upraise_updown_flag = 1;
			upraise.updown_flag = RAISE; //̧����־
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
			upraise.updown_flag = RAISE; //̧����־
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
			upraise.updown_flag = RAISE; //̧����־
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
{ /*������ʾ������*/
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







