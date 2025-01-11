#include "keyboard.h"
#include "remote_ctrl.h"
#include "STM32_TIM_BASE.h"
#include "sys_config.h"
#include "ramp.h"
#include "chassis_task.h"
#include "clamp_task.h"
#include "rescue_task.h"
#include "supply_task.h"
#include "upraise_task.h"
#include "barrier_carry_task.h"
#include "modeswitch_task.h"
#include "pid.h"
#include "bsp_can.h"
#include "comm_task.h"

#include "clamp.h"

kb_ctrl_t km;
ramp_t key_fbramp;
ramp_t key_rlramp;

/*****************************************************
*                     �� �� �� ��                     *
*                                                     *
*  ��                   _oo0oo_                 ��    *
*                      o8888888o                      *
*  ��                  88" . "88                ̤    *
*                      (| -_- |)                      *
*  ��                  0\  =  /0                ʵ    *
*                    ___/`--- \___                    *
*  ��              .' \\|     |// '.            ��    *
*                 / \\|||  :  |||// \                 *
*  ��            / _||||| -:- |||||- \          ��    *
*               |   | \\\  -  /// |   |               *
*  ʨ           | \_|  ''\---/''  |_/ |         ��    *
*               \  .-\__  '-'  ___/-. /               *
*  ��        ___'. .'  /--.--\  `. .'___        ��    *
*         ."" '<  `.___\_<|>_/___.' >' "".            *
*  ��     | | :  `- \`.;`\ _ /`;.`/ - ` : | |   ��    *
*         \  \ `_.   \_ __\ /__ _/   .-` /  /         *
*    =====`-.____`.___ \_____/___.-`___.-'=====       *
*                       `=---='                       *																			
*                                                     *
*     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~     *
*                                                     *
*               ���汣��         ����BUG               *
******************************************************/

/*21�������̶��ڰ��������Ŀ����߼� 121.2.3*/

/*��ȡ�ϴ�����λ�ü���ǰλ�� ͨ����ʯ��Ų���ȷ���ƶ�λ��*/

/*���水��ǰ�ı��*/
/****************************************
*** @ͨ���ԱȰ���ǰ�ı�źͰ�����ı��
*** @����ͬ����ʹ�ܼ�ȡ
*** @����ͬ����ʹ�ܵ����ƶ�
****************************************/
uint8_t button_press_state =0;              //����������1

/*��ʯ���λ�ÿ��Ƶ����ƶ�*/
uint8_t big_island_chassis_move_done         = 1;//�����ƶ���ָ��λ�ñ�־λ ��Ϊ1ʱ��֤���Ѿ��ƶ���ָ��λ��
int big_island_chassis_move                  = 0;//��Ϊ0ʱ�����������˶�
int big_island_chassis_move_middle_change    = 0;
//static void clamp_big_island_chassis_ctrl()
//{
//	if(button_press_state && big_island_ore_numbering_before_key != big_island_ore_numbering)//�������¼�����ǰ�ı�źͰ�����ı�Ų�һ��
//	{	
//		big_island_chassis_move_done = 0;
//		
//		button_press_state = 0;//�԰��������ˣ�������0
//		
//		big_island_chassis_move = (int)(big_island_ore_numbering - big_island_ore_last_numbering);//��ȡ�ƶ�����
//	}
//}

//}
/*ģʽѡ�� -- ���� ����/�˳� �Ӿ�ģʽ*/
/************************************************************
************ǰ�������������Ѵ��ڲ������жϿ�ʯ����λ��*********
*****@1.�����ֿ�ͨ���ض�����ʹ�������Զ��˶���ָ��λ��
*****@2.����������������ƶ����̣���ͨ�� �̰� SHIFT�� ���� �Ӿ�������ȡģʽ
*****@2.����������������ƶ����̣���ͨ�� ���� SHIFT�� �˳� �Ӿ�������ȡģʽ
*/
uint8_t  KB_BIG_ISLAND_AUTO_SINGLE_CLAMP_STATE = 0;//���ȼ�
//uint16_t big_island_shift_button_press_time    = 0;//��Ϊ�Ӿ�������ȡʱ���ɳ����˳�����ͨ��ȡģʽ
//static void clamp_big_island_mode_ctrl()//2021.2.27 �ú����༭���Ժ�ʱ5Сʱ
//{
//	if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE && big_island_chassis_move_done)/*/�����ƶ����*/
//	{
//		if(KB_BIG_ISLAND_AUTO_SINGLE_CLAMP_CMD)	
//		{					
//			big_island_shift_button_press_time++;//����ʱ��
//			
//			if(big_island_shift_button_press_time >= 300)//����SHIFT ������ͨ��ȡģʽ
//			{
//				big_island_mode = BIG_ISLAND_ORDINARY_MODE;
//				
//				KB_BIG_ISLAND_AUTO_SINGLE_CLAMP_STATE = 0;				
//			}				
//		}
//		else
//		{
//			if(big_island_shift_button_press_time < 300 && big_island_shift_button_press_time > 10)//�̰�
//			{
//				big_island_mode = BIG_ISLAND_AUTOMATIC_CLAMP_ONE_MODE;
//				
//				KB_BIG_ISLAND_AUTO_SINGLE_CLAMP_STATE = 1;                          //�Ӹñ�־λ��Ϊ�ˣ���ʹң�ز����ǽ�����ͨ��ȡ��Ҳ�ܾ����̲��������Ӿ�������ȡһ�䣨�����̲�����ң�ز������ȼ����ߣ�
//			}
//			
//			big_island_shift_button_press_time = 0;
//		}
//	}
//	else
//	{
//		big_island_mode = BIG_ISLAND_ORDINARY_MODE;//�˳������½���Ĭ������ͨģʽ
//	}
//	
//}
uint8_t INIT_STATE       = 0;//��ʼ�����е����־λ
int32_t INIT_ACTION_TIME = 0;
int32_t debug_time = 0;
static void chassis_kb_operation_func(rc_info_t rc);
void keyboard_clamp_hook(void)
{

	/*�һ���������Ϊ����*/
	keyboard_exchange_hook();
	keyboard_init_hook();
	kb_adrust_angle_ctrl();
	// keyboard_ore_adjust_hook();
}



/*�һ���������Ϊ����*/
uint8_t kb_pick_box_state=1;;
void keyboard_exchange_hook(void)
{
	if(km.kb_enable)
	{
		if(KB_HAVE_BOX_INC)
		{
			if(kb_pick_box_state)
			{
				have_box_number++;
				kb_pick_box_state=0;
			}
		}
		
		else if(KB_HAVE_BOX_DEC)
		{
			if(kb_pick_box_state)
			{
				have_box_number--;
				kb_pick_box_state=0;
			}
		}
		else kb_pick_box_state=1;
	}
}

int32_t kb_slide_delay_time;
int32_t kb_uprise_delay_time;

int32_t kb_manipulator_delay_times[7];
int16_t kb_slide_angle[2] = {0};
int16_t kb_upraise_angle[2] = {0};

int16_t kb_adrust_angle[6]={0};

uint8_t kb_turn_state ;
/************************ 2023�������� **********************/
//����Ҫ��debug��һ��
int16_t kb_manipulator_adjust_angle[7] = {0};//��е�۽Ƕ�

/*******************24����************************************/
kb_adjust_t kb_adrust[6];
kb_adjust_t kb_slide_clamp_del;
void kb_kb_adrust_angle_change(kb_adjust_t *kb_adrust,int8_t flag,uint16_t delay_times)
{
	if((HAL_GetTick() - kb_adrust->delay_times) >delay_times )
	{
		kb_adrust->angle+=flag;
		kb_adrust->delay_times=HAL_GetTick();
	}
}
//void kb_kb_adrust_angle_change_test(kb_adjust_t *kb_adrust,int8_t flag,uint16_t delay_times)
//{
//	if((HAL_GetTick() - kb_adrust->delay_times) >delay_times )
//	{
//		kb_adrust->angle+=flag;
//		kb_adrust->delay_times=HAL_GetTick();
//	}
//}
void kb_adrust_angle_ctrl(void)//��е�۽Ƕȿ���
{
	if(km.kb_enable && (chassis_mode == CHASSIS_GROUND_MODE || 
	chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE ||
	chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE ||	
	chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE ||
	chassis_mode == CHASSIS_ANGLE_MODE ||
	chassis_mode == CHASSIS_EXCHANGE_MODE))
	{
		if(KB_SLIDE_FORWARD)
		{
			kb_kb_adrust_angle_change(&kb_adrust[ADRUST_X],1,3);
		}
		if(KB_SLIDE_BACK)
		{
			kb_kb_adrust_angle_change(&kb_adrust[ADRUST_X],-1,3);
		}
		if(KB_TRAVERSE_LEFT)
		{
			kb_kb_adrust_angle_change(&kb_adrust[ADRUST_Y],1,1);
		}
		if(KB_TRAVERSE_RIGHT)
		{
			kb_kb_adrust_angle_change(&kb_adrust[ADRUST_Y],-1,1);
		}
		if(KB_UPRAISE_UP)
		{
			kb_kb_adrust_angle_change(&kb_adrust[ADRUST_Z],4,1);
		}
		if(KB_UPRAISE_DN)
		{
			kb_kb_adrust_angle_change(&kb_adrust[ADRUST_Z],-4,1);
		}
		////////////////////////////////////////////
		if(KB_YAW_LEFT)
		{
			kb_kb_adrust_angle_change(&kb_adrust[ADRUST_YAW],-1,60);
		}
		if(KB_YAW_RIGHT)
		{
			kb_kb_adrust_angle_change(&kb_adrust[ADRUST_YAW],+1,60);
		}
		if(KB_PITCH_UP)
		{
			kb_kb_adrust_angle_change(&kb_adrust[ADRUST_PITCH],-2,1);
		}
		if(KB_PITCH_DN)
		{
			kb_kb_adrust_angle_change(&kb_adrust[ADRUST_PITCH],+2,1);
		}
		if(KB_ROLL_LEFT)
		{
			kb_kb_adrust_angle_change(&kb_adrust[ADRUST_ROLL],1,10);
		}
		if(KB_ROLL_RIGHT)
		{
			kb_kb_adrust_angle_change(&kb_adrust[ADRUST_ROLL],-1,10);
		}
		
		if(KB_CLAMP_SLIDE_DEL)
		{
			kb_kb_adrust_angle_change(&kb_slide_clamp_del,1,10);
		}
	}
	else if (chassis_mode == CHASSIS_NORMAL_MODE)
	{
		memset(kb_adrust,0,sizeof(kb_adrust));
	}
}

void Stm32_SoftReset(void)
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

void keyboard_init_hook()
{
	if(km.kb_enable && chassis_mode == CHASSIS_RELEASE)
	{
		if(KB_RESET_INIT)
			Stm32_SoftReset();
	}		
}
static void chassis_direction_ctrl(uint8_t forward, uint8_t back,uint8_t left, uint8_t right)
{
	if (back)
	{
		km.vx = 1;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
	else if (forward)
	{
		km.vx = -1;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
	else
	{
		km.vx = 0;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}

	if (right)
	{
		km.vy = 1;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
	else if (left)
	{
		km.vy = -1;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
	else
	{
		km.vy = 0;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
}
void keyboard_chassis_hook(void)
{
	if (km.kb_enable)
	{
		chassis_direction_ctrl(FORWARD, BACK, LEFT, RIGHT);
		chassis_kb_operation_func(rc);
	}
		
	else
	{
		km.vx = 0;
		km.vy = 0;
	}
}

static void chassis_kb_operation_func(rc_info_t rc)//�����˶����ƺ���
{
	if(rc.kb.bit.W)
	{
		km.vy = -rc.kb.bit.W  * CHASSIS_KB_MAX_SPEED_Y * CHASSIS_KB_MOVE_RATIO_Y;
	}
	if(rc.kb.bit.S)
	{
		km.vy = rc.kb.bit.S  * CHASSIS_KB_MAX_SPEED_Y * CHASSIS_KB_MOVE_RATIO_Y;
	}	
	if(rc.kb.bit.A)
	{
		km.vx =  -rc.kb.bit.A  * CHASSIS_KB_MAX_SPEED_X * CHASSIS_KB_MOVE_RATIO_X;
	}	
  if(rc.kb.bit.D)
	{
		km.vx =  rc.kb.bit.D  * CHASSIS_KB_MAX_SPEED_X * CHASSIS_KB_MOVE_RATIO_X;
	}
 
	km.vw = rc.mouse.x / RC_RESOLUTION * CHASSIS_KB_MAX_SPEED_R * CHASSIS_KB_MOVE_RATIO_R*0.5;//����0.5����Ϊ����ʱ�о�ת���������
}
// void keyboard_ore_adjust_hook()
// {
// 	if(KB_ORE_ADJUST)
// 	{
// 		ore_adjust_state = 1;
// 	}
// }


