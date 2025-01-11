#include "chassis_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "clamp_task.h"
#include "upraise_task.h"
#include "info_get_task.h"
#include "detect_task.h"
#include "pid.h"
#include "sys_config.h"
#include "stdlib.h"
#include "math.h"
#include "pc_rx_data.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "arm_math.h"
#include "ramp.h"
#include "bsp_can.h"
#include "rescue_task.h"
#include "supply_task.h"
#include "manipulator_task.h"

#include "keyboard.h"
#include "controller.h"

/*********************24ע��****************************
	���ݹ�ȥ�����ϰ�ߣ�x��ǰ��y������



**************************************************/
UBaseType_t chassis_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;
//extern uint8_t rescue_over;
/*����Դ�������ƶ���ָ��λ��*/
int16_t chassis_big_island_move_one_y_spd;
int32_t chassis_big_island_move_one_init_angle;
int32_t chassis_big_island_move_one_ref_angle = 360*0.6;
uint8_t big_island_chassis_move_change_state  = 0;//�����ƶ������ı��������仯״̬
/*�㷨ʶ���ʯ���ñ���*/
uint16_t OUT_OF_SIGHT       = 999;      //�㷨ʶ�𲻵�������ֵ999
uint8_t  out_of_sight_state = 0;        //�����㷨��Ұ��

uint8_t  pc_recv_mesg_judge_state = 0; //�㷨��λ�Ƿ���ɱ�־λ

int16_t out_of_sight_number[5] ={0};                  //���յ��㷨����������Ч��־λ�����������Ϊ��Ч
uint8_t  out_of_sight_numb;                           //��¼���µ���������г����˶��ٸ�out_of_sight�������ж��Ƿ����Ϊ�ǲ����㷨����Ұ��

uint8_t exchange_ctrl_chassis_state;
uint32_t chassis_action_times;

/*�ϴε�ͷ����*/
Direction_t Last_chassi_direction=AHEAD_BACK;
Direction_t Now_chassi_direction=AHEAD_BACK;

int16_t exchange_x_angle_move =70;
int16_t exchange_y_angle_move;

uint8_t chassis_speed_or_angle_flag=0;
/*�����˶�����*/
chassis_t chassis;
ramp_t chassis_ramp;

//float chassis_pid[6] = {13.0f, 0.0f, 90.0f, 4.5f, 0.05, 0};
float chassis_pid[6] = {40.0f, 0.05f, 0.0f, 4.5f, 0.05, 0};
void chassis_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_CHASSIS_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
    if(STAUS == pdTRUE)
		{
			if(Signal & INFO_GET_CHASSIS_SIGNAL)
			{
        /*����vw��ת��pid*/
        PID_Struct_Init(&pid_chassis_angle,chassis_pid[0],chassis_pid[1],chassis_pid[2],1000, 50, DONE);
        /*����vx,vyƽ�Ƶ�pid*/
        for(int i = 0; i < 4; i++)
        {
          PID_Struct_Init(&pid_spd[i],chassis_pid[3],chassis_pid[4],chassis_pid[5],20000, 2000, DONE);
        }
        
        if(chassis_mode != CHASSIS_RELEASE )//��̨����֮����̲��ܶ�  && gimbal.state != GIMBAL_INIT_NEVER
        {
          switch(chassis_mode)
          {
            case CHASSIS_NORMAL_MODE:
						case CHASSIS_BARRIER_CARRY_MODE:
						case CHASSIS_RESCUE_MODE:
            case CHASSIS_SUPPLY_MODE:
						case CHASSIS_DEFEND_MODE:
						case CHASSIS_CHECK_MODE:
						{
							chassis_ahead_to_back();
						}break;
					  case CHASSIS_GROUND_MODE:
						{
							chassis_ahead_to_front();
						}break;
						case CHASSIS_CLAMP_CATCH_MODE:
            {
              chassis_ahead_to_front();
            }break;
						
            case CHASSIS_EXCHANGE_MODE:
						{
							//chassis_ahead_to_back();
							chassis_exchange_handler();
						}break;
            case CHASSIS_ANGLE_MODE:
            {
              chassis_exchange_angle_handler();
            }break;
						/*С��Դ����ȡʱ�����̿���*/
            case CHASSIS_CLAMP_SMALL_ISLAND_MODE:
            {
							if(small_island_mode == SMALL_ISLAND_AUTOMATIC_CLAMP_ONE_MODE)
							{
								//chassis_auto_clamp_one_mode_handler();
								chassis_ahead_to_front();
							}							
							else if(small_island_mode == SMALL_ISLAND_AUTOMATIC_MODE)//��������
              {
								//chassis_small_auto_mode_handler();
								chassis_ahead_to_front();
							}
							else
							{
								chassis_ahead_to_front();
							}
            }break;
						/*����Դ����ȡʱ�����̿���*/
						 case CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE:
            {
							chassis_ahead_to_front();
            }break;
						 case CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE:
            {
							chassis_ahead_to_front();
            }break;
            
            /*����ֹͣģʽ*/
            case CHASSIS_STOP_MODE:
            {
              chassis_stop_handler();
            }break;
            default:
            {
							chassis_stop_handler();
            }break;
          }
					
					if(chassis_mode != CHASSIS_ANGLE_MODE )
						chassis_speed_or_angle_flag=0;
          mecanum_calc(chassis.vx, chassis.vy, chassis.vw, chassis.wheel_spd_ref);

//					if(small_island_mode    != SMALL_ISLAND_AUTOMATIC_CLAMP_ONE_MODE 
//						 && small_island_mode != SMALL_ISLAND_AUTOMATIC_MODE
//						 && big_island_mode   != BIG_ISLAND_AUTOMATIC_CLAMP_ONE_MODE)
//					{
//						clamp.identify_cmd = 0;                 //�ɿ�ʼ��ȡ������־λ,Ϊ1ʱ�㷨ʶ����ϣ�����˶����
//					}
					
//					if(chassis_mode!=CHASSIS_EXCHANGE_MODE) 
//					{
//						chassis_speed_or_angle_flag=0;
//						kb_exchange_mode=SPEED_MODE;
//					}
					if(chassis_speed_or_angle_flag==0)
					{//�ٶȻ�
						for (int i = 0; i < 4; i++)
						{
							chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
							chassis.wheel_angle_offset[i]=chassis.wheel_angle_fdb[i];//ˢ�½ǶȻ�
						}
					}
					else if(chassis_speed_or_angle_flag==1)
						for (int i = 0; i < 4; i++)
						{//�ǶȻ�
							chassis.wheel_spd_ref[i]=pid_calc(&pid_chassis_angle,chassis.wheel_angle_fdb[i],chassis.wheel_angle_ref[i]);
							chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
						}
          if (!chassis_is_controllable())//����
          {
            memset(chassis.current, 0, sizeof(chassis.current));
          }
          //������ʱ���ģ���˵���can�䶯�����ֶ�����
          memcpy(&CAN1_current[1], chassis.current, sizeof(chassis.current));
        }
        else
        {
          memset(&CAN1_current[1], 0, sizeof(chassis.current));//��������Ļ�ֻ�������8���ֽ�
        }
       
        xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                  (uint32_t) CHASSIS_MOTOR_MSG_SIGNAL, 
                  (eNotifyAction) eSetBits, 
                  (uint32_t *)NULL );
      }
    }
    
//    chassis_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}
uint32_t Turn_Record_times;
static uint8_t chassis_Auto_Turn()
{
	uint8_t Turn_ready_flag=0;
	int16_t Turn_speed=300;
	if(Now_chassi_direction!=Last_chassi_direction)
	{
		Turn_Record_times=HAL_GetTick();
		Last_chassi_direction=Now_chassi_direction;
	}
	if(HAL_GetTick()-Turn_Record_times <= 700 && HAL_GetTick()>10000)
	{
		switch(Now_chassi_direction)
		{
			case AHEAD_BACK:
			{
				chassis.vw= -Turn_speed;
			}break;
			case AHEAD_FRONT:
			{
				chassis.vw= +Turn_speed;
			}break;
		}
		Turn_ready_flag=0;
	}
	else 
		Turn_ready_flag=1;
	return Turn_ready_flag;
}

extern uint8_t gimbal_ctrl;
static void chassis_ahead_to_back(void)
{
	#ifdef BACK_DRIVE
	Now_chassi_direction=AHEAD_BACK;
	chassis.vy = +(rm.vx * CHASSIS_RC_MOVE_RATIO_X + km.vx * CHASSIS_KB_MOVE_RATIO_X);
  chassis.vx = +(rm.vy * CHASSIS_RC_MOVE_RATIO_Y + km.vy * CHASSIS_KB_MOVE_RATIO_Y);
	if(chassis_Auto_Turn() || NOT_AUTO_TURN)
		chassis.vw = -rm.vw * CHASSIS_RC_MOVE_RATIO_R ;
	#else
		chassis_ahead_to_front();
	#endif
//	if(gimbal_ctrl == 1)
//	{
//		chassis.vw = 0;
//	}
}
static void chassis_ahead_to_front(void)
{
	Now_chassi_direction=AHEAD_FRONT;
	chassis.vy = -(rm.vx * CHASSIS_RC_MOVE_RATIO_X + km.vx * CHASSIS_KB_MOVE_RATIO_X);
  chassis.vx = -(rm.vy * CHASSIS_RC_MOVE_RATIO_Y + km.vy * CHASSIS_KB_MOVE_RATIO_Y);
	#ifdef BACK_DRIVE
	if(chassis_Auto_Turn() || NOT_AUTO_TURN)
	#endif
	chassis.vw = -(rm.vw * CHASSIS_RC_MOVE_RATIO_R + km.vw * CHASSIS_KB_MOVE_RATIO_R);
	
//	if(gimbal_ctrl == 1)
//	{
//		chassis.vw = 0;
//	}
}
//uint32_t chassis_exchange_action_time;
//int16_t  chassis_exchange_action_spd;
uint8_t chassis_exchange_action;						//�һ�ʱ�����˶�ʹ�ܱ�־λ	0������1��
uint32_t chassis_exchange_action_time;			//�����˶�ʱ��
int16_t exchange_chassis_spd;								//�����˶��ٶȺͷ���
uint8_t chassis_exchange_flag;							//����ǰ���ͺ��˵ı�־λ
static void chassis_exchange_handler(void)
{
//	switch(kb_exchange_mode)
//	{
//		case SPEED_MODE:
//		{
			chassis.vy = -(rm.vx * CHASSIS_RC_MOVE_RATIO_X + km.vx * CHASSIS_KB_MOVE_RATIO_X);
			chassis.vx = -(rm.vy * CHASSIS_RC_MOVE_RATIO_Y + km.vy * CHASSIS_KB_MOVE_RATIO_Y);
			chassis.vw = -rm.vw * CHASSIS_RC_MOVE_RATIO_R ;
			chassis_speed_or_angle_flag=0;
//		}break;
//		case ANGLE_MODE:
//		{
//			exchange_x_angle_move = +(model_out_angle[1]);
//			chassis.wheel_angle_ref[0]=(chassis.wheel_angle_offset[0]-exchange_x_angle_move)*ANGLE_RATIO_FR;
//			chassis.wheel_angle_ref[1]=(chassis.wheel_angle_offset[1]-exchange_x_angle_move)*ANGLE_RATIO_FL;
//			chassis.wheel_angle_ref[2]=(chassis.wheel_angle_offset[2]+exchange_x_angle_move)*ANGLE_RATIO_BR;
//			chassis.wheel_angle_ref[3]=(chassis.wheel_angle_offset[3]+exchange_x_angle_move)*ANGLE_RATIO_BL;
//			chassis_speed_or_angle_flag=1;
//		}break;
//	}
}
/*********************************************
	���̽ǶȻ�ģʽ���ƴ���
***********************************************/
static void chassis_exchange_angle_handler(void)
{
	exchange_x_angle_move = +(model_out_angle[0]+kb_adrust[ADRUST_X].angle);
	exchange_y_angle_move = +(model_out_angle[1]-kb_adrust[ADRUST_Y].angle);
	chassis.wheel_angle_ref[0]=(chassis.wheel_angle_offset[0]+exchange_y_angle_move-exchange_x_angle_move)*ANGLE_RATIO_FR;
	chassis.wheel_angle_ref[1]=(chassis.wheel_angle_offset[1]+exchange_y_angle_move+exchange_x_angle_move)*ANGLE_RATIO_FL;
	chassis.wheel_angle_ref[2]=(chassis.wheel_angle_offset[2]-exchange_y_angle_move+exchange_x_angle_move)*ANGLE_RATIO_BR;
	chassis.wheel_angle_ref[3]=(chassis.wheel_angle_offset[3]-exchange_y_angle_move-exchange_x_angle_move)*ANGLE_RATIO_BL;
	chassis_speed_or_angle_flag=1;
}
/*��Ԯ��������ת180�㺯��   ���ڣ�2021��5��19��*/
/*********************************��Ԯ********************************
**** @1.�ѽ����Ԯģʽ �ҵ�ǰ��û�о������������� ��ʱ��Ҫת180��
**** @2.�ϴξ������������ˣ���������ʱ����Ҫת�����Ʒſ�������Ҳ����Ҫת
**** @3.����ʲô�����ң�������ܿ��Ƶ��̵���Ӧ�˶�
**********************************����*******************************
**** @4.��������ĵ���ʱ��Ҳ��Ҫ����ת��180��
**** @5.ʹ�ܲ���ʱ���ƶ���
*********************************************************************/
int  supply_jitter_spd;             //���ƶ����ٶȸ���
uint8_t supply_change_direction = 0;//���Ʊ���

/********************�Ӿ�������ȡ********************************
*******1.����ȷ���Ӿ�����ʱ��Ҫ������翪�ز��
*******2.��������Ұ����������ʱֻ�����������ɨ����������������������
*******3.identify_done_stateΪ1ʱ��������ȡ����
*******4.PI2 -->��翪��Զ���루Z��.��⵽Ϊ�͵�ƽ
*******5.PI7 -->��翪�ؽ����루Y��
******************************************���ڣ�2021.1.16********/
uint8_t get_out_of_sight_state_handler(void)
{
	/*������ν��յ��㷨����Ϊ������Ұ�ڽ����ж�Ӧ����*/
	out_of_sight_number[0] = pc_recv_mesg.gimbal_control_data.y_distance;
	for(int i = 4;i > 0; i--)
	{
		out_of_sight_number[i] = out_of_sight_number[i-1];
	}		
	for(int i = 0;i < 5;i++)
	{
		if(out_of_sight_number[i] == OUT_OF_SIGHT)
		{
			out_of_sight_numb++;
		}
		if(out_of_sight_numb == 5 && i== 4)
		{
			out_of_sight_state = 1;
		}
		else
		{
			out_of_sight_state = 0;
		}
		if(i==4)
		{		
			out_of_sight_numb = 0;
		}
	}
	return out_of_sight_state;
}

static void chassis_stop_handler(void)
{
  chassis.vy = 0;
  chassis.vx = 0;
  chassis.vw = 0;
}

void chassis_param_init(void)
{
  memset(&chassis,0,sizeof(chassis));
  
  ramp_init(&chassis_ramp,1000);
  
  /*����vw��ת��pid*/
  PID_Struct_Init(&pid_chassis_angle,chassis_pid[0],chassis_pid[1],chassis_pid[2],MAX_CHASSIS_VR_SPEED, 50, INIT);
  /*����vx,vyƽ�Ƶ�pid*/
  for(int i = 0; i < 4; i++)
    PID_Struct_Init(&pid_spd[i],chassis_pid[3],chassis_pid[4],chassis_pid[5],10000, 500, INIT);
  
}



/**
  * @brief mecanum chassis velocity decomposition
  * @param input : ��=+vx(mm/s)  ��=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm)
	* @trans ���룺		ǰ�����ҵ���
	*				 �����		ÿ�����Ӷ�Ӧ���ٶ�
  * @note  1=FR 2=FL 3=BL 4=BR
	* @work	 �������㹫ʽ�����Ч��
  */
int rotation_center_gimbal = 0;
static void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
  static float rotate_ratio_fr;
  static float rotate_ratio_fl;
  static float rotate_ratio_bl;
  static float rotate_ratio_br;
  static float wheel_rpm_ratio;
  
  taskENTER_CRITICAL();//�����жϻ�����
	/*21����û��С����*/
//  if(chassis_mode == CHASSIS_DODGE_MODE || chassis.dodge_ctrl)
//  {
//    chassis.rotate_x_offset = GIMBAL_X_OFFSET;
//    chassis.rotate_y_offset = 0;
//  }
//  else
//  {
    if (rotation_center_gimbal)
    {
      chassis.rotate_x_offset = glb_struct.gimbal_x_offset;
      chassis.rotate_y_offset = glb_struct.gimbal_y_offset;
    }
    else
    {
      chassis.rotate_x_offset = 0;
      chassis.rotate_y_offset = 0;
    }
//  }
  //@work
  rotate_ratio_fr = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f \
                      - chassis.rotate_x_offset + chassis.rotate_y_offset)/RADIAN_COEF;
  rotate_ratio_fl = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f \
                      - chassis.rotate_x_offset - chassis.rotate_y_offset)/RADIAN_COEF;
  rotate_ratio_bl = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f \
                      + chassis.rotate_x_offset - chassis.rotate_y_offset)/RADIAN_COEF;
  rotate_ratio_br = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f \
                      + chassis.rotate_x_offset + chassis.rotate_y_offset)/RADIAN_COEF;

  wheel_rpm_ratio = 60.0f/(glb_struct.wheel_perimeter*CHASSIS_DECELE_RATIO);
  taskEXIT_CRITICAL();//�뿪�жϻ�����
  

  VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
  VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
	/*С���������ģʽ��vw��������*/
	//if(chassis_mode != CHASSIS_DODGE_MODE && !chassis.dodge_ctrl)
    VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s
	/*С����ʱ��vw��������*/
//	else
//		VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

  int16_t wheel_rpm[4];
  float   max = 0;
  
  wheel_rpm[0] = (-vx - vy - vw * rotate_ratio_fr) * wheel_rpm_ratio;
  wheel_rpm[1] = ( vx - vy - vw * rotate_ratio_fl) * wheel_rpm_ratio;
  wheel_rpm[2] = ( vx + vy - vw * rotate_ratio_bl) * wheel_rpm_ratio;
  wheel_rpm[3] = (-vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;

  //find max item
  for (uint8_t i = 0; i < 4; i++)
  {
    if (abs(wheel_rpm[i]) > max)
      max = abs(wheel_rpm[i]);
  }
  //equal proportion
  if (max > MAX_WHEEL_RPM)
  {
    float rate = MAX_WHEEL_RPM / max;
    for (uint8_t i = 0; i < 4; i++)
      wheel_rpm[i] *= rate;
  }
  memcpy(speed, wheel_rpm, 4*sizeof(int16_t));
}
