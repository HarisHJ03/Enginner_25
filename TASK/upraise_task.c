#include "upraise_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "comm_task.h"
#include "clamp_task.h"
#include "pid.h"
#include "math.h"
#include "bsp_can.h"
#include "keyboard.h"
#include "remote_ctrl.h"
#include "sys_config.h"
#include "slide_task.h"
#include "manipulator_task.h"
#include "IO.h"
#include "controller.h"
/************************************************************************************


********** @UP    ����̧�����
********** @RAISE ����̧����
********** @FALL  �����½���
********** @DOWN �����½����
*/

extern TaskHandle_t can_msg_send_Task_Handle;
UBaseType_t upraise_stack_surplus;
upraise_t upraise;
uint16_t upraise_error_angle[2] = {0};            //��¼��������ת�Ĵ���������У׼

//min[1]=5    max[1]=660  guroud[1]=170
int16_t uprise_normal_angle               			= 1000;//5000
int16_t defend_upraise_angle              			= 0;  //����̧�������
int16_t supply_upraise_angle              			= 0;
int16_t clamp_upraise_big_island_straight_angle = 0;  //����Դ��
int16_t clamp_upraise_big_island_slanted_angle  = 0;
int16_t clamp_upraise_small_island_angle  			= 0;  //255
int16_t rescue_upraise_angle              			= 0;
int16_t barrier_carry_upraise_angle       			= 0;
int16_t exchange_upraise_angle        	  			= 0;    //ԭ500����ʱ����
int16_t catch_upraise_angle   		      				= 0;
int16_t upraise_max_angle			 	 								= 0;  //650
int16_t clamp_ground_angle			      					= 0;
uint8_t direction_flag				 									= 1;    //�˵�����仯�ǶȺ��棬һ���ı䷽��
int16_t store_upraise_angle			 	 							= 0;
int16_t turn2_upraise_angle			 	 							= 550;  //������
int16_t turn2_upraise_angle_2		 	  						= 480;
int16_t pick_upraise_angle			 	  						= 0;

int32_t upraise_exchange_action_angle[2];

uint8_t upraise_minimum_state             		= 0;    //̧�����״̬��־λ Ϊ1���������λ��
uint8_t upraise_angle_ctrl_state[2]       		= {0,0};//�ֱ�̧���������˫������(��Ϊ�������Ƴ��ֿ����б���� 21.7.19)
//float upraise_pid[6];

uint8_t upraise_f;
float upraise_pid[6] = {30.0f, 0.2f, 0, 15.0f, 0.001f, 0};

void upraise_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	
  	while(1)
  	{
    	STAUS = xTaskNotifyWait((uint32_t) NULL, 
								(uint32_t) INFO_GET_UPRAISE_SIGNAL, 
								(uint32_t *)&Signal, 
								(TickType_t) portMAX_DELAY );
		if(STAUS == pdTRUE)
		{
			if((Signal & INFO_GET_UPRAISE_SIGNAL) && chassis_mode != CHASSIS_RELEASE)//���̲��Ƕϵ�ģʽ
			{
				if( chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE || chassis_mode == CHASSIS_RESCUE_MODE 
				 || chassis_mode == CHASSIS_SUPPLY_MODE|| chassis_mode == CHASSIS_EXCHANGE_MODE 
				 || chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE || chassis_mode == CHASSIS_BARRIER_CARRY_MODE
				 || chassis_mode == CHASSIS_DEFEND_MODE || chassis_mode == CHASSIS_GROUND_MODE 
				 || chassis_mode == CHASSIS_CLAMP_CATCH_MODE || chassis_mode == CHASSIS_CHECK_MODE)//�����ǹ���ģʽ
				{
					upraise_minimum_state = 0;//ֻҪ���빤��ģʽ����˵��̧��״̬�������״̬
					for(int i = 0; i < 2; i++)//PID��ʼ��
					{
						PID_Struct_Init(&pid_upraise[i],upraise_pid[0],upraise_pid[1],upraise_pid[2],7000, 500, DONE);//max.out 5000
						PID_Struct_Init(&pid_upraise_spd[i],upraise_pid[3],upraise_pid[4],upraise_pid[5],10000, 500, DONE);//max.out 10000
			   		}
					if(upraise.state == INIT_DONE)
					{
						if(1)//����̧��
						{
							for(uint8_t i = 0; i < 2; i++)
							{
								if(fabs(fabs(upraise.angle_ref[i]) - fabs(moto_upraise[i].total_angle)) < 30)//10 fabs(upraise.angle_ref[0] - moto_upraise[0].total_angle) < 5 &&
								{									
									upraise_angle_ctrl_state[i] = 1;         //�����ٶȻ���һ����־λ
								}
								if(upraise_angle_ctrl_state[0] == 1 && upraise_angle_ctrl_state[1] == 1)
								{
									upraise.updown_flag = UP;                //̧�����
								}
							}
						}						
						if( chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE 		 ||
							chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE   || 
							chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE    || 
							chassis_mode == CHASSIS_EXCHANGE_MODE 				  	 || 
							chassis_mode == CHASSIS_CLAMP_CATCH_MODE				 ||
							chassis_mode == CHASSIS_GROUND_MODE)
						{
							if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE)
							{
								upraise_change_angle_ref(clamp_upraise_big_island_straight_angle,-2000,1000,4000);//655
							}
							
							if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE)
							{
								upraise_change_angle_ref(clamp_upraise_big_island_slanted_angle,-600,-100,0);//655
							}
							else if(chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE)
							{
								upraise_change_angle_ref(clamp_upraise_small_island_angle,0,0,5000);//395
							}
							
							else if(chassis_mode == CHASSIS_CLAMP_CATCH_MODE)
							{
								upraise_change_angle_ref(catch_upraise_angle,-650,-485,0);
							}							
							
							else if(chassis_mode == CHASSIS_EXCHANGE_MODE)
							{
								upraise_change_angle_ref(exchange_upraise_angle+small_model_data[1]*SMALL_MODEL_Y_RATIO,-500,-250,0);								
							}
							
							else if(chassis_mode == CHASSIS_GROUND_MODE)
							{
								upraise_change_angle_ref(clamp_ground_angle,-4000,0,1000);
							}
						}
					}
					else//��һ�ν�������У׼���ò��о��Ƕ���ģ�����Ϊ�˱������ȼ��ϣ���ͨ�������ж������費��Ҫ��
					{
						upraise_init_handler();
					}
					upraise_motor_angle();
       	}
				else//��������ͨģʽ
				{
					if(upraise.state != INIT_DONE)//δ��ʼ����
					{
						upraise_angle_ctrl_state[0] = 0;//Ŀ����Ϊ��ÿ��̧�����̶��ǽ��ٶȻ�����
						upraise_angle_ctrl_state[1] = 0;
						upraise_init_handler();
					}
					else//��ʼ����
					{
						upraise_angle_ctrl_state[0] = 0;//Ŀ����Ϊ��ÿ��̧�����̶��ǽ��ٶȻ�����
						upraise_angle_ctrl_state[1] = 0;
						upraise.angle_ref[0] = upraise.init_angle[0];
						upraise.angle_ref[1] = upraise.init_angle[1];
						upraise_motor_angle();
					}
				}
        		xTaskGenericNotify((TaskHandle_t) can_msg_send_Task_Handle, 
        		                   (uint32_t) UPRAISE_MOTOR_MSG_SIGNAL, 
        		                   (eNotifyAction) eSetBits, 
        		                   (uint32_t *)NULL );
        
        		upraise_stack_surplus = uxTaskGetStackHighWaterMark(NULL);       
      		}
	  		/*�ر�ң��ʱ������*/
//			if(chassis_mode == CHASSIS_RELEASE)
//			{
//				memset(glb_cur.upraise_cur, 0, sizeof(glb_cur.upraise_cur));//memset,�������е��������ݳ�ʼ��Ϊ�м��Ǹ���				
//			}
//			else 
//			{
//				memcpy(glb_cur.upraise_cur, upraise.current, sizeof(upraise.current));
//			}
   		}
  }
}
void upraise_motor_angle()
{
	
	for(int i = 0; i < 2; i++)
	{
		upraise_angle_ctrl_state[i]=1;
//		if(fabs(fabs(upraise.angle_ref[i] + kb_upraise_angle[i]) - fabs(upraise.angle_fdb[i])) < 20)//10 fabs(upraise.angle_ref[0] - moto_upraise[0].total_angle) < 5 &&
//		{									
//			upraise_angle_ctrl_state[i] = 1;                   //�����ٶȻ���һ����־λ
//		}
//		else
//		{
//			upraise_angle_ctrl_state[i] = 0; 
//		}
	}	
////	
////	if(!suction_advancing_flag)
////	{
//		if(!upraise_angle_ctrl_state[0])
//		{
//			if(((upraise.angle_ref[0]) ) > moto_upraise[0].total_angle)
//			{
//				upraise.spd_ref[0] =  2000;							
//			}
//			else if(((upraise.angle_ref[0])) < moto_upraise[0].total_angle)
//			{
//				upraise.spd_ref[0] =  -2000;								
//			}
//			upraise.current[0] = pid_calc(&pid_upraise_spd[0], upraise.spd_fdb[0], upraise.spd_ref[0]);     //���ٶȻ�
//		}
//		if(!upraise_angle_ctrl_state[1])
//		{
//			if(((upraise.angle_ref[1])) > moto_upraise[1].total_angle)
//			{
//				upraise.spd_ref[1] =  2000;							
//			}
//			else if(((upraise.angle_ref[1])) < moto_upraise[1].total_angle)
//			{
//				upraise.spd_ref[1] = -2000;								
//			}
//			upraise.current[1] = pid_calc(&pid_upraise_spd[1], upraise.spd_fdb[1], upraise.spd_ref[1]);     //���ٶȻ�
//		}
//	}
	for(int i = 0;i < 2;i++)
	{	
		/***************�ǶȻ���**********************************/
//		upraise.angle_ref[i]=(upraise.angle_ref[i]-upraise.init_angle[i])*ANGLE_HIGHT;
//		
		upraise.angle_ref[i]+=uprise_normal_angle;
		
		/********************pid����*************************************/
		if(upraise_angle_ctrl_state[i])
		{						
			upraise.spd_ref[i] = pid_calc(&pid_upraise[i], moto_upraise[i].total_angle, upraise.angle_ref[i]);//�ǶȻ�
			upraise.current[i] = pid_calc(&pid_upraise_spd[i], upraise.spd_fdb[i],pid_upraise[i].out);        //�ٶȻ�
		}
		else	
		upraise.current[i] = pid_calc(&pid_upraise_spd[i], upraise.spd_fdb[i], upraise.spd_ref[i]);     //���ٶȻ�
	}
 
}
void upraise_change_angle_ref(int16_t upraise_mode_angle,int16_t small,int16_t medium,int16_t large)
{
	if(clamp_upraise_flag == INITIAL)
	{							
		upraise.angle_ref[0] = upraise.init_angle[0] + (upraise_mode_angle);
		upraise.angle_ref[1] = upraise.init_angle[1] + (upraise_mode_angle);									
	}
	else if(clamp_upraise_flag == SMALL)
	{							
		upraise.angle_ref[0] = upraise.init_angle[0] + (upraise_mode_angle+small);
		upraise.angle_ref[1] = upraise.init_angle[1] + (upraise_mode_angle+small);									
	}
	else if(clamp_upraise_flag == MEDIUM)
	{							
		upraise.angle_ref[0] = upraise.init_angle[0] + (upraise_mode_angle+medium);
		upraise.angle_ref[1] = upraise.init_angle[1] + (upraise_mode_angle+medium);							
	}
	else if(clamp_upraise_flag == LARGE)
	{							
		upraise.angle_ref[0] = upraise.init_angle[0] + (upraise_mode_angle+large);
		upraise.angle_ref[1] = upraise.init_angle[1] + (upraise_mode_angle+large);
	}
	
	if(clamp_upraise_flag == 4)
	{
		if(chassis_mode == CHASSIS_EXCHANGE_MODE)
		{
			upraise.angle_ref[0] = upraise.init_angle[0] + pick_upraise_angle;
			upraise.angle_ref[1] = upraise.init_angle[1] + pick_upraise_angle;
		}
		else
		{
			upraise.angle_ref[0] = upraise.init_angle[0] - store_upraise_angle;
			upraise.angle_ref[1] = upraise.init_angle[1] + store_upraise_angle;
		}
	}
	else if(clamp_upraise_flag == 5)
	{							
		upraise.angle_ref[0] = upraise.init_angle[0] - turn2_upraise_angle;
		upraise.angle_ref[1] = upraise.init_angle[1] + turn2_upraise_angle;
	}
	else if(clamp_upraise_flag == 6)
	{							
		upraise.angle_ref[0] = upraise.init_angle[0] - turn2_upraise_angle_2;
		upraise.angle_ref[1] = upraise.init_angle[1] + turn2_upraise_angle_2;
	}
}

void upraise_param_init(void)
{
  	memset(&upraise, 0, sizeof(upraise_t));
	
  	upraise.state = INIT_NEVER;
  	upraise.updown_flag = DOWN;//DOWN

	for(int i = 0; i < 2; i++)
  	{
  	  	PID_Struct_Init(&pid_upraise[i],upraise_pid[0],upraise_pid[1],upraise_pid[2],4000, 500, INIT);
  	  	PID_Struct_Init(&pid_upraise_spd[i],upraise_pid[3],upraise_pid[4],upraise_pid[5],8000, 500, INIT);
  	}
    
}

/*��ǰģʽ����Ҫ̧������Ҫ���������״̬�������λ�ü����У׼��ͬʱ���ϵ�ʱ��У׼��� ���ڲ�ȡ����У׼���о��е��˷�ʱ�䣬Ч��һ����*/
void upraise_init_handler(void)  //21����У׼����
{
	uint8_t init_flag[2]={0,0};
	if(clamp.clamp_flag == CLAMPED && (upraise.updown_flag != DOWN || upraise_minimum_state != 1) && upraise.state == INIT_NEVER)//���������״̬ͬʱ����Ҫ̧���������ջ��������½����������˶������λ�� && upraise.state == 1
	{ 					
//	 	if(upraise_error_angle[0] < 5)
//	 	{
				upraise.spd_ref[0] = -700;//�����½�
	   		upraise.spd_ref[1] = -700;
//	 	}
//		else if(upraise_error_angle[0] > 5)
//		{
//				upraise.spd_ref[0] = -1000;//�����½�
//	   		upraise.spd_ref[1] = -1000;
//		}
//		else if(upraise_error_angle[0] > 50)
//		{
//			upraise.spd_ref[0] = 0;//�����½�
//		}
											
		for(int i = 0;i < 2;i++)//У׼���̲��õ�������
		{
			upraise.current[i] = pid_calc(&pid_upraise_spd[i], upraise.spd_fdb[i], upraise.spd_ref[i]);//�ٶȻ�
		}
		/*����û����ֻ�ж���һ�������ΪУ׼*/
		for(int i = 0;i < 2;i++)
		{
//			#if UPRISE_KET_MODE
//			if( GPIO_ReadInputDataBit(UPRISE_KEY1_PORT,UPRISE_KEY1_PIN) && GPIO_ReadInputDataBit(UPRISE_KEY1_PORT,UPRISE_KEY1_PIN) )
//			{
//				upraise_minimum_state = 1;
//				upraise.updown_flag   = DOWN; //�½����
//			}
//			
//			#else
			if((fabs(upraise.spd_ref[i]) - fabs(upraise.spd_fdb[i])) >= 0.9*fabs(upraise.spd_ref[i]))//�ж��Ƿ��ת��ռ��Խ������Խ���̣�
			{
				upraise_error_angle[i]++;
				
				if(upraise_error_angle[i] > 200)//200
				{
					upraise.spd_ref[i] = 0;
					
					
					init_flag[i] = 1;
					upraise.updown_flag   = DOWN; //�½����
					
				}
			}
//			#endif
			if(init_flag[0] && init_flag[1])
				upraise_minimum_state = 1;
		} 																				
	}							
	if( upraise_minimum_state == 1)//�Ѿ������״̬ʱ����Ҫ����      upraise.updown_flag == DOWN ||
	{
//		upraise.spd_ref[0] = upraise.spd_fdb[0];
//		upraise.spd_ref[1] = upraise.spd_fdb[0];	

		upraise_error_angle[0] = 0;
		upraise_error_angle[1] = 0;		           								
		
		for(int i = 0;i < 2;i++)
		{
			upraise.current[i] = pid_calc(&pid_upraise_spd[i], upraise.spd_fdb[i], upraise.spd_ref[i]);//�ٶȻ�
		
			upraise.init_angle[i] = moto_upraise[i].total_angle;                  //ȷ��У׼�Ƕȣ�֮��ͨ���ǶȻ���ȷ����̧��λ�ã�ÿ�δӲ������λ�ý������Ƕ���У׼һ�飩						
		}	
		
    	upraise.state = INIT_DONE;		
	}
}



