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


********** @UP    ——抬升完成
********** @RAISE ——抬升中
********** @FALL  ——下降中
********** @DOWN ——下降完成
*/

extern TaskHandle_t can_msg_send_Task_Handle;
UBaseType_t upraise_stack_surplus;
upraise_t upraise;
uint16_t upraise_error_angle[2] = {0};            //记录电机满足堵转的次数，用以校准

//min[1]=5    max[1]=660  guroud[1]=170
int16_t uprise_normal_angle               			= 1000;//5000
int16_t defend_upraise_angle              			= 0;  //防守抬升至最高
int16_t supply_upraise_angle              			= 0;
int16_t clamp_upraise_big_island_straight_angle = 0;  //大资源岛
int16_t clamp_upraise_big_island_slanted_angle  = 0;
int16_t clamp_upraise_small_island_angle  			= 0;  //255
int16_t rescue_upraise_angle              			= 0;
int16_t barrier_carry_upraise_angle       			= 0;
int16_t exchange_upraise_angle        	  			= 0;    //原500，临时置零
int16_t catch_upraise_angle   		      				= 0;
int16_t upraise_max_angle			 	 								= 0;  //650
int16_t clamp_ground_angle			      					= 0;
uint8_t direction_flag				 									= 1;    //乘到电机变化角度后面，一键改变方向
int16_t store_upraise_angle			 	 							= 0;
int16_t turn2_upraise_angle			 	 							= 550;  //测试中
int16_t turn2_upraise_angle_2		 	  						= 480;
int16_t pick_upraise_angle			 	  						= 0;

int32_t upraise_exchange_action_angle[2];

uint8_t upraise_minimum_state             		= 0;    //抬升最低状态标志位 为1代表处于最低位置
uint8_t upraise_angle_ctrl_state[2]       		= {0,0};//分别抬升电机进入双环控制(因为单独控制出现框架倾斜现象 21.7.19)
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
			if((Signal & INFO_GET_UPRAISE_SIGNAL) && chassis_mode != CHASSIS_RELEASE)//底盘不是断电模式
			{
				if( chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE || chassis_mode == CHASSIS_RESCUE_MODE 
				 || chassis_mode == CHASSIS_SUPPLY_MODE|| chassis_mode == CHASSIS_EXCHANGE_MODE 
				 || chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE || chassis_mode == CHASSIS_BARRIER_CARRY_MODE
				 || chassis_mode == CHASSIS_DEFEND_MODE || chassis_mode == CHASSIS_GROUND_MODE 
				 || chassis_mode == CHASSIS_CLAMP_CATCH_MODE || chassis_mode == CHASSIS_CHECK_MODE)//底盘是工程模式
				{
					upraise_minimum_state = 0;//只要进入工程模式，则说明抬升状态不是最低状态
					for(int i = 0; i < 2; i++)//PID初始化
					{
						PID_Struct_Init(&pid_upraise[i],upraise_pid[0],upraise_pid[1],upraise_pid[2],7000, 500, DONE);//max.out 5000
						PID_Struct_Init(&pid_upraise_spd[i],upraise_pid[3],upraise_pid[4],upraise_pid[5],10000, 500, DONE);//max.out 10000
			   		}
					if(upraise.state == INIT_DONE)
					{
						if(1)//正在抬升
						{
							for(uint8_t i = 0; i < 2; i++)
							{
								if(fabs(fabs(upraise.angle_ref[i]) - fabs(moto_upraise[i].total_angle)) < 30)//10 fabs(upraise.angle_ref[0] - moto_upraise[0].total_angle) < 5 &&
								{									
									upraise_angle_ctrl_state[i] = 1;         //控制速度环的一个标志位
								}
								if(upraise_angle_ctrl_state[0] == 1 && upraise_angle_ctrl_state[1] == 1)
								{
									upraise.updown_flag = UP;                //抬升完成
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
					else//第一次进入则需校准（该步感觉是多余的，但是为了保险则先加上，可通过调试判断最终需不需要）
					{
						upraise_init_handler();
					}
					upraise_motor_angle();
       	}
				else//底盘是普通模式
				{
					if(upraise.state != INIT_DONE)//未初始化完
					{
						upraise_angle_ctrl_state[0] = 0;//目的是为了每次抬升过程都是仅速度环控制
						upraise_angle_ctrl_state[1] = 0;
						upraise_init_handler();
					}
					else//初始化完
					{
						upraise_angle_ctrl_state[0] = 0;//目的是为了每次抬升过程都是仅速度环控制
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
	  		/*关闭遥控时发送零*/
//			if(chassis_mode == CHASSIS_RELEASE)
//			{
//				memset(glb_cur.upraise_cur, 0, sizeof(glb_cur.upraise_cur));//memset,将数组中的所有数据初始化为中间那个数				
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
//			upraise_angle_ctrl_state[i] = 1;                   //控制速度环的一个标志位
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
//			upraise.current[0] = pid_calc(&pid_upraise_spd[0], upraise.spd_fdb[0], upraise.spd_ref[0]);     //仅速度环
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
//			upraise.current[1] = pid_calc(&pid_upraise_spd[1], upraise.spd_fdb[1], upraise.spd_ref[1]);     //仅速度环
//		}
//	}
	for(int i = 0;i < 2;i++)
	{	
		/***************角度换算**********************************/
//		upraise.angle_ref[i]=(upraise.angle_ref[i]-upraise.init_angle[i])*ANGLE_HIGHT;
//		
		upraise.angle_ref[i]+=uprise_normal_angle;
		
		/********************pid计算*************************************/
		if(upraise_angle_ctrl_state[i])
		{						
			upraise.spd_ref[i] = pid_calc(&pid_upraise[i], moto_upraise[i].total_angle, upraise.angle_ref[i]);//角度环
			upraise.current[i] = pid_calc(&pid_upraise_spd[i], upraise.spd_fdb[i],pid_upraise[i].out);        //速度环
		}
		else	
		upraise.current[i] = pid_calc(&pid_upraise_spd[i], upraise.spd_fdb[i], upraise.spd_ref[i]);     //仅速度环
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

/*当前模式不需要抬升，则要保持在最低状态，在最低位置即完成校准（同时在上电时将校准完毕 不在采取上升校准，感觉有点浪费时间，效果一样）*/
void upraise_init_handler(void)  //21赛季校准函数
{
	uint8_t init_flag[2]={0,0};
	if(clamp.clamp_flag == CLAMPED && (upraise.updown_flag != DOWN || upraise_minimum_state != 1) && upraise.state == INIT_NEVER)//若不是最低状态同时不需要抬升（夹子收回来才能下降）将向下运动至最低位置 && upraise.state == 1
	{ 					
//	 	if(upraise_error_angle[0] < 5)
//	 	{
				upraise.spd_ref[0] = -700;//负是下降
	   		upraise.spd_ref[1] = -700;
//	 	}
//		else if(upraise_error_angle[0] > 5)
//		{
//				upraise.spd_ref[0] = -1000;//负是下降
//	   		upraise.spd_ref[1] = -1000;
//		}
//		else if(upraise_error_angle[0] > 50)
//		{
//			upraise.spd_ref[0] = 0;//负是下降
//		}
											
		for(int i = 0;i < 2;i++)//校准过程采用单环控制
		{
			upraise.current[i] = pid_calc(&pid_upraise_spd[i], upraise.spd_fdb[i], upraise.spd_ref[i]);//速度环
		}
		/*左电机没抱紧只判断另一个电机作为校准*/
		for(int i = 0;i < 2;i++)
		{
//			#if UPRISE_KET_MODE
//			if( GPIO_ReadInputDataBit(UPRISE_KEY1_PORT,UPRISE_KEY1_PIN) && GPIO_ReadInputDataBit(UPRISE_KEY1_PORT,UPRISE_KEY1_PIN) )
//			{
//				upraise_minimum_state = 1;
//				upraise.updown_flag   = DOWN; //下降完成
//			}
//			
//			#else
			if((fabs(upraise.spd_ref[i]) - fabs(upraise.spd_fdb[i])) >= 0.9*fabs(upraise.spd_ref[i]))//判断是否堵转（占比越大条件越苛刻）
			{
				upraise_error_angle[i]++;
				
				if(upraise_error_angle[i] > 200)//200
				{
					upraise.spd_ref[i] = 0;
					
					
					init_flag[i] = 1;
					upraise.updown_flag   = DOWN; //下降完成
					
				}
			}
//			#endif
			if(init_flag[0] && init_flag[1])
				upraise_minimum_state = 1;
		} 																				
	}							
	if( upraise_minimum_state == 1)//已经是最低状态时不需要给定      upraise.updown_flag == DOWN ||
	{
//		upraise.spd_ref[0] = upraise.spd_fdb[0];
//		upraise.spd_ref[1] = upraise.spd_fdb[0];	

		upraise_error_angle[0] = 0;
		upraise_error_angle[1] = 0;		           								
		
		for(int i = 0;i < 2;i++)
		{
			upraise.current[i] = pid_calc(&pid_upraise_spd[i], upraise.spd_fdb[i], upraise.spd_ref[i]);//速度环
		
			upraise.init_angle[i] = moto_upraise[i].total_angle;                  //确定校准角度，之后通过角度环精确控制抬升位置（每次从不是最低位置降下来是都会校准一遍）						
		}	
		
    	upraise.state = INIT_DONE;		
	}
}



