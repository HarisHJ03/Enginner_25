#include "slide_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "comm_task.h"
#include "detect_task.h"
#include "slide_task.h"
#include "chassis_task.h"
#include "remote_ctrl.h"
#include "pc_rx_data.h"
#include "pid.h"
#include "bsp_can.h"
#include "sys_config.h"
#include "math.h"
#include "ramp.h"
#include "keyboard.h"
#include "clamp_task.h"
#include "manipulator_task.h"
#include "controller.h"

UBaseType_t slide_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

slide_t				slide;

float slide_pid[6]					={15,0,0,10,0,0};


//slide[0] -    slide[1] +
//0-370
//min[0]=205  max[0]=-261     ground[0]=28
int16_t front_slide_normal_angle 				= 100;//normal模式就是最短模式
int16_t slide_normal_angle 				= 0;
int16_t slide_small_island_angle				= 0;//177-177小资源岛不用伸出？
int16_t slide_big_island_straight_angle = 150;//440-177大资源岛伸出最长
int16_t slide_ground_angle 							= 0;//177-177
int16_t slide_bigisland_slanted_angle 	= 0;//466-177空接模式伸出最长
int16_t slide_exchange_angle						= 0;//327-177伸出一段距离等待兑换
int16_t slide_max_angle 								= 0;//466-177
int16_t slide_second_box_angle 					= 0;//177-177
int16_t slide_store_angle 							= 0;
int16_t slide_pick_angle 								= 0;

int16_t slide_turn2_angle 							= 470;//测试中
int16_t slide_turn2_angle_2 						= 250;//测试中

uint8_t slide_finish_flag;
uint8_t slide_angle_ctrl_state[SLIDE_NUMBER];

uint8_t slide_minimum_state;
uint8_t slide_error_angle[SLIDE_NUMBER];


pid_parameter_t pid_slide_angle_parameter[SLIDE_NUMBER];
pid_parameter_t pid_slide_spd_parameter[SLIDE_NUMBER];

manipulator_t slide_24[SLIDE_NUMBER];



void slide_PID_INIT(INIT_STATUS init_status)
{//先暂时只用后两个
	/*********************角度设置********************************/
	slide_24[0].speed=200;
	slide_24[1].speed=-200;
	slide_24[2].speed=0;
	slide_24[3].speed=0;
	
	slide_24[0].spd_ref=0;
	slide_24[1].spd_ref=0;
	slide_24[2].spd_ref=0;
	slide_24[3].spd_ref=0;

	slide_24[0].normal_angle=0;
	slide_24[1].normal_angle=0;//-16->-346
	slide_24[2].normal_angle=0;//179->54
	slide_24[3].normal_angle=0;

	slide_24[0].exchange_angle=0;
	slide_24[1].exchange_angle=0;
	slide_24[2].exchange_angle=20;
	slide_24[3].exchange_angle=0;

	slide_24[0].store_angle=-196;
	slide_24[1].store_angle=40;
	slide_24[2].store_angle=170;
	slide_24[3].store_angle=0;

	slide_24[0].exchange_pick_angle=-196;
	slide_24[1].exchange_pick_angle=40;
	slide_24[2].exchange_pick_angle=170;
	slide_24[3].exchange_pick_angle=1;

	slide_24[1].exchange_pick_angle_2=-10;
	slide_24[1].exchange_pick_angle_2=-10;
	slide_24[2].exchange_pick_angle_2=207;
	slide_24[3].exchange_pick_angle_2=0;

	//     straight/slanted
	slide_24[0].bigisland_straight_angle=0;
	slide_24[1].bigisland_straight_angle=0;//
	slide_24[2].bigisland_straight_angle=200;//增加上抬30
	slide_24[3].bigisland_straight_angle=200;

	slide_24[0].smallisland_angle=0;
	slide_24[1].smallisland_angle=0;//加大是放低
	slide_24[2].smallisland_angle=0;//绝对值减小是上抬
	slide_24[3].smallisland_angle=0;

	slide_24[0].ground_angle=0;
	slide_24[1].ground_angle=90;//
	slide_24[2].ground_angle=-60;//
	slide_24[3].ground_angle=0;
	
	slide_24[1].bigisland_slanted_angle=90;//加大是放低
	slide_24[2].bigisland_slanted_angle=0;//加大是上抬
	slide_24[3].bigisland_slanted_angle=0;

	slide_24[0].mode_angle=0;
	slide_24[1].mode_angle=0;
	slide_24[2].mode_angle=0;
	slide_24[3].mode_angle=0;
	
	/*********************PID参数定义********************************/
	uint8_t ID;
	
	pid_slide_angle_parameter[0].p=50.0;//50
	pid_slide_angle_parameter[0].i=0.1;
	pid_slide_angle_parameter[0].d=0.0;
	pid_slide_angle_parameter[0].max_out=6000;
	pid_slide_angle_parameter[0].integral_limit=500;
	
	pid_slide_spd_parameter[0].p=30.0;//10
	pid_slide_spd_parameter[0].i=0.1;
	pid_slide_spd_parameter[0].d=0.0;
	pid_slide_spd_parameter[0].max_out=3500;
	pid_slide_spd_parameter[0].integral_limit=500;
	
	pid_slide_angle_parameter[1].p=50.0;//50
	pid_slide_angle_parameter[1].i=0.1;
	pid_slide_angle_parameter[1].d=0.0;
	pid_slide_angle_parameter[1].max_out=6000;
	pid_slide_angle_parameter[1].integral_limit=500;
	
	pid_slide_spd_parameter[1].p=30.0;//10
	pid_slide_spd_parameter[1].i=0.1;
	pid_slide_spd_parameter[1].d=0.0;
	pid_slide_spd_parameter[1].max_out=3500;
	pid_slide_spd_parameter[1].integral_limit=500;
	
	pid_slide_angle_parameter[2].p=50.0;
	pid_slide_angle_parameter[2].i=0.01;
	pid_slide_angle_parameter[2].d=0.0;
	pid_slide_angle_parameter[2].max_out=7000;
	pid_slide_angle_parameter[2].integral_limit=500;
	
	pid_slide_spd_parameter[2].p=10.0;
	pid_slide_spd_parameter[2].i=0.0;
	pid_slide_spd_parameter[2].d=0.0;
	pid_slide_spd_parameter[2].max_out=14000;
	pid_slide_spd_parameter[2].integral_limit=500;
	
	pid_slide_angle_parameter[3].p=50.0;
	pid_slide_angle_parameter[3].i=0.01;
	pid_slide_angle_parameter[3].d=0.0;
	pid_slide_angle_parameter[3].max_out=7000;
	pid_slide_angle_parameter[3].integral_limit=500;
	
	pid_slide_spd_parameter[3].p=10.0;
	pid_slide_spd_parameter[3].i=0.0;
	pid_slide_spd_parameter[3].d=0.0;
	pid_slide_spd_parameter[3].max_out=14000;
	pid_slide_spd_parameter[3].integral_limit=500;
	
	
	for(ID=0;ID<SLIDE_NUMBER;ID++)
	{
		PID_Struct_Init(&pid_slide[ID],pid_slide_angle_parameter[ID].p,pid_slide_angle_parameter[ID].i,pid_slide_angle_parameter[ID].d,
		pid_slide_angle_parameter[ID].max_out,pid_slide_angle_parameter[ID].integral_limit,init_status);
			
		PID_Struct_Init(&pid_slide_spd[ID],pid_slide_spd_parameter[ID].p,pid_slide_spd_parameter[ID].i,pid_slide_spd_parameter[ID].d,
		pid_slide_spd_parameter[ID].max_out,pid_slide_spd_parameter[ID].integral_limit,init_status);
	}
}

void slide_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	
	while(1)
	{
		STAUS = xTaskNotifyWait((uint32_t) NULL, 
								(uint32_t) INFO_GET_SLIDE_SIGNAL, 
								(uint32_t *)&Signal, 
								(TickType_t) portMAX_DELAY );
		if(STAUS ==pdTRUE)
		{
			if((Signal & INFO_GET_SLIDE_SIGNAL) && chassis_mode != CHASSIS_RELEASE)
			{
				
		    if(slide.state == INIT_DONE)
				{	
					
				if(chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE  				||
						chassis_mode == CHASSIS_EXCHANGE_MODE					   				||
						chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE 	||
						chassis_mode == CHASSIS_DEFEND_MODE						   				||
						chassis_mode == CHASSIS_GROUND_MODE 						 				||
						chassis_mode == CHASSIS_CLAMP_CATCH_MODE 			   				||
						chassis_mode == CHASSIS_CHECK_MODE											||
						chassis_mode == CHASSIS_NORMAL_MODE
						)
				{
					
				
					slide_minimum_state = 0;
				/************按需切换模式，FRONT_MODE代表兑换的前臂，SIDE_MODE代表夹取的侧臂*******************/
						
						if(chassis_mode == CHASSIS_NORMAL_MODE)
						{
							slide_change_angle_ref(slide_normal_angle,-420,-210,70,FRONT_MODE);//420
						}
						if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE)
						{
							slide_change_angle_ref(slide_big_island_straight_angle,-420,-210,70,SIDE_MODE);//420
						}
						else if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE)//466-177
						{
							slide_change_angle_ref(slide_bigisland_slanted_angle,-289,-145,0,SIDE_MODE);//289
						}		
						else if(chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE)
						{
							slide_change_angle_ref(slide_small_island_angle,0,130,263,SIDE_MODE);// 0
						}
						else if(chassis_mode == CHASSIS_EXCHANGE_MODE)
						{
							slide_change_angle_ref(slide_exchange_angle+small_model_data[0]*SMALL_MODEL_X_RATIO,-150,0,139,FRONT_MODE);//150															
						}
						
						else if(chassis_mode == CHASSIS_GROUND_MODE)
						{
							slide_change_angle_ref(slide_ground_angle,0,0,0,FRONT_MODE);//0
						}							
						
						
						/*********更改目标角度后进行角度环计算********************/
						slide_motor_angle();
					}
				}else//没有初始化
					{
						slide_init_handler();
					}
				
			}
//			else//除了取矿任务以外的模式（正常模式。。。）
//			{
//				if(slide.state == INIT_NEVER)
//				{
//				
//					slide_init_handler();
//				}
//				else
//				{
//						slide_motor_angle();								
//			
//				}	
//			}
				xTaskGenericNotify((TaskHandle_t) can_msg_send_Task_Handle, 
													 (uint32_t) SLIDE_MOTOR_MSG_SIGNAL, 
													 (eNotifyAction) eSetBits, 
													 (uint32_t *)NULL );
				
				slide_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    					
		}
//		if(chassis_mode == CHASSIS_RELEASE)
//		{
//			memset(glb_cur.slide_cur, 0, sizeof(glb_cur.slide_cur));
//			glb_cur.manipulator[0]=0;				
//		}
//		else 
//		{

//			memcpy(glb_cur.slide_cur, slide.current, sizeof(slide.current));
//		}
	}
}

void slide_motor_angle()//仅限兑换
{
	uint8_t i;
	for( i= 0; i<2;i++)
	{
		slide_angle_ctrl_state[i]=0;
		if(fabs(slide.angle_ref[i] - moto_slide[i].total_angle) < 20)//角度差小
		{
			slide_angle_ctrl_state[i] = 1;//角使用度控制
		}
		if(slide_angle_ctrl_state[0] == 1 && slide_angle_ctrl_state[1] == 1)
		{
			slide_finish_flag = 1;												
		}
	}	
	for(int i = 0;i < 2;i++)
	{
		if(slide_angle_ctrl_state[i])//算角度环
		{
			slide.spd_ref[i] =pid_calc(&pid_slide[i], slide.angle_fdb[i], slide.angle_ref[i]);//角度环
			slide.current[i] = pid_calc(&pid_slide_spd[i], slide.spd_fdb[i],slide.spd_ref[i]);        //速度环								
		}
		else//算速度环，速度固定
		{
			if(!slide_angle_ctrl_state[i])//如果不用角度环
			{
				if(((slide.angle_ref[i]+ kb_slide_angle[i]) ) > slide.angle_fdb[i])
				{
					slide.spd_ref[i] =  1500;							
				}
				else if(((slide.angle_ref[i]+ kb_slide_angle[i]) ) < slide.angle_fdb[i])
				{
					slide.spd_ref[i] =  -1500;								
				}
			}
		slide.current[i] = pid_calc(&pid_slide_spd[i], slide.spd_fdb[i], slide.spd_ref[i]);     //仅速度环
		}
	}
}
void slide_change_angle_ref(int16_t slide_mode_angle,int16_t small,int16_t medium,int16_t large,uint8_t mode_flag)//四档可调
{
	if(mode_flag==FRONT_MODE)
	{
		if(clamp_slide_flag == INITIAL)
		{							
			slide.angle_ref[0] = slide.init_angle[0] - (slide_mode_angle);
			slide.angle_ref[1] = slide.init_angle[1] + (slide_mode_angle);									
		}
		else if(clamp_slide_flag == SMALL)
		{							
			slide.angle_ref[0] = slide.init_angle[0] - (slide_mode_angle+small);
			slide.angle_ref[1] = slide.init_angle[1] + (slide_mode_angle+small);									
		}
		else if(clamp_slide_flag == MEDIUM)
		{							
			slide.angle_ref[0] = slide.init_angle[0] - (slide_mode_angle+medium);
			slide.angle_ref[1] = slide.init_angle[1] + (slide_mode_angle+medium);							
		}
		else if(clamp_slide_flag == LARGE)
		{							
			slide.angle_ref[0] = slide.init_angle[0] - (slide_mode_angle+large);
			slide.angle_ref[1] = slide.init_angle[1] + (slide_mode_angle+large);
		}
		else if(clamp_slide_flag == 4)
		{
			if(chassis_mode == CHASSIS_EXCHANGE_MODE)
			{
				slide.angle_ref[0] = slide.init_angle[0] - slide_pick_angle;
				slide.angle_ref[1] = slide.init_angle[1] + slide_pick_angle;
			}
			else
			{
				slide.angle_ref[0] = slide.init_angle[0] - slide_store_angle;
				slide.angle_ref[1] = slide.init_angle[1] + slide_store_angle;
			}
		}
		else if(clamp_slide_flag == 5)
		{							
			slide.angle_ref[0] = slide.init_angle[0] - slide_turn2_angle;
			slide.angle_ref[1] = slide.init_angle[1] + slide_turn2_angle;
		}
		else if(clamp_slide_flag == 6)
		{							
			slide.angle_ref[0] = slide.init_angle[0] - slide_turn2_angle_2;
			slide.angle_ref[1] = slide.init_angle[1] + slide_turn2_angle_2;
		}
	}
	
	else if(mode_flag==SIDE_MODE)
	{
		if(clamp_slide_flag == INITIAL)
		{							
			slide.angle_ref[2] = slide.init_angle[2] + (slide_mode_angle);						
		}
		else if(clamp_slide_flag == SMALL)
		{							
			slide.angle_ref[2] = slide.init_angle[2] + (slide_mode_angle+small);		
		}
		else if(clamp_slide_flag == MEDIUM)
		{							
			slide.angle_ref[2] = slide.init_angle[2] + (slide_mode_angle+medium);				
		}
		else if(clamp_slide_flag == LARGE)
		{							
			slide.angle_ref[2] = slide.init_angle[2] + (slide_mode_angle+large);
		}
	}
	uint8_t ID;
	for(ID=2;ID<SLIDE_NUMBER;ID++)
	{
		slide.angle_ref[ID]=slide_24[ID].normal_angle + slide_24[ID].mode_angle;
	}

}

void slide_param_init(void)
{
	memset(&slide,0,sizeof(slide_t));
	
	slide.state = INIT_NEVER;
	
	for(int i = 0; i<=2;i++)
	{
    PID_Struct_Init(&pid_slide[i],slide_pid[0],slide_pid[1],slide_pid[2],4000, 500, INIT);
    PID_Struct_Init(&pid_slide_spd[i],slide_pid[3],slide_pid[4],slide_pid[5],8000, 500, INIT);
	}
}

void slide_init_handler(void)
{
	slide_PID_INIT(DONE);
	uint8_t ID;
	uint8_t init_done_flag=0;
	int flag[2];
	
	for(int i = 0;i<2;i++)//暂用，覆盖了前面4slidepid设置	
	{
		PID_Struct_Init(&pid_slide[i],slide_pid[0],slide_pid[1],slide_pid[2],5000, 500, DONE);//max.out 5000
		PID_Struct_Init(&pid_slide_spd[i],slide_pid[3],slide_pid[4],slide_pid[5],8000, 500, DONE);//max.out 10000		
	}
	
	
	if(slide_minimum_state != 1 || slide.state == INIT_NEVER)
	{
		init_done_flag=0;
		for(ID=0;ID<2;ID++)
		{
			
			if(slide_error_angle[ID] <200)
			{
				slide.spd_ref[ID] = slide_24[ID].speed;
				flag[ID]=0;
			}
			else
			{
				slide.spd_ref[ID] = 0;
				flag[ID]=1;
			}
			init_done_flag+=flag[ID];
			
		}
		if(init_done_flag==2) slide_minimum_state=1;
		
		for(int i = 0;i < SLIDE_NUMBER;i++)
		{
			slide.current[i] = pid_calc(&pid_slide_spd[i],slide.spd_fdb[i],slide.spd_ref[i]);
		}
		
		for(int i = 0;i < SLIDE_NUMBER;i++)
		{
			if((fabs(slide.spd_ref[i]) - fabs(slide.spd_fdb[i])) > 0.3*fabs(slide.spd_ref[i]))//判断是否堵转（占比越大条件越苛刻）
			{
				slide_error_angle[i]++;
				slide.init_angle[i] = moto_slide[i].total_angle;                  //一直刷新						
			}
		}
	}
		else if(manipulator[0].init_flag == 1)
		{
		}
		
	if( slide_minimum_state == 1 )//&& manipulator[0].init_flag == 1 )//已经是最低状态时不需要给定
	{

		slide_error_angle[0] = 0;
		slide_error_angle[1] = 0;	
		slide_error_angle[2] = 0;
		manipulator_error[0] = 0;
		
		for(int i = 0;i < SLIDE_NUMBER;i++)
		{
			 slide.spd_ref[0] = 0;
			 slide.spd_ref[1] = 0;

			 slide.init_angle[i] = moto_slide[i].total_angle;                  //确定校准角度，之后通过角度环精确控制抬升位置（每次从不是最低位置降下来是都会校准一遍）						
			 slide.current[i] = pid_calc(&pid_slide_spd[i], slide.spd_fdb[i], slide.spd_ref[i]);//速度环

		}
    slide.state = INIT_DONE;		
	}
	
}
