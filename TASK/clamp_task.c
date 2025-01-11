#include "clamp_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "comm_task.h"
#include "detect_task.h"
#include "upraise_task.h"
#include "chassis_task.h"
#include "slide_task.h"
#include "remote_ctrl.h"
#include "remote_ctrl.h"
#include "pc_rx_data.h"
#include "pid.h"
#include "bsp_can.h"
#include "sys_config.h"
#include "math.h"
#include "ramp.h"
#include "keyboard.h"
#include "manipulator_task.h"
#include "controller.h"
#include "User.h"

UBaseType_t clamp_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

clamp_t clamp;
ramp_t  clamp_pit_ramp;
store_t store;
int16_t CeBi=0;//*()
float clamp_pid_pit[6] = {30,0,0,10,0,0};

float clamp_attitude_adjustment_pid[6] = {30.0f, 0.2f, 0, 13.5f, 0.001f, 0};
/*为了一上电夹取电机能保持在水平位置添加了下面两行 日期：20.10.1*/
uint32_t clamp_initialization_error_angle[3]        = {0};//初始化
uint32_t clamping_ore_initialization_error_angle[3] = {0};

uint32_t clamp_action_times_old;						//完成取矿用到的延时
uint32_t store_action_times;						//存矿用延时
uint32_t exchange_action_times; 					//兑换用延时
uint32_t clamp_ready_times;							//准备取矿用的延时
uint32_t clamp_init_times;						  	//初始化的延时	
uint8_t ore_adjust_state       				= 0; 	//控制使能遥控调整舵机
uint8_t ore_clamp_abjust_state 				= 0; 	//取矿石避免干涉需要将矿石转上来
int32_t posture_steering_engine_ref[2]; 			//姿态调整舵机
uint8_t exchange_cmd_finish    				= 0; 	//控制仅夹取两箱时的时间刷新
uint8_t exchange_finish 	  				= 0;	//当兑换完毕 且执行了纠错动作后 为了防止误操作 当切换了SW1后将无法执行兑换动作

clamp_adjust_t clamp_adjust = CLAMP_ADJUST_UNCMD; 	//使能夹取机构的姿态调整功能

exchange_sequence_t exchange_sequence = EXCGANHE_INIT;
exchange_sequence_t last_exchange_sequence;
uint8_t exchange_frist_box_finish = 0; 			  	//因为兑换时 考虑没能一次推落 则无法在 兑换时刷新为零箱
uint8_t exchange_ctrl_gimbal      = 0;  		  	//控制图传抬头查看调整
//uint8_t start_exexchange_cmd_state = 0;
uint32_t exchange_action_timess   = 0;

pick_box_t pick_box;

//23赛季使用
int16_t store_servo_angle[2] ={1834,1166};
int32_t clamp_exchange_action_angle;
uint8_t clamp_slide_flag;							//1在里面，2中间，3伸出
uint8_t clamp_upraise_flag;							//1低 2中 3高
uint8_t exchange_spd_ctrl_state;
uint8_t mode_prepareing_flag=1;
int16_t kb_inhale_servos;
uint8_t clamping_flag;

//24赛季添加
uint8_t have_box_number_old=0;
uint8_t storeing_flag=0;
uint8_t picking_flag=0;

void clamp_task(void *parm)
{
	int8_t i;
	uint32_t Signal;
	BaseType_t STAUS;
	while(1)
	{
    	STAUS = xTaskNotifyWait((uint32_t) NULL, 
								(uint32_t) INFO_GET_CLAMP_SIGNAL, 
								(uint32_t *)&Signal, 
								(TickType_t) portMAX_DELAY );
		if(STAUS == pdTRUE && 0)
		{
			
			if((Signal & INFO_GET_CLAMP_SIGNAL) && (chassis_mode != CHASSIS_RELEASE))
			{
			 	manipulator_pid_init(DONE);//pid参数定义//测试
				if(	clamp_mode == SMALL_ISLAND 		   	||	
				   	clamp_mode == CATCH_MODE  		   	||
					clamp_mode == BIG_ISLAND_STRAIGHT  	||	
					clamp_mode == BIG_ISLAND_SLANTED   	||
					clamp_mode == GROUND_MODE)
				{
//					store_handler_old();
				}					
				if(	clamp_mode == SMALL_ISLAND 		   	|| 
					clamp_mode == BIG_ISLAND_STRAIGHT  	||	
					clamp_mode == BIG_ISLAND_SLANTED   	||  
					clamp_mode == GROUND_MODE)
				{ 
          view_switch = CLAMP_VIEW;//显示器视野
					/*大小资源岛*/
					if(clamp_mode == SMALL_ISLAND)
					{
						switch(small_island_mode)
						{
							/*小资源岛的夹取操作其实都是一致的 只是使能方式不一样*/
							case SMALL_ISLAND_ORDINARY_MODE:
							case SMALL_ISLAND_AUTOMATIC_CLAMP_ONE_MODE://夹取一箱和连夹三箱的区别在于底盘的控制不同
							case SMALL_ISLAND_AUTOMATIC_MODE:          //连夹三箱
							{ 
								small_island_clamp_handler_MKO();
							}break;
							default:
							{
							}break;    
						}
					}
					else if( clamp_mode == BIG_ISLAND_STRAIGHT)//大资源岛夹取控制
					{
						big_island_stright_clamp_handler_MKO();				
					}
					else if( clamp_mode == BIG_ISLAND_SLANTED)//大资源岛夹取控制
					{
						big_island_stright_clamp_handler_MKO();				
					}
					else if(clamp_mode == GROUND_MODE)//夹取地上控制
					{
						clamp_ground_handler_MKO();
					}
				}
				else if(clamp_mode == CATCH_MODE)//空接模式
				{
					big_island_slanted_clamp_handler_MKO();
				}
				else if(clamp_mode == EXCHANGE_MODE)//兑换模式
				{
        			exchange_handler_MKO();		
				}
				else if((clamp_mode == DEFEND_MODE  && upraise.updown_flag == UP))
				{
					defend_handler();//保卫前哨站
				}
			
			/*算法视觉辅助没有使用该使能位。只需判断不是普通的夹取模式都将失能*/
			if(	clamp_mode != BIG_ISLAND_STRAIGHT 	&& 
				clamp_mode != BIG_ISLAND_SLANTED 	&& 
				clamp_mode != SMALL_ISLAND 			&& 
				clamp_mode != GROUND_MODE			&& 
				clamp_mode != EXCHANGE_MODE 		&&
				clamp_mode != CATCH_MODE)
			{
				clamp.clamp_cmd  = 0; //单夹失能			
				clamp.clamp_flag = CLAMPED;		
			}
			
			if(	small_island_mode != SMALL_ISLAND_AUTOMATIC_CLAMP_ONE_MODE && 
				small_island_mode != SMALL_ISLAND_AUTOMATIC_MODE &&
				big_island_mode   != BIG_ISLAND_AUTOMATIC_CLAMP_ONE_MODE) //初始化
			{
				clamp.identify_cmd = 0;	//自动识别的使能位
			}
			
			if( chassis_mode != CHASSIS_EXCHANGE_MODE && 
				chassis_mode != CHASSIS_CHECK_MODE) //若不是兑换模式，且现在夹着的是第两箱，则一直保持着夹紧的状态
			{
				/*刷新标志位*/
				exchange_cmd_finish = 0;
			}
			
			/*无异常状态且不在任务中时，保持初始状态*/	
			if((!clamp_is_controllable()) 
				&& chassis_mode != CHASSIS_CLAMP_SMALL_ISLAND_MODE 
				&& chassis_mode != CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE
				&& chassis_mode != CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE 			
				&& chassis_mode != CHASSIS_GROUND_MODE
				&& chassis_mode != CHASSIS_EXCHANGE_MODE 
				&& chassis_mode != CHASSIS_DEFEND_MODE
				&& chassis_mode != CHASSIS_GROUND_MODE
				&& chassis_mode != CHASSIS_CLAMP_CATCH_MODE
				&& chassis_mode != CHASSIS_CHECK_MODE)
			{
				clamp_slide_flag = 0;
				clamp_upraise_flag = 0;		
				/*检查电机校准*/
			 	if(clamp.state == INIT_DONE)
			 	{
					manipulator[1].mode_angle=0;
					manipulator[2].mode_angle=0;
					manipulator[3].mode_angle=0;
					manipulator[4].mode_angle=0;
					manipulator[5].mode_angle=0;
					manipulator[6].mode_angle=0;
					
					manipulator_motor_angle(0);
					manipulator_motor_angle(1);
					manipulator_motor_angle(2);
					manipulator_motor_angle(3);
					manipulator_motor_angle(4);
					manipulator_motor_angle(5);
//					TIM_SetCompare3(TIM2,manipulator[6].normal_angle+manipulator[6].mode_angle+kb_manipulator_adjust_angle[6]);
//					PUMP_OFF					 
				}
				else
			 	{
					clamp_power_on_initialization();//上电就 校准函数  										
				}
			}	
	}
	
		/*关闭遥控时发送零 所谓关闭遥控并不是直接断电 而是将SW2打到最下面*/
			if(chassis_mode == CHASSIS_RELEASE)
			{
//				for(i=0;i<=5;i++)
//				{
//					glb_cur.manipulator[i]=0;
//				}					
			}
			else //在不是关闭遥控状态时持续计算角度环
			{
//			TIM_SetCompare3(TIM2,manipulator[6].normal_angle+manipulator[6].mode_angle+kb_manipulator_adjust_angle[6]);
				for(i=0;i<=5;i++)
				{
					
					if(clamp.state == INIT_DONE)
					{
						manipulator_motor_angle(i);
					}
//					else
//					{
//						manipulator_motor_speed(i,manipulator[i].spd_ref);
//					}
					
//					glb_cur.manipulator[i]=manipulator[i].current;
				}					
			}		
//			clamp_turn_handler();
//			manipulator_servo_total=manipulator[6].normal_angle+manipulator[6].mode_angle+kb_manipulator_adjust_angle[6];
//			TIM_SetCompare3(TIM2,Servo_convert(270,manipulator[2].angle_ref) );//兑换roll
//			TIM_SetCompare2(TIM4,Servo_convert(180,manipulator[3].angle_ref) );//夹取yaw
//			TIM_SetCompare3(TIM4,Servo_convert(270,manipulator[4].angle_ref) );//夹取pitch
			
	 xTaskGenericNotify((TaskHandle_t) can_msg_send_Task_Handle, 
						(uint32_t) CLAMP_MOTOR_MSG_SIGNAL, 
						(eNotifyAction) eSetBits, 
						(uint32_t *)NULL );			
  		}
 	}
}

void clamp_param_init(void)
{ 
  memset(&clamp, 0, sizeof(clamp_t));//分配内存并初始化
	
	/*夹取翻转电机*/
	manipulator_pid_init(INIT);

  clamp.state      = INIT_NEVER ;    //夹取一开始状态为未初始化
 	clamp.last_state = INIT_DONE;
  clamp.clamp_flag = CLAMPED;

  pick_box = INIT_BOX;           //初始化时，并没有已夹取的矿石
	
	ramp_init(&clamp_pit_ramp, 100); 	//斜坡函数
	clamp_init_times = HAL_GetTick();
	/*张开*/
//	PUMP_OFF
}

void manipulator_angle_init(uint8_t ID,int16_t speed)
{
	if(manipulator[ID].init_flag == 0)
	{
		manipulator_motor_speed(ID,speed);
	}
	else if(manipulator[ID].init_flag == 1)
	{
		speed=0;
		manipulator[ID].init_angle = moto_manipulator[ID].total_angle;
	}
}

/*添加一上电便初始化好的函数如下*/
//6020 朝前5084 
void clamp_power_on_initialization(void)//校准函数
{  
		manipulator_angle_init(1,4000);
		manipulator_angle_init(5,3000);
//		manipulator_angle_init(3,700);
//		manipulator[MANIPULATOR_6020_ID].spd_ref = pid_calc(&pid_manipulator[MANIPULATOR_6020_ID],moto_manipulator[MANIPULATOR_6020_ID].ecd,938)*50*(1-ramp_calc(&clamp_pit_ramp));
		if((moto_manipulator[MANIPULATOR_6020_ID].ecd - 938) > 0)
			manipulator_motor_speed(0,-30);
		else if((moto_manipulator[MANIPULATOR_6020_ID].ecd - 938) < 0)
			manipulator_motor_speed(0,30);
		else
		{
			manipulator[MANIPULATOR_6020_ID].current=pid_calc(&pid_manipulator_spd[MANIPULATOR_6020_ID],manipulator[MANIPULATOR_6020_ID].spd_fdb,manipulator[MANIPULATOR_6020_ID].spd_ref);
			manipulator[MANIPULATOR_6020_ID].init_angle = moto_manipulator[MANIPULATOR_6020_ID].total_angle;
		}
		if(manipulator_error[5]>=100)
		{
			manipulator_motor_speed(5,0);
		}
		else if(manipulator_error[1]>=5)
		{
			manipulator_motor_speed(1,200);
		}
		
	if(HAL_GetTick() - clamp_init_times >4000)
	{
		manipulator[0].normal_angle+=manipulator[0].init_angle;
		manipulator[1].normal_angle+=manipulator[1].init_angle;
		manipulator[2].normal_angle+=manipulator[2].init_angle;
		manipulator[3].normal_angle+=manipulator[3].init_angle;
		manipulator[4].normal_angle+=manipulator[4].init_angle;
		manipulator[5].normal_angle+=manipulator[5].init_angle;
		ramp_init(&clamp_pit_ramp, 100);
		clamp.state = INIT_DONE;
	}
}
/*
*  error_angle[2]      : 记录每个电机误差的次数，用来判断电机是否堵转
*  error_state[2]      : 记录每个电机的状态，是否已经堵转
*  clamp.init_flag     ：为了某电机出现堵转后就停下来，且不影响其他电机
*
*/

/**********************************************BIG ISLAND*********************************************************************/
void store_handler_old()
{
	uint8_t beging_store_flag=1;
	if(!beging_store_flag)
	{//这个大括号内写存矿动作
//			store_one_init_box();
		switch(have_box_number_old)//已夹取箱数
		{
			case 0:
			{//模板
				if(store_action_times-HAL_GetTick()<1000)	
				{
					
					
				}
			
				storeing_flag=0;
				have_box_number_old=1;
			}break;
			
			case 1:
			{//模板
				if(store_action_times-HAL_GetTick()<1000)	
				{
					
					
				}
			
				storeing_flag=0;
				have_box_number_old=2;
			}break;
			
			case 2:
			{//模板
				if(store_action_times-HAL_GetTick()<1000)	
				{
					
					
				}
			
				storeing_flag=0;
				have_box_number_old=3;
			}break;
			
		}
	}
	else
	{
		manipulator[1].mode_angle=manipulator[1].store_angle;
		manipulator[2].mode_angle=manipulator[2].store_angle;
		manipulator[3].mode_angle=manipulator[3].store_angle;
		manipulator[4].mode_angle=manipulator[4].store_angle;
		manipulator[5].mode_angle=manipulator[5].store_angle;
		manipulator[6].mode_angle=manipulator[6].store_angle;
	  store_action_times = HAL_GetTick();
		beging_store_flag=0;
		
		storeing_flag=1;
	}
}

void big_island_straight_clamp()//动作过程 store_handle也放这里
{
	clamp.clamp_flag = CLAMPING;
	
	if(HAL_GetTick() - clamp_action_times_old ==10)
	{
		clamp_upraise_flag = 3;
		manipulator[1].mode_angle-=2200;
	}
	if(HAL_GetTick() - clamp_action_times_old ==2000)
	{
		manipulator[5].mode_angle+=1200;
	}
	else if(HAL_GetTick() - clamp_action_times_old ==4000)
	{
		clamp_upraise_flag = 2;
	}
	else if(HAL_GetTick() - clamp_action_times_old ==5000)
	{
		clamp_upraise_flag = 3;
		manipulator[1].mode_angle=0;
	}

		else if(HAL_GetTick() - clamp_action_times_old >7500)//必须用大于
	{
		//store_handler_old();
		if(!storeing_flag)
		{//这下面写原本有的标志位控制
			clamp.clamp_cmd    = 0;
			clamp.identify_cmd = 0;
			big_island_mode = BIG_ISLAND_ORDINARY_MODE;
		}			
	}
}



void big_island_stright_clamp_handler_MKO()//
{
	if(clamp.clamp_cmd)//大概取矿的动作是一样的，只不过放矿动作不同
	{
			big_island_straight_clamp();		
	}
	else if(mode_prepareing_flag)
	{
		clamp.clamp_cmd 				= 0;
		clamp.clamp_flag				= CLAMPED;
		
		clamp_slide_flag=0;
		clamp_upraise_flag=0;
		manipulator[1].mode_angle=manipulator[1].bigisland_straight_angle;
		manipulator[2].mode_angle=manipulator[2].bigisland_straight_angle;
		manipulator[3].mode_angle=manipulator[3].bigisland_straight_angle;
		manipulator[4].mode_angle=manipulator[4].bigisland_straight_angle;
		manipulator[5].mode_angle=manipulator[5].bigisland_straight_angle;
		manipulator[6].mode_angle=manipulator[6].bigisland_straight_angle;
		slide_24[3].mode_angle=slide_24[3].bigisland_straight_angle;
		clamp_action_times_old= HAL_GetTick();
		
		PUMP_ON
		EXCHANGE_ON
	}
}
void big_island_slanted_clamp()//倾斜大资源岛
{
	clamp.identify_cmd  = 0;
	clamp_upraise_flag = 2;		
	clamp_slide_flag = 2;
}
void big_island_slanted_clamp_handler_MKO()
{
//  clamp.clamp_flag = CLAMPING;
	if((clamp.clamp_cmd && pick_box !=SECOND_BOX))
	{
		if(pick_box == INIT_BOX)
		{
			big_island_slanted_clamp();		
		}
	}
	else if(mode_prepareing_flag)
	{
		clamp_action_times_old = HAL_GetTick();
		clamp.clamp_flag = CLAMPED;
		PUMP_ON
		clamp_upraise_flag = 0;
		clamp_slide_flag = 0;
		manipulator[1].mode_angle=manipulator[1].bigisland_slanted_angle;
		manipulator[2].mode_angle=manipulator[2].bigisland_slanted_angle;
		manipulator[3].mode_angle=manipulator[3].bigisland_slanted_angle;
		manipulator[4].mode_angle=manipulator[4].bigisland_slanted_angle;
		manipulator[5].mode_angle=manipulator[5].bigisland_slanted_angle;
		manipulator[6].mode_angle=manipulator[6].bigisland_slanted_angle;
	}
}
/*******************************************************SMALL ISLAND***********************************************************************/
void small_island_clamp_one_init_box()
{
	clamp.clamp_flag = CLAMPING;
	
	if(HAL_GetTick() - clamp_action_times_old ==100)
	{
		clamp_upraise_flag = 3;
	}
		else if(HAL_GetTick() - clamp_action_times_old == 5000)
	{
			clamp.clamp_cmd     = 0;
			clamp.identify_cmd  = 0;
			big_island_mode = BIG_ISLAND_ORDINARY_MODE;		
	}
}

void small_island_clamp_handler_MKO()
{
	if(clamp.clamp_cmd)//大概取矿的动作是一样的，只不过放矿动作不同
	{
//		if(pick_box == INIT_BOX)
//		{
			small_island_clamp_one_init_box();		
//		}
//		else if(pick_box == FRIST_BOX)
//		{
//			small_island_clamp_one_frist_box();
//		}
	}
	else if(mode_prepareing_flag)
	{
		clamp.clamp_cmd 				= 0;
		clamp.clamp_flag				= CLAMPED;
		
		clamp_slide_flag=0;
		clamp_upraise_flag=0;
		manipulator[1].mode_angle=manipulator[1].smallisland_angle;
		manipulator[2].mode_angle=manipulator[2].smallisland_angle;
		manipulator[3].mode_angle=manipulator[3].smallisland_angle;
		manipulator[4].mode_angle=manipulator[4].smallisland_angle;
		manipulator[5].mode_angle=manipulator[5].smallisland_angle;
		manipulator[6].mode_angle=manipulator[6].smallisland_angle;
		clamp_action_times_old= HAL_GetTick();

		PUMP_ON

	}
}

/*******************************************EXCHANGE***************************************************************************/
uint8_t exchange_push_flag;
uint8_t exchange_start_push_flag;
 
uint8_t exchange_second_state     = 0; //防止误操作 防止还未完成第二箱兑换就直接进入下一箱
uint8_t exchange_clamp_ramp_flag;
uint8_t exchange_view_switch_flag;
//以上为旧的使用的变量

uint8_t exchange_time_init_flag=0;
uint8_t exchange_ready_flag=0;

void exchange_pick()
{
	uint16_t pick_actiong_times;
	uint8_t beging_pick_flag=1;
	if(!beging_pick_flag)
	{//这个大括号内写存矿动作
//			store_one_init_box();
		switch(have_box_number_old)
		{

			case 1:
			{
				if(pick_actiong_times-HAL_GetTick()<1000)
				{
					
				}
			
				picking_flag=0;
				have_box_number_old=0;
			}break;
			
			case 2:
			{
				if(pick_actiong_times-HAL_GetTick()<1000)
				{
					
				}
			
				picking_flag=0;
				have_box_number_old=1;
			}break;
			
			case 3:
			{
				if(pick_actiong_times-HAL_GetTick()<1000)
				{
					
				}
			
				picking_flag=0;
				have_box_number_old=2;
			}break;
		}
	}
	else
	{
		manipulator[1].mode_angle=manipulator[1].exchange_pick_angle;
		manipulator[2].mode_angle=manipulator[2].exchange_pick_angle;
		manipulator[3].mode_angle=manipulator[3].exchange_pick_angle;
		manipulator[4].mode_angle=manipulator[4].exchange_pick_angle;
		manipulator[5].mode_angle=manipulator[5].exchange_pick_angle;
		manipulator[6].mode_angle=manipulator[6].exchange_pick_angle;
			
	  store_action_times = HAL_GetTick();
		beging_pick_flag=0;
		
		picking_flag=1;
	}
}

void exchange_handler_MKO()
{

  if(clamp.exchange_cmd)
	{
		EXCHANGE_OFF
		if(HAL_GetTick() - exchange_action_times ==1500)	
		clamp.exchange_cmd=0;

	}
	else if(clamp.exchange_pick_cmd)//将矿石从框中取出
	{
		exchange_pick();//掏矿还是采用手动掏,防止兑换失败却又拿一个矿到手上
	}
	else if(clamp.kb_turn_left_cmd || clamp.kb_turn_right_cmd)
	{
		clamp_turn_2_handler();
		//clamp_test();
	}
	else if(mode_prepareing_flag)
	{
		exchange_action_times=HAL_GetTick();
		
		clamp_slide_flag=0;
		clamp_upraise_flag=0;		
		manipulator[0].mode_angle=manipulator[0].exchange_angle-small_model_data[2];
		manipulator[1].mode_angle=manipulator[1].exchange_angle-small_model_data[4]*SMALL_MODEL_PITCH_RATIO;
		manipulator[2].mode_angle=manipulator[2].exchange_angle-small_model_data[5];
		manipulator[3].mode_angle=manipulator[3].exchange_angle;
		manipulator[4].mode_angle=manipulator[4].exchange_angle;
		manipulator[5].mode_angle=manipulator[5].exchange_angle;
		manipulator[6].mode_angle=manipulator[6].exchange_angle;
		
		EXCHANGE_ON
		PUMP_ON
		exchange_ctrl_chassis_state=0;
	}
}

void defend_handler()
{
	
}




/*********************************************GROUND***************************************************************/

uint8_t ground_clamp_finish;
void ground_clamp_one_init_box()
{
	clamp.clamp_flag = CLAMPING;
//	
	if(HAL_GetTick() - clamp_action_times_old == 10)
	{
		clamp_upraise_flag = 1;
	}
	else if(HAL_GetTick() - clamp_action_times_old == 2500)
	{
		clamp_upraise_flag = 0;
	}
	else if(HAL_GetTick() - clamp_action_times_old == 3000)
	{
		manipulator[4].mode_angle-=90;
	}
	else if(HAL_GetTick() - clamp_action_times_old == 4000)
	{
		
		clamp.clamp_cmd = 0;
		
	}
}


void ground_clamp_one_frist_box()
{
	clamp.clamp_flag = CLAMPING;
//	
	if(HAL_GetTick() - clamp_action_times_old == 500)
	{
		clamp_upraise_flag = 1;
	}
	else if(HAL_GetTick() - clamp_action_times_old == 1000)
	{
		clamp_upraise_flag = 3;
	}
	else if(HAL_GetTick() - clamp_action_times_old == 1300)
	{
		clamp.clamp_cmd     = 0;
		clamp.identify_cmd  = 0;
//			clamp_upraise_flag  = 2;
//			pick_box = SECOND_BOX;//第一箱夹取完成
		chassis_mode = CHASSIS_NORMAL_MODE;
	}		
}
//右中左下轮下滚
void clamp_ground_handler_MKO()
{
	if(clamp.clamp_cmd)
	{           
//		if(pick_box == INIT_BOX)
	//	{
		ground_clamp_one_init_box();		
//		}
//		else if(pick_box == FRIST_BOX)
//		{
//			ground_clamp_one_frist_box();
	//	}
	}
	else if(mode_prepareing_flag)
	{
		clamp.clamp_cmd = 0;
		clamp_action_times_old = HAL_GetTick();
		clamp.clamp_flag = CLAMPED;
		
		PUMP_ON
		
		clamp_slide_flag=0;
		clamp_upraise_flag=3;
		manipulator[1].mode_angle=manipulator[1].ground_angle;
		manipulator[2].mode_angle=manipulator[2].ground_angle;
		manipulator[3].mode_angle=manipulator[3].ground_angle;
		manipulator[4].mode_angle=manipulator[4].ground_angle;
		manipulator[5].mode_angle=manipulator[5].ground_angle;
		manipulator[6].mode_angle=manipulator[6].ground_angle;
	}
}
void clamp_turn_handler()//翻转机构
{
	uint8_t i;
	for(i=0;i<=1;i++)
	{
		if(store_servo_angle[i]<=500)store_servo_angle[i]=500;
		if(store_servo_angle[i]>=2500)store_servo_angle[i]=2500;
	}
	
//	TIM_SetCompare2(TIM4,store_servo_angle[0]);
//	TIM_SetCompare3(TIM4,store_servo_angle[1]);
	if(pick_box == FRIST_BOX) manipulator[0].mode_angle = manipulator[0].store_angle;
	else manipulator[0].mode_angle = 0;
}
void clamp_turn_2_handler()
{
	if(HAL_GetTick() - exchange_action_times ==100)
	{
		clamp_slide_flag=5;
		clamp_upraise_flag=5;
		manipulator[1].mode_angle=manipulator[1].exchange_pick_angle_2;
		manipulator[2].mode_angle=manipulator[2].exchange_pick_angle_2;
		manipulator[3].mode_angle=manipulator[3].exchange_pick_angle_2;
		manipulator[4].mode_angle=manipulator[4].exchange_pick_angle_2;
		manipulator[5].mode_angle=manipulator[5].exchange_pick_angle_2;
		manipulator[6].mode_angle=manipulator[6].exchange_pick_angle_2;
	}
	else if(HAL_GetTick() - exchange_action_times ==1000)
	{
		pick_box = INIT_BOX;
		clamp_upraise_flag=6;
	}
	else if(HAL_GetTick() - exchange_action_times ==2000)
	{
		clamp_upraise_flag=5;
		clamp_slide_flag=6;
	}
	else if(HAL_GetTick() - exchange_action_times ==3000)
	{
		clamp_slide_flag=5;
		clamp_upraise_flag=5;
		manipulator[2].mode_angle-=80;
		if(clamp.kb_turn_left_cmd) manipulator[5].mode_angle-=90;
		if(clamp.kb_turn_right_cmd) manipulator[5].mode_angle+=90;
	}
//	else if(HAL_GetTick() - exchange_action_times ==2500)
//	{
//		clamp_slide_flag=5;
//	}
//	else if(HAL_GetTick() - exchange_action_times ==3000)
//	{
//		
//	}
	else if(HAL_GetTick() - exchange_action_times ==4500)
	{
		manipulator[2].mode_angle+=60;
	}
	else if(HAL_GetTick() - exchange_action_times ==4800)
	{		
		pick_box = FRIST_BOX;
	}
	else if(HAL_GetTick() - exchange_action_times ==5500)
	{
//		PUMP_OFF
	}
	else if(HAL_GetTick() - exchange_action_times ==6000)
	{
		manipulator[2].mode_angle-=70;
//		clamp_slide_flag=6;
//		clamp_upraise_flag=4;
	}
//		else if(HAL_GetTick() - exchange_action_times ==7000)
//	{
//		manipulator[2].mode_angle-=40;
////		clamp_slide_flag=5;
//	}
	else if(HAL_GetTick() - exchange_action_times ==6500)
	{
		clamp.kb_turn_left_cmd=0;
		clamp.kb_turn_right_cmd=0;
	}
}


