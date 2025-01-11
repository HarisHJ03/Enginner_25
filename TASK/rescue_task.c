//#include "rescue_task.h"
//#include "STM32_TIM_BASE.h"

//#include "FreeRTOSConfig.h"
//#include "FreeRTOS.h"
//#include "task.h"

//#include "modeswitch_task.h"
//#include "comm_task.h"
//#include "pid.h"
//#include "bsp_can.h"
//#include "math.h"
//#include "sys_config.h"
//#include "remote_ctrl.h"
//#include "gimbal_task.h"
//#include "keyboard.h"
//#include "barrier_carry_task.h"
///*****************************************************************************************************************
//**************************************************救援模式********************************************************
//**********************************************日期：22.3.20*******************************************************
//******* @首先进入校准，现象：两个救援模块降至最低。校准目的是确定降至最低时的电机角度，将通过角度双环精确控制角度   
//******* @校准完毕使能救援                                                                                         
//******* @操作((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))使能抓取，抓住机器人                                 
//******* @复活——在救援模块是抓取时，操作((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))使能复活                   
//******* @再次操作((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))将放开抓住的机器人                                 
//******* @现象：操作进入救援，电机张开发生堵转，等待rescue_cmd置1,-->开始闭合完成救援动作-->等待rescue_cmd置0-->放开机器人 
//*****************************框架编辑者：黄敬凯（20赛季组长）   调试注释者：张某秀（20赛季组员）***************************
//***********************************************************************************************************************
//*********************************修改复活的控制逻辑****************************************日期：2021.4.25**************
//****** @因为复活利用2006的控制过程中 利用双环进制时 伸出和缩回总是经常没有达到预想效果
//**********************************************************************************************************************
//****** @所有复活控制伸出和缩回采用速度环控制 通过判断堵转再利用双环控制
//********************************************************************************************************************/
///*********************************************复活改为气缸控制*****************************************************
//*/


//extern TaskHandle_t can_msg_send_Task_Handle;

//UBaseType_t rescue_stack_surplus;
//rescue_t rescue;
////uint8_t DEFEND_OUTPOST_STATE = 0; //记录进入和退出保卫前哨站状态 控制挡板状态

//float normal_motoget_r = 5;
//float normal_motoget_l = 13;

//float rescue_ready_motoget_r = 54;
//float rescue_ready_motoget_l = 54;

//float rescue_motoget_r =  120;
//float rescue_motoget_l =  115;

//float rescue_barrier_1_r = 45;
//float rescue_barrier_1_l = 40;

//float rescue_barrier_2_r = 74;
//float rescue_barrier_2_l = 74;

//float rescue_pid[6] = {20.0f, 0, 0.0f, 10.0f, 0, 0};
//uint8_t rescue_spd_ctrl_state[3] = {0};                     // 为了利用仅速度环抬升，定义该标志位
//void rescue_task(void *parm)
//{
//	uint32_t Signal;
//	BaseType_t STAUS;
//  while(1)
//  {
//    STAUS = xTaskNotifyWait((uint32_t) NULL, 
//										        (uint32_t) INFO_GET_RESCUE_SIGNAL, 
//									        	(uint32_t *)&Signal, 
//									        	(TickType_t) portMAX_DELAY );
//		if(STAUS == pdTRUE)
//		{
//			if((Signal & INFO_GET_RESCUE_SIGNAL) && chassis_mode != CHASSIS_RELEASE)
//			{
//				for(int i = 0; i<3; i++)
//				{
//          PID_Struct_Init(&pid_rescue[i],rescue_pid[0],rescue_pid[1],rescue_pid[2],5000, 500, DONE);
//          PID_Struct_Init(&pid_rescue_spd[i],rescue_pid[3],rescue_pid[4],rescue_pid[5],14000, 500, DONE);					
//				}
//        if(chassis_mode == CHASSIS_RESCUE_MODE)
//        {	
//          if(rescue.state == INIT_NEVER )
//          {
//             rescue_mode = RESCUE_INIT;
//          }       
//          switch(rescue_mode)
//          {
//            case RESCUE_INIT:
//            {
//              rescue_init_handler();
//            }break;
//            
//            case RESCUE_ENABLE:
//            {
//              rescue_enable_handler();
//            }break;
//            
//            default:
//            {             
//            }break;           
//          }			         
//        }
//        else  
//        {
///*************************************************************************************************
//*******************************非救援模式**********************************************************
//******************* @1.若从未校准过，则控制电机电机转动判断堵转进行校准
//******************* @2.若校准过，则控制电机保持校准位置
//**************************************************************************************************/					
//				if(rescue.state != INIT_DONE)
//				{
//					rescue_init_handler();
//				}
//				else
//				{
//					if(rescue.recuse_flag == RESCUEED)
//					{
//						if(rescue_mode == RESCUE_ENABLE)
//						{
//							rescue.angle_ref[0] = rescue.init_angle[0] - normal_motoget_r;//加上一个参数是为了避免发生堵转掉线
//							rescue.angle_ref[1] = rescue.init_angle[1] + normal_motoget_l;
//							
//							/*复活收回*/
//							RESCUE_OFF
//							for(uint8_t i = 0; i<3; i++)
//							{
//								rescue_spd_ctrl_state[i] = 0;//
//							}
//						}
//						else
//						{
//							rescue_init_handler();
//						}
//						rescue.rescue_cmd              = 0;
//						rescue.rescue_resurrection_cmd = 0;		
//					}
//				}
//			}
//        if(rescue_mode == RESCUE_ENABLE && rescue.state == INIT_DONE)		
//				{
//					for(uint8_t i = 0; i < 3;i++)
//					{
//						if(fabs(rescue.angle_ref[i] - moto_rescue[i].total_angle) < 10)//10
//						{						
//							rescue_spd_ctrl_state[i] = 1;                                                      //控制速度环的一个标志位
//						}	
//								if(!rescue_spd_ctrl_state[i])
//								{
//									if((rescue.angle_ref[i]) <= moto_rescue[i].total_angle)//+
//										rescue.spd_ref[i]  =   -1000;	
//									else if((rescue.angle_ref[i]) >= moto_rescue[i].total_angle)
//										 rescue.spd_ref[i] =  1000;							
//									
//									rescue.current[i] = pid_calc(&pid_rescue_spd[i], rescue.spd_fdb[i], rescue.spd_ref[i]);//单环控制				
//								}
//								else
//								{	         					
//									pid_calc(&pid_rescue[i], moto_rescue[i].total_angle, rescue.angle_ref[i]);
//									rescue.current[i] = pid_calc(&pid_rescue_spd[i], rescue.spd_fdb[i], pid_rescue[i].out);//双环控制（和单环控制的区别在于给定值的不同）					
//								}
//					}			
//				}
//				
//        xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle ,
//                           (uint32_t) RESCUE_MOTOR_MSG_SIGNAL, 
//                           (eNotifyAction) eSetBits, 
//                           (uint32_t *)NULL );
//        
////        rescue_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
//					if(chassis_mode == CHASSIS_RELEASE)
//					{
////						memset(glb_cur.rescue_cur, 0, sizeof(glb_cur.rescue_cur));
//					}
//					else 
//					{
////						memcpy(glb_cur.rescue_cur, rescue.current, sizeof(rescue.current));
//					}
//      }
//    }
//  }
//}
///******************************************************************************************/
//void rescue_param_init(void)
//{
//  memset(&rescue, 0, sizeof(rescue_t));//分配内存并初始化
//  
//  //rescue.calibration_flag   = 0;
//	
//	
//	rescue.recuse_flag = RESCUEED;
//  rescue.state       = INIT_NEVER;
//  rescue.last_state  = INIT_DONE;

//		for(int i = 0; i < 3; i++)
//  {
//    PID_Struct_Init(&pid_rescue[i],rescue_pid[0],rescue_pid[1],rescue_pid[2],5000, 500, INIT);
//    PID_Struct_Init(&pid_rescue_spd[i],rescue_pid[3],rescue_pid[4],rescue_pid[5],10000, 500, INIT);
//  }

//	/*复活失能*/
//		RESCUE_OFF
//}

//uint32_t error_angle[3] = {0};
///**************************************************************************************/
//void rescue_init_handler(void)//校准函数(调试没问题，所以出现问题一般为别处)
//{	
//  static uint8_t  rescue_error_state[3] = {0};
//	
//  if((rescue.calibration_flag == 0) || (rescue.state != rescue.last_state))//记得调试确定电机的方向再给大
//  {
//		if( rescue_error_state[0] != 1)
//     rescue.spd_ref[0] =   600;
//		if( rescue_error_state[1] != 1)
//     rescue.spd_ref[1] =  -600;
//  }	
//  for(int i = 0;i < 2;i++) 
//  {
//    if((fabs(pid_rescue_spd[i].set) - fabs(pid_rescue_spd[i].get)) > 0.9f*fabs(pid_rescue_spd[i].set))//判断是否堵转。
//    {
//      error_angle[i]++;
//      if(error_angle[i] > 100)
//      {
//        rescue_error_state[i]    = 1;
//        rescue.spd_ref[i]        = 0;//电机出现堵转后，速度环给定为0
//        rescue.calibration_flag  = 1;					
//      }
//    }
//  }
//  if(rescue_error_state[0] == 1  && rescue_error_state[1] == 1) //三个电机发生堵转后  
//  {
//    for(int i = 0;i < 3;i++)
//    {
//      rescue.init_angle[i] = moto_rescue[i].total_angle;//将堵转时的电机角度反馈值设置为校准角度
//			
//			/*按键初始化后 控制其不用回到初始化之前的状态*/
//		  rescue.angle_ref[0]  = rescue.init_angle[0] - normal_motoget_r;
//      rescue.angle_ref[1]  = rescue.init_angle[1] + normal_motoget_l;
//			RESCUE_OFF
//			
//      error_angle[i]          = 0;
//      rescue_error_state[i]   = 0;
//      rescue.calibration_flag = 0;      
//   		}
//		  rescue.state            = INIT_DONE;
//			rescue_mode             = RESCUE_ENABLE;
//  }
//	if(rescue.state == INIT_NEVER)//加入该判断只是保险起见
//	{
//		for(int i = 0;i < 3;i++)
//		{
//			 rescue.current[i] = pid_calc(&pid_rescue_spd[i], rescue.spd_fdb[i], rescue.spd_ref[i]);//单环控制
//		}
//	}
//}
///***********************************************************************************************/
//uint8_t rescue_over = 0;

//void rescue_enable_handler(void)//#define RC_RESCUE_MODE_CMD ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP)) 满足一次抓住再一次将放开
//{

//  if(rescue.rescue_cmd)
//  {

//		
//    rescue.angle_ref[0] = rescue.init_angle[0] - rescue_motoget_r;
//    rescue.angle_ref[1] = rescue.init_angle[1] + rescue_motoget_l;



//		
//		for(uint8_t i = 0;i < 3;i++)
//	   	rescue_spd_ctrl_state[i] = 0;
//		
//		/*以下三行是调试添加的，还未测得是否可行 （不可行，在子函数里while会影响其他指令运行出现电机疯等现象，所以改用下面if条件，证实if else可行）
//		  预想其可以在抓住机器人后经操作才进行复活，而不是抓取同时进行复活
//		  日期：20.9.25*/
////		while(!RC_RESCUE_RESURGENCE_MODE)
////		{				
////		}
//		if(rescue.rescue_resurrection_cmd)//将复活和救援嵌套，但又要两者分开
//		{
//			RESCUE_ON
////			rescue.angle_ref[2] = rescue.init_angle[2] + flex_motoget;
//		}
//		else
//		{
//			RESCUE_OFF
////			 rescue.angle_ref[2] = rescue.init_angle[2] + 10;       //减一个参数是为了防止堵转
//		}
//    rescue.recuse_flag = RECUSEING;

//	}
//  else
//  {

//		rescue.angle_ref[0] = rescue.init_angle[0] - rescue_ready_motoget_r;//加上一个参数是为了避免发生堵转掉线
//		rescue.angle_ref[1] = rescue.init_angle[1] + rescue_ready_motoget_l;
//						
//		for(uint8_t i = 0;i < 3;i++)
//	   	rescue_spd_ctrl_state[i] = 0;

//				RESCUE_OFF//缩回
////    rescue.angle_ref[2] = rescue.init_angle[2] + 10;//复活 加一参数是因为实际中其缩回量不够，所以缩回多点 +2
//		
//		rescue.rescue_resurrection_cmd = 0;             //置零，防止下次进入救援就直接进入复活
//		
//		rescue.recuse_flag = RESCUEED;

//  }
//}

//void barrier_rescue_enable()
//{
//	if(barrier_rescue_angle == 1)
//	{
//		rescue.angle_ref[0] = rescue.init_angle[0] - rescue_barrier_1_r;
//    rescue.angle_ref[1] = rescue.init_angle[1] + rescue_barrier_1_l;
//	}
//	else if(barrier_rescue_angle == 2)
//	{
//		rescue.angle_ref[0] = rescue.init_angle[0] - rescue_barrier_2_r;
//    rescue.angle_ref[1] = rescue.init_angle[1] + rescue_barrier_2_l;
//	}
//	else
//	{
//	  rescue.angle_ref[0] = rescue.init_angle[0] - normal_motoget_r;
//    rescue.angle_ref[1] = rescue.init_angle[1] + normal_motoget_l;
//	}
//	
//	for(uint8_t i = 0; i < 3;i++)
//	{
//		if(fabs(rescue.angle_ref[i] - moto_rescue[i].total_angle) < 10)//10
//		{						
//			rescue_spd_ctrl_state[i] = 1;                                                      //控制速度环的一个标志位
//		}	
//				if(!rescue_spd_ctrl_state[i])
//				{
//					if(rescue.angle_ref[i] <= moto_rescue[i].total_angle)//+
//						rescue.spd_ref[i]  =   -1000;	
//					else if(rescue.angle_ref[i] >= moto_rescue[i].total_angle)
//						 rescue.spd_ref[i] =  1000;							
//					
//					rescue.current[i] = pid_calc(&pid_rescue_spd[i], rescue.spd_fdb[i], rescue.spd_ref[i]);//单环控制				
//				}
//				else
//				{	         					
//					pid_calc(&pid_rescue[i], moto_rescue[i].total_angle, rescue.angle_ref[i]);
//					rescue.current[i] = pid_calc(&pid_rescue_spd[i], rescue.spd_fdb[i], pid_rescue[i].out);//双环控制（和单环控制的区别在于给定值的不同）					
//				}
//	}
//}
