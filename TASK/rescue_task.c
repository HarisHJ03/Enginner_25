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
//**************************************************��Ԯģʽ********************************************************
//**********************************************���ڣ�22.3.20*******************************************************
//******* @���Ƚ���У׼������������Ԯģ�齵����͡�У׼Ŀ����ȷ���������ʱ�ĵ���Ƕȣ���ͨ���Ƕ�˫����ȷ���ƽǶ�   
//******* @У׼���ʹ�ܾ�Ԯ                                                                                         
//******* @����((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))ʹ��ץȡ��ץס������                                 
//******* @������ھ�Ԯģ����ץȡʱ������((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))ʹ�ܸ���                   
//******* @�ٴβ���((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))���ſ�ץס�Ļ�����                                 
//******* @���󣺲��������Ԯ������ſ�������ת���ȴ�rescue_cmd��1,-->��ʼ�պ���ɾ�Ԯ����-->�ȴ�rescue_cmd��0-->�ſ������� 
//*****************************��ܱ༭�ߣ��ƾ�����20�����鳤��   ����ע���ߣ���ĳ�㣨20������Ա��***************************
//***********************************************************************************************************************
//*********************************�޸ĸ���Ŀ����߼�****************************************���ڣ�2021.4.25**************
//****** @��Ϊ��������2006�Ŀ��ƹ����� ����˫������ʱ ������������Ǿ���û�дﵽԤ��Ч��
//**********************************************************************************************************************
//****** @���и��������������ز����ٶȻ����� ͨ���ж϶�ת������˫������
//********************************************************************************************************************/
///*********************************************�����Ϊ���׿���*****************************************************
//*/


//extern TaskHandle_t can_msg_send_Task_Handle;

//UBaseType_t rescue_stack_surplus;
//rescue_t rescue;
////uint8_t DEFEND_OUTPOST_STATE = 0; //��¼������˳�����ǰ��վ״̬ ���Ƶ���״̬

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
//uint8_t rescue_spd_ctrl_state[3] = {0};                     // Ϊ�����ý��ٶȻ�̧��������ñ�־λ
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
//*******************************�Ǿ�Ԯģʽ**********************************************************
//******************* @1.����δУ׼��������Ƶ�����ת���ж϶�ת����У׼
//******************* @2.��У׼��������Ƶ������У׼λ��
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
//							rescue.angle_ref[0] = rescue.init_angle[0] - normal_motoget_r;//����һ��������Ϊ�˱��ⷢ����ת����
//							rescue.angle_ref[1] = rescue.init_angle[1] + normal_motoget_l;
//							
//							/*�����ջ�*/
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
//							rescue_spd_ctrl_state[i] = 1;                                                      //�����ٶȻ���һ����־λ
//						}	
//								if(!rescue_spd_ctrl_state[i])
//								{
//									if((rescue.angle_ref[i]) <= moto_rescue[i].total_angle)//+
//										rescue.spd_ref[i]  =   -1000;	
//									else if((rescue.angle_ref[i]) >= moto_rescue[i].total_angle)
//										 rescue.spd_ref[i] =  1000;							
//									
//									rescue.current[i] = pid_calc(&pid_rescue_spd[i], rescue.spd_fdb[i], rescue.spd_ref[i]);//��������				
//								}
//								else
//								{	         					
//									pid_calc(&pid_rescue[i], moto_rescue[i].total_angle, rescue.angle_ref[i]);
//									rescue.current[i] = pid_calc(&pid_rescue_spd[i], rescue.spd_fdb[i], pid_rescue[i].out);//˫�����ƣ��͵������Ƶ��������ڸ���ֵ�Ĳ�ͬ��					
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
//  memset(&rescue, 0, sizeof(rescue_t));//�����ڴ沢��ʼ��
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

//	/*����ʧ��*/
//		RESCUE_OFF
//}

//uint32_t error_angle[3] = {0};
///**************************************************************************************/
//void rescue_init_handler(void)//У׼����(����û���⣬���Գ�������һ��Ϊ��)
//{	
//  static uint8_t  rescue_error_state[3] = {0};
//	
//  if((rescue.calibration_flag == 0) || (rescue.state != rescue.last_state))//�ǵõ���ȷ������ķ����ٸ���
//  {
//		if( rescue_error_state[0] != 1)
//     rescue.spd_ref[0] =   600;
//		if( rescue_error_state[1] != 1)
//     rescue.spd_ref[1] =  -600;
//  }	
//  for(int i = 0;i < 2;i++) 
//  {
//    if((fabs(pid_rescue_spd[i].set) - fabs(pid_rescue_spd[i].get)) > 0.9f*fabs(pid_rescue_spd[i].set))//�ж��Ƿ��ת��
//    {
//      error_angle[i]++;
//      if(error_angle[i] > 100)
//      {
//        rescue_error_state[i]    = 1;
//        rescue.spd_ref[i]        = 0;//������ֶ�ת���ٶȻ�����Ϊ0
//        rescue.calibration_flag  = 1;					
//      }
//    }
//  }
//  if(rescue_error_state[0] == 1  && rescue_error_state[1] == 1) //�������������ת��  
//  {
//    for(int i = 0;i < 3;i++)
//    {
//      rescue.init_angle[i] = moto_rescue[i].total_angle;//����תʱ�ĵ���Ƕȷ���ֵ����ΪУ׼�Ƕ�
//			
//			/*������ʼ���� �����䲻�ûص���ʼ��֮ǰ��״̬*/
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
//	if(rescue.state == INIT_NEVER)//������ж�ֻ�Ǳ������
//	{
//		for(int i = 0;i < 3;i++)
//		{
//			 rescue.current[i] = pid_calc(&pid_rescue_spd[i], rescue.spd_fdb[i], rescue.spd_ref[i]);//��������
//		}
//	}
//}
///***********************************************************************************************/
//uint8_t rescue_over = 0;

//void rescue_enable_handler(void)//#define RC_RESCUE_MODE_CMD ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP)) ����һ��ץס��һ�ν��ſ�
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
//		/*���������ǵ�����ӵģ���δ����Ƿ���� �������У����Ӻ�����while��Ӱ������ָ�����г��ֵ������������Ը�������if������֤ʵif else���У�
//		  Ԥ���������ץס�����˺󾭲����Ž��и��������ץȡͬʱ���и���
//		  ���ڣ�20.9.25*/
////		while(!RC_RESCUE_RESURGENCE_MODE)
////		{				
////		}
//		if(rescue.rescue_resurrection_cmd)//������;�ԮǶ�ף�����Ҫ���߷ֿ�
//		{
//			RESCUE_ON
////			rescue.angle_ref[2] = rescue.init_angle[2] + flex_motoget;
//		}
//		else
//		{
//			RESCUE_OFF
////			 rescue.angle_ref[2] = rescue.init_angle[2] + 10;       //��һ��������Ϊ�˷�ֹ��ת
//		}
//    rescue.recuse_flag = RECUSEING;

//	}
//  else
//  {

//		rescue.angle_ref[0] = rescue.init_angle[0] - rescue_ready_motoget_r;//����һ��������Ϊ�˱��ⷢ����ת����
//		rescue.angle_ref[1] = rescue.init_angle[1] + rescue_ready_motoget_l;
//						
//		for(uint8_t i = 0;i < 3;i++)
//	   	rescue_spd_ctrl_state[i] = 0;

//				RESCUE_OFF//����
////    rescue.angle_ref[2] = rescue.init_angle[2] + 10;//���� ��һ��������Ϊʵ�������������������������ض�� +2
//		
//		rescue.rescue_resurrection_cmd = 0;             //���㣬��ֹ�´ν����Ԯ��ֱ�ӽ��븴��
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
//			rescue_spd_ctrl_state[i] = 1;                                                      //�����ٶȻ���һ����־λ
//		}	
//				if(!rescue_spd_ctrl_state[i])
//				{
//					if(rescue.angle_ref[i] <= moto_rescue[i].total_angle)//+
//						rescue.spd_ref[i]  =   -1000;	
//					else if(rescue.angle_ref[i] >= moto_rescue[i].total_angle)
//						 rescue.spd_ref[i] =  1000;							
//					
//					rescue.current[i] = pid_calc(&pid_rescue_spd[i], rescue.spd_fdb[i], rescue.spd_ref[i]);//��������				
//				}
//				else
//				{	         					
//					pid_calc(&pid_rescue[i], moto_rescue[i].total_angle, rescue.angle_ref[i]);
//					rescue.current[i] = pid_calc(&pid_rescue_spd[i], rescue.spd_fdb[i], pid_rescue[i].out);//˫�����ƣ��͵������Ƶ��������ڸ���ֵ�Ĳ�ͬ��					
//				}
//	}
//}
