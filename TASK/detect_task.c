#include "detect_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "bsp_can.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

RCC_ClocksTypeDef RCC_Clocks;

UBaseType_t detect_stack_surplus;

global_err_t global_err;

State_t detect_state,detect_last_state;

/*******
******��ά���������17��ѧУ��ÿ��ѧУ�������࣬ÿ������12��ͬѧ
******txt��ά�����˵��17����ͬģ�飬ÿ��ģ��������״̬��ÿ��״̬��12���ַ�����˵��
*******/
char txt[17][2][14] = {
                    {"",""},                             // 0
                    {"������������","���������쳣"},       // 1
                    {"������������","���������쳣"},       // 2
                    {"������������","���������쳣"},       // 3
                    {"������������","���������쳣"},       // 4
                    {"ң������"    ,"ң���쳣"    },       // 5
                    {"��ȡ����"    ,"��ȡ�쳣"   },        // 6
                    {"�н�ǰ����"  ,"�н�ǰ�쳣" },        // 7
                    {"�н�������"  ,"�н����쳣" },        //8
                    {"̧������"    ,"̧���쳣"   },        //9                   
                    {"��Ԯ������"  ,"��Ԯ���쳣" },         //10
                    {"��Ԯ������"  ,"��Ԯ���쳣" },         //11
                    {"��������"    ,"�����쳣"   },         //12
                    {"����������"  ,"�������쳣"   },       //13
										{"����������"  ,"�������쳣"   },      //14
                    {"����ϵͳ����","����ϵͳ�쳣"},       //15
                    {"С��������"  ,"С�����쳣"  }};      //16 
uint32_t temp1,temp2;
void detect_task(void *parm)
{
  char message[100];
  char *point = NULL;
	uint32_t detect_wake_time = osKernelSysTick();//΢�뼶��ʱ���̹߳�����
  while(1)
  {
    module_offline_detect();//��ȡ��ȡ���ư����������
		
    detect_state.offline = global_err.offline;//global_err.offlineΪ�Ͽ�����������ݣ�global_err.gimbal_offlineΪ��̨����������
		
    if(detect_state.offline != detect_last_state.offline) //�ж���������״̬�Ƿ����仯����û����û��Ҫ������Ϣ
    {
      taskENTER_CRITICAL();//�������������ٽ�������
      temp2 = detect_state.offline;//�����쳣λ
      temp1 = (detect_state.offline ^ detect_last_state.offline) >> 1;//��ȡ�����仯��λ Ϊ1˵���仯��0�򲻱䣨^��������,λֵ��ͬΪ0,��ͬΪ1��������Ϊʲô��Ҫ����һλ�� ��Ϊid=0ʱ�ǲ���Ҫ��ʾ��
      for(uint8_t id = CHASSIS_M1_OFFLINE; id <= PC_SYS_OFFLINE; id++)
      {
        if(temp1 & 0x00000001)//˵����λ�����˱仯�����������쳣���߼�����
        {
          if((temp2 >> id)&0x00000001)//�жϸ�λ��ǰ��״̬ -- �쳣�Ĵ���
          {
            global_err.list[id].err_exist = 1;
            
            sprintf(message,"page%d.t%d.pco=RED",(id/9)+1,(id-1)%8); 
						/*��ɫ����ʾ��Ӧҳ�漰��Ӧλ��(
						sprintf�������ǽ�һ����ʽ�����ַ��������һ��Ŀ���ַ�����),sprintf�����ַ�����
						sprintf��һ������Ϊmessage���׵�ַ
						page%d.t%d.pco=RED��page%d��ʾ��ʾ�ڵڼ�ҳ��t%d������ʾ���ĸ��ı���
						*/
            point = message;//message��ַ
            for(int i = 0;i < strlen((char*)message);i++ )//strlen()�������������ַ����ĳ���,strlen()�������������ֱ������'\0'
            {
							/*USART_FLAG_TXE�������ݼĴ����ձ�־λ
							**1�ǸüĴ���Ϊ�գ���������������*/

              while((UART8->SR & USART_FLAG_TXE) != USART_FLAG_TXE)//ֱ��UART8->SR��USART_FLAG_TXE��ͬ
              {
              }
              UART8->DR = (*point++ & (uint8_t)0xff);//���ݼĴ���
            }
            for(int i = 0;i < 3;i++ )
            {
              for(int j = 0;j < sizeof(uint8_t);j++ )
              {
                 while((UART8->SR & USART_FLAG_TXE) != USART_FLAG_TXE)
                {
                }
                UART8->DR = (uint8_t)0xff;
              }
            }
            sprintf(message,"page%d.t%d.txt=\"%s\"",(id/9)+1,(id-1)%8,txt[id][1]); //����
            point = message;
            for(int i = 0;i < strlen((char*)message);i++ )
            {
              while((UART8->SR & USART_FLAG_TXE) != USART_FLAG_TXE)
              {
              }
              UART8->DR = (*point++ & (uint8_t)0xff);
            }
            for(int i = 0;i < 3;i++ )
            {
              for(int j = 0;j < sizeof(uint8_t);j++ )
              {
                while((UART8->SR & USART_FLAG_TXE) != USART_FLAG_TXE)
                {
                }
                UART8->DR = (uint8_t)0xff;
              }
            }
          }
          else/*������ ��ʾ*/
          {
            global_err.list[id].err_exist = 0;
            
            sprintf(message,"page%d.t%d.pco=GREEN",(id/9)+1,(id-1)%8); //��ɫ
            point = message;
            for(int i = 0;i < strlen((char*)message);i++ )
            {							 
              while((UART8->SR & USART_FLAG_TXE) != USART_FLAG_TXE)
              {
              }
              UART8->DR = (*point++ & (uint8_t)0xff);
            }
            for(int i = 0;i < 3;i++ )
            {
              for(int j = 0;j < sizeof(uint8_t);j++ )
              {
                while((UART8->SR & USART_FLAG_TXE) != USART_FLAG_TXE)
                {
                }
                UART8->DR = (uint8_t)0xff;
              }
            }
            sprintf(message,"page%d.t%d.txt=\"%s\"",(id/9)+1,(id-1)%8,txt[id][0]); //����
            point = message;
            for(int i = 0;i < strlen((char*)message);i++ )
            {
              while((UART8->SR & USART_FLAG_TXE) != USART_FLAG_TXE)
              {
              }
              UART8->DR = (*point++ & (uint8_t)0xff);
            }
            for(int i = 0;i < 3;i++ )
            {
              for(int j = 0;j < sizeof(uint8_t);j++ )
              {
                while((UART8->SR & USART_FLAG_TXE) != USART_FLAG_TXE)
                {
                }
                UART8->DR = (uint8_t)0xff;
              }
            }
          }
        }
        else
        {
          temp1 = temp1 >> 1;//��������
        }
      }
      detect_last_state.offline = detect_state.offline;//��ȡ��һ������״̬
      taskEXIT_CRITICAL();
    }
    
    RCC_GetClocksFreq(&RCC_Clocks);//����ⲿ�����Ƿ�����
    
    
    
    
    /**************debug****************/
    
//    sprintf(message,"page%d.t%d.pco=RED",1,1); //��ɫ
//    sprintf(message,"page%d.t%d.txt=\"%s\"",1,2,"����"); //����
//    point = message;
//    for(int i = 0;i <= strlen(message);i++ )
//    {
//        while((UART8->SR & USART_FLAG_TXE) != USART_FLAG_TXE)
//        {
//        }
//        UART8->DR = (*point++ & (uint8_t)0xFF);
//    }
    
    /*****************debug*********************/
    
    detect_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    
    vTaskDelayUntil(&detect_wake_time, 50);//������ʱ��������ʱ50�ν���
  }
}


void detect_param_init(void)
{
  for(uint8_t id = CHASSIS_M1_OFFLINE; id <= JUDGE_SYS_OFFLINE; id++)//
  {
    global_err.list[id].param.set_timeout = 500;
    global_err.list[id].param.last_times  = 0;
    global_err.list[id].param.delta_times = 0;
    global_err.list[id].err_exist         = 0;
    global_err.err_now_id[id]             = BOTTOM_DEVICE;
  }
  global_err.list[PC_SYS_OFFLINE].param.set_timeout = 2000;
  global_err.list[PC_SYS_OFFLINE].param.last_times  = 0;
  global_err.list[PC_SYS_OFFLINE].param.delta_times = 0;
  global_err.list[PC_SYS_OFFLINE].err_exist         = 0;
  global_err.err_now_id[PC_SYS_OFFLINE]             = BOTTOM_DEVICE;
}

void err_detector_hook(int err_id)
{
  global_err.list[err_id].param.last_times = HAL_GetTick();
}

void module_offline_detect(void)
{
  for (uint8_t id = CHASSIS_M1_OFFLINE; id <= PC_SYS_OFFLINE; id++)
  {
    global_err.list[id].param.delta_times = HAL_GetTick() - global_err.list[id].param.last_times;
		
    if(global_err.list[id].param.delta_times > global_err.list[id].param.set_timeout)
    {
      global_err.err_now_id[id]     = (err_id)id;
      global_err.list[id].err_exist = 1;
      Set_bit(global_err.offline,id);//��־�쳣λ--��λ
    }
    else
    {
      global_err.err_now_id[id]     = BOTTOM_DEVICE;
      global_err.list[id].err_exist = 0;
      Reset_bit(global_err.offline,id);
    }
  }
}
void module_offline_callback(void)
{
  for(uint8_t id = CHASSIS_M1_OFFLINE; id <= PC_SYS_OFFLINE; id++)
  {
    switch(global_err.err_now_id[id])
    {
      case CHASSIS_M1_OFFLINE:
      {
//        Set_bit(global_err.offline,CHASSIS_M1_OFFLINE);
      }break;
      case CHASSIS_M2_OFFLINE:
      {
//        Set_bit(global_err.offline,CHASSIS_M2_OFFLINE);
      }break;       
      case CHASSIS_M3_OFFLINE:
      {
//        Set_bit(global_err.offline,CHASSIS_M3_OFFLINE);
      }break;
      case CHASSIS_M4_OFFLINE:
      {
//        Set_bit(global_err.offline,CHASSIS_M4_OFFLINE);
      }break;        
      case CLAMP_M1_OFFLINE:
      {
//        Set_bit(global_err.offline,CLAMP_M1_OFFLINE);
      }break;         			
      case UPRAISE_M1_OFFLINE:
      {
//        Set_bit(global_err.offline,UPRAISE_M1_OFFLINE);
      }break;
      case UPRAISE_M2_OFFLINE:
      {
//        Set_bit(global_err.offline,UPRAISE_M2_OFFLINE);
      }break; 			
      case RESCUE_M1_OFFLINE:
      {
//        Set_bit(global_err.offline,RESCUE_M1_OFFLINE);
      }break;       
      case RESCUE_M2_OFFLINE:
      {
//        Set_bit(global_err.offline,RESCUE_M2_OFFLINE);
      }break;   
//      case BARRIER_CARRY_M1_OFFLINE:
//      {
////        Set_bit(global_err.offline,BARRIER_CARRY_M1_OFFLINE);
//      }break;  
//     case BARRIER_CARRY_M2_OFFLINE:
//      {
////        Set_bit(global_err.offline,BARRIER_CARRY_M2_OFFLINE);
//      }break;  			
      case JUDGE_SYS_OFFLINE:
      {
//        Set_bit(global_err.offline,JUDGE_SYS_OFFLINE);
      }break;        
      case PC_SYS_OFFLINE:
      {
//        Set_bit(global_err.offline,PC_SYS_OFFLINE);
      }break;
      default:
      {
        /*�û��������*/
      }break;
    }
  }
}

//uint8_t gimbal_is_controllable(void)
//{
//  if (gimbal_mode == GIMBAL_RELEASE
//   || global_err.list[REMOTE_CTRL_OFFLINE].err_exist
//   || global_err.list[GIMBAL_YAW_OFFLINE].err_exist
//   || global_err.list[GIMBAL_PIT_OFFLINE].err_exist)
//    return 0;
//  else
//    return 1;
//}
//uint8_t shoot_is_controllable(void)
//{
//  if (shoot_mode == SHOOT_DISABLE 
//   || global_err.list[REMOTE_CTRL_OFFLINE].err_exist)
//    return 0;
//  else
//    return 1;
//}

uint8_t chassis_is_controllable(void)
{
  if (chassis_mode == CHASSIS_RELEASE 
   || global_err.list[REMOTE_CTRL_OFFLINE].err_exist)
    return 0;
  else
    return 1;
}

uint8_t rescue_is_controllable(void)
{
  if (chassis_mode != CHASSIS_RESCUE_MODE 
//   || global_err.list[MCU_COMM_OFFLINE].err_exist
   || global_err.list[REMOTE_CTRL_OFFLINE].err_exist
   || global_err.list[RESCUE_M1_OFFLINE].err_exist
   || global_err.list[RESCUE_M2_OFFLINE].err_exist)
    return 0;
  else
    return 1;
}
uint8_t clamp_is_controllable(void)
{
  if (chassis_mode != CHASSIS_CLAMP_SMALL_ISLAND_MODE //ģʽ��Ϊ��ȡģʽ	
	 || chassis_mode != CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE
//   || global_err.list[MCU_COMM_OFFLINE].err_exist   //��Ƭ��ͨ������
   || global_err.list[REMOTE_CTRL_OFFLINE].err_exist  //Զ�̿�������
   || global_err.list[UPRAISE_M1_OFFLINE].err_exist)		//̧���������
//	 || global_err.list[EXCHANGE_OFFLINE].err_exist)  	//�һ�����
    return 0;
  else
    return 1;
}
uint8_t barrier_carry_is_controllable(void)
{
  if (chassis_mode != CHASSIS_BARRIER_CARRY_MODE      //ģʽ��Ϊ��ȡģʽ
//   || global_err.list[MCU_COMM_OFFLINE].err_exist     //��Ƭ��ͨ������
   || global_err.list[REMOTE_CTRL_OFFLINE].err_exist) //Զ�̿�������
    return 0;
  else
    return 1;
}
/*UART8 �жϺ���*/
void UART8_IRQHandler(void)
{
	if(USART_GetFlagStatus(UART8,USART_FLAG_IDLE) != RESET 
		 && USART_GetITStatus(UART8,USART_IT_IDLE) != RESET)
	{
		USART_ReceiveData(UART8);		
		USART_ClearFlag(UART8, USART_FLAG_IDLE);//��������жϱ�־λ
    
	}
}

