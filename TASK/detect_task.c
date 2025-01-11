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
******三维数组举例：17个学校，每个学校有两个班，每个班有12个同学
******txt三维数组解说：17个不同模块，每个模块有两种状态，每个状态有12个字符进行说明
*******/
char txt[17][2][14] = {
                    {"",""},                             // 0
                    {"底盘右上正常","底盘右上异常"},       // 1
                    {"底盘左上正常","底盘左上异常"},       // 2
                    {"底盘左下正常","底盘左下异常"},       // 3
                    {"底盘右下正常","底盘右下异常"},       // 4
                    {"遥控正常"    ,"遥控异常"    },       // 5
                    {"夹取正常"    ,"夹取异常"   },        // 6
                    {"夹紧前正常"  ,"夹紧前异常" },        // 7
                    {"夹紧后正常"  ,"夹紧后异常" },        //8
                    {"抬升正常"    ,"抬升异常"   },        //9                   
                    {"救援左正常"  ,"救援左异常" },         //10
                    {"救援右正常"  ,"救援右异常" },         //11
                    {"复活正常"    ,"复活异常"   },         //12
                    {"搬运右正常"  ,"搬运右异常"   },       //13
										{"搬运左正常"  ,"搬运左异常"   },      //14
                    {"裁判系统正常","裁判系统异常"},       //15
                    {"小电脑正常"  ,"小电脑异常"  }};      //16 
uint32_t temp1,temp2;
void detect_task(void *parm)
{
  char message[100];
  char *point = NULL;
	uint32_t detect_wake_time = osKernelSysTick();//微秒级计时，线程管理函数
  while(1)
  {
    module_offline_detect();//获取夹取控制板的离线数据
		
    detect_state.offline = global_err.offline;//global_err.offline为上开发板出错数据；global_err.gimbal_offline为云台开发板数据
		
    if(detect_state.offline != detect_last_state.offline) //判断整体离线状态是否发生变化，若没有则没必要更新信息
    {
      taskENTER_CRITICAL();//任务代码里进入临界区函数
      temp2 = detect_state.offline;//发送异常位
      temp1 = (detect_state.offline ^ detect_last_state.offline) >> 1;//获取发生变化的位 为1说明变化，0则不变（^异或运算符,位值相同为0,不同为1）？？？为什么需要右移一位呢 因为id=0时是不需要显示的
      for(uint8_t id = CHASSIS_M1_OFFLINE; id <= PC_SYS_OFFLINE; id++)
      {
        if(temp1 & 0x00000001)//说明该位发生了变化，表明出现异常的逻辑处理
        {
          if((temp2 >> id)&0x00000001)//判断该位当前的状态 -- 异常的处理
          {
            global_err.list[id].err_exist = 1;
            
            sprintf(message,"page%d.t%d.pco=RED",(id/9)+1,(id-1)%8); 
						/*红色，显示对应页面及对应位置(
						sprintf的作用是将一个格式化的字符串输出到一个目的字符串中),sprintf返回字符长度
						sprintf第一个参数为message的首地址
						page%d.t%d.pco=RED中page%d表示显示在第几页，t%d处是显示在哪个文本框
						*/
            point = message;//message地址
            for(int i = 0;i < strlen((char*)message);i++ )//strlen()函数用来计算字符串的长度,strlen()会继续向后检索，直到遇到'\0'
            {
							/*USART_FLAG_TXE发送数据寄存器空标志位
							**1是该寄存器为空，可以往里存放数据*/

              while((UART8->SR & USART_FLAG_TXE) != USART_FLAG_TXE)//直至UART8->SR与USART_FLAG_TXE相同
              {
              }
              UART8->DR = (*point++ & (uint8_t)0xff);//数据寄存器
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
            sprintf(message,"page%d.t%d.txt=\"%s\"",(id/9)+1,(id-1)%8,txt[id][1]); //离线
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
          else/*正常的 显示*/
          {
            global_err.list[id].err_exist = 0;
            
            sprintf(message,"page%d.t%d.pco=GREEN",(id/9)+1,(id-1)%8); //绿色
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
            sprintf(message,"page%d.t%d.txt=\"%s\"",(id/9)+1,(id-1)%8,txt[id][0]); //在线
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
          temp1 = temp1 >> 1;//遍历所有
        }
      }
      detect_last_state.offline = detect_state.offline;//获取上一次离线状态
      taskEXIT_CRITICAL();
    }
    
    RCC_GetClocksFreq(&RCC_Clocks);//检测外部晶振是否起振
    
    
    
    
    /**************debug****************/
    
//    sprintf(message,"page%d.t%d.pco=RED",1,1); //红色
//    sprintf(message,"page%d.t%d.txt=\"%s\"",1,2,"离线"); //离线
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
    vTaskDelayUntil(&detect_wake_time, 50);//绝对延时函数，延时50次节拍
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
      Set_bit(global_err.offline,id);//标志异常位--置位
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
        /*用户处理代码*/
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
  if (chassis_mode != CHASSIS_CLAMP_SMALL_ISLAND_MODE //模式不为夹取模式	
	 || chassis_mode != CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE
//   || global_err.list[MCU_COMM_OFFLINE].err_exist   //单片机通信离线
   || global_err.list[REMOTE_CTRL_OFFLINE].err_exist  //远程控制离线
   || global_err.list[UPRAISE_M1_OFFLINE].err_exist)		//抬升电机离线
//	 || global_err.list[EXCHANGE_OFFLINE].err_exist)  	//兑换挡板
    return 0;
  else
    return 1;
}
uint8_t barrier_carry_is_controllable(void)
{
  if (chassis_mode != CHASSIS_BARRIER_CARRY_MODE      //模式不为夹取模式
//   || global_err.list[MCU_COMM_OFFLINE].err_exist     //单片机通信离线
   || global_err.list[REMOTE_CTRL_OFFLINE].err_exist) //远程控制离线
    return 0;
  else
    return 1;
}
/*UART8 中断函数*/
void UART8_IRQHandler(void)
{
	if(USART_GetFlagStatus(UART8,USART_FLAG_IDLE) != RESET 
		 && USART_GetITStatus(UART8,USART_IT_IDLE) != RESET)
	{
		USART_ReceiveData(UART8);		
		USART_ClearFlag(UART8, USART_FLAG_IDLE);//清除空闲中断标志位
    
	}
}

