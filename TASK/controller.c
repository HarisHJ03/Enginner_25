#include "controller.h"
#include "STM32_TIM_BASE.h"
#include "modeswitch_task.h"
#include "judge_rx_data.h"
#include "tim.h"

gyrosope_data_t gyrosope_data,gyrosope_past_data;

int16_t small_model_data[AS5600_NUMBER];
int16_t model_out_angle[AS5600_NUMBER];

//static int read_encoder(void)
//{
//    int encoderNum = 0;
//    encoderNum = (int)((int16_t)(TIM4->CNT)); /*CNT为uint32, 转为int16*/
//    TIM_SetCounter(TIM4, CNT_INIT);/*CNT设初值*/

//    return encoderNum;
//}

void controller_task(void)
{
	usart_data_t usart_controller_data;
	gyrosope_data_t gyrosope_now_data;
	
	memcpy(&usart_controller_data,&judge_recv_mesg.robot_interactive_data,sizeof(usart_controller_data));
	
	switch(judge_recv_mesg.robot_interactive_data.data[0])//第一位为cmd_id,表示传回来的符号的意义
	{
		case MODEL_MODE://小模型模式
		{
			memcpy(&small_model_data,&usart_controller_data.main_data,sizeof(small_model_data));
		}break;
		case GYROSOPE_MODE://陀螺仪模式
		{
			memcpy(&gyrosope_now_data,&usart_controller_data.main_data,sizeof(gyrosope_data));
		}break;
	}
	if(chassis_mode != CHASSIS_EXCHANGE_MODE && chassis_mode != CHASSIS_ANGLE_MODE)
	{ 
		memset(&model_out_angle,0,sizeof(model_out_angle));
		
		memset(&gyrosope_data,0,CONTROLLER_DATA_LENGTH);
		memcpy(&gyrosope_past_data,&gyrosope_now_data,sizeof(gyrosope_now_data));//陀螺仪兑换前的数值在其他模式下不断记录用作处理零飘
		
	}
	else if(chassis_mode == CHASSIS_EXCHANGE_MODE||chassis_mode == CHASSIS_ANGLE_MODE)
	{
		model_out_angle[0]=small_model_data[0]*SMALL_MODEL_X_RATIO;
		model_out_angle[1]=small_model_data[1]*SMALL_MODEL_Y_RATIO;
		model_out_angle[2]=small_model_data[2]*SMALL_MODEL_Z_RATIO;
		model_out_angle[3]=small_model_data[3];
		model_out_angle[4]=small_model_data[4]*SMALL_MODEL_PITCH_RATIO;
		model_out_angle[5]=small_model_data[5];
		
		memcpy(&gyrosope_data,&gyrosope_now_data,sizeof(gyrosope_data));
		gyrosope_data.yaw_gyro_angle=gyrosope_now_data.yaw_gyro_angle-gyrosope_past_data.yaw_gyro_angle;
	}
//	counter_test=read_encoder();

}
//	if(chassis_mode == CHASSIS_EXCHANGE_MODE)//chassis_mode == CHASSIS_EXCHANGE_MODE
//	memcpy(&small_model_data,&judge_recv_mesg.robot_interactive_data,CONTROLLER_DATA_LENGTH);
//	else 
//	memset(&small_model_data,0,CONTROLLER_DATA_LENGTH);
