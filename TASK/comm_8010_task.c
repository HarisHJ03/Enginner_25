#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_can.h"
#include "pid.h"
#include "motor_8010.h"
#include "comm_8010_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "comm_8010_task.h"
#include "motor_task.h"
#include "math.h"

float send_current_8010_1 = 0;
float send_current_8010_2 = 0;

float send_ref_angle_8010_1 = 0;
float send_ref_angle_8010_2 = 0;

float send_fdb_angle_8010_1 = 0;
float send_fdb_angle_8010_2 = 0;
float angle_test = 0;
float speed_test = 0;

uint8_t motor8010_init_flag = 0;
fp32 motor8010_init_angle = 0;
fp32 test_8010_angle = 0;
fp32 test_test = 0;
uint8_t chazhi = 0;

uint8_t send1_8010_flag = 1; // 1表示可以发送，0表示不可发送
uint8_t send2_8010_flag = 0;

M8010_SendData m8010_senddata[4];

void go8010_task_1(void);
// void go8010_task_2(void);
float tamp_task(float input, float step, float ref_angle);
void m8010_init(void);
void go8010_send(uint8_t ID, M8010_SendData *m8010_senddata);
void m8010_stop(void);

void coom_8010_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	uint32_t comm_8010_time = osKernelSysTick();

	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL, // 等待前清零指定任务通知值的比特位（旧值对应bit清0）
								(uint32_t)INFO_SEND_MOTOR_SIGNAL | MODE_SWITCH_MSG_SIGNAL,
								(uint32_t *)&Signal,		// 用来取出通知值（如果不需要取出，可设为NULL）
								(TickType_t)portMAX_DELAY); // 设置阻塞时间
		if (STAUS == pdTRUE)
		{
			if (Signal == MODE_SWITCH_MSG_SIGNAL) //???? -- ??????
			{
				m8010_stop();
				vTaskDelayUntil(&comm_8010_time, 2);
			}
			if (Signal == INFO_SEND_MOTOR_SIGNAL) //??????
			{
				if (!motor8010_init_flag)
				{
					m8010_init();
					motor8010_init_flag = 1;
				}
				else
				{
					go8010_send(0, &m8010_senddata[0]);
					vTaskDelayUntil(&comm_8010_time, 2);
					go8010_send(2, &m8010_senddata[2]);
				}
			}

			vTaskDelayUntil(&comm_8010_time, 6); // 绝对延时函数
		}
	}
}
float tete = 2;

void go8010_send(uint8_t ID, M8010_SendData *m8010_senddata) // ID 0和1 2和3 不能同时用
{
	if (m8010_senddata[ID].Pos >= 3.14f)
		m8010_senddata[ID].Pos = 3.14f;
	GO_M8010_send_data(ID, m8010_senddata[ID].T, tamp_task(motor_recevie.Pos, tete, m8010_senddata[ID].Pos * 6.33f),
					   m8010_senddata[ID].W, m8010_senddata[ID].K_P, m8010_senddata[ID].K_W);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_Cmd(DMA2_Stream6, ENABLE);
	switch (ID)
	{
	case 0:
		send1_8010_flag = 0;
		break;
	case 1:
		send1_8010_flag = 0;
		break;
	case 2:
		send2_8010_flag = 0;
		break;
	case 3:
		send2_8010_flag = 0;
		break;
	default:
		break;
	}
}
void go8010_task_1(void)
{
	GO_M8010_send_data(0, 0, tamp_task(motor_recevie.Pos, tete, test_8010_angle), 0, 0.2, 0.05); // 计算发数		//0.1   0.05
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_Cmd(DMA2_Stream6, ENABLE);
	send1_8010_flag = 0;
}

void go_8010_test_tesk1(int id, float T, float Pos, float W, float K_P, float K_W)
{
	GO_M8010_send_data(id, T, Pos, W, K_P, K_W); // 计算发数
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_Cmd(DMA2_Stream6, ENABLE);
}

void m8010_stop(void)
{
	go_8010_test_tesk1(15, 0, 0, 0, 0, 0);
}

float tamp_task(float input, float step, float ref_angle)
{

	if (fabs(ref_angle - input) < step || fabs(input - ref_angle) < step)
		return ref_angle;
	else
	{
		if (input < ref_angle)
			input += step;

		else if (input > ref_angle)
			input -= step;
	}
	return input;
}
void m8010_init(void)
{
	// m8010_senddata[0].id = 0;
	m8010_senddata[0].K_P = 0.2;
	m8010_senddata[0].K_W = 0.05;
	m8010_senddata[0].Pos = 0;
	m8010_senddata[0].T = 0;
	m8010_senddata[0].W = 0;

	// m8010_senddata[1].id = 1;
	m8010_senddata[1].K_P = 0;
	m8010_senddata[1].K_W = 0;
	m8010_senddata[1].Pos = 0;
	m8010_senddata[1].T = 0;
	m8010_senddata[1].W = 0;

	// m8010_senddata[2].id = 2;
	m8010_senddata[2].K_P = 0.2;
	m8010_senddata[2].K_W = 0.05;
	m8010_senddata[2].Pos = 0;
	m8010_senddata[2].T = 0;
	m8010_senddata[2].W = 0;

	// m8010_senddata[3].id = 3;
	m8010_senddata[3].K_P = 0;
	m8010_senddata[3].K_W = 0;
	m8010_senddata[3].Pos = 0;
	m8010_senddata[3].T = 0;
	m8010_senddata[3].W = 0;
}
