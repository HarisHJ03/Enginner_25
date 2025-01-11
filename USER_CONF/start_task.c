/*********************************************************
*                    �� �� �� ��                       *
*                                                     *
*  ��                   _oo0oo_                 ��    *
*                      o8888888o                      *
*  ��                  88" . "88                ̤    *
*                      (| -_- |)                      *
*  ��                  0\  =  /0                ʵ    *
*                    ___/`--- \___                    *
*  ��              .' \\|     |// '.            ��    *
*                 / \\|||  :  |||// \                 *
*  ��            / _||||| -:- |||||- \          ��    *
*               |   | \\\  -  /// |   |               *
*  ʨ           | \_|  ''\---/''  |_/ |         ��    *
*               \  .-\__  '-'  ___/-. /               *
*  ��        ___'. .'  /--.--\  `. .'___        ��    *
*         ."" '<  `.___\_<|>_/___.' >' "".            *
*  ��     | | :  `- \`.;`\ _ /`;.`/ - ` : | |   ��    *
*         \  \ `_.   \_ __\ /__ _/   .-` /  /         *
*    =====`-.____`.___ \_____/___.-`___.-'=====       *
*                       `=---='                       *
*                                                     *
*                                                     *
*     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~     *
*                                                     *
*               ���汣��         ����BUG               *
******************************************************/

#include "start_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "imu_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "info_get_task.h"
#include "detect_task.h"

#include "gimbal_task.h"
#include "chassis_task.h"
#include "rescue_task.h"
#include "clamp_task.h"
#include "upraise_task.h"
#include "barrier_carry_task.h"
#include "supply_task.h"
#include "slide_task.h"

#include "motor_task.h"

#include "judge_task.h"
#include "pc_task.h"
#include "comm_8010_task.h"

#define START_TASK_SIZE 128  // 128 * 32 Bit = 128 * 4 Byte����ջ��
#define START_TASK_PRIO 2    // ���ȼ�

#define CHASSIS_TASK_SIZE 128
#define CHASSIS_TASK_PRIO 5

#define GIMBAL_TASK_SIZE 128 
#define GIMBAL_TASK_PRIO 5

#define CLAMP_TASK_SIZE 128
#define CLAMP_TASK_PRIO 5

#define UPRAISE_TASK_SIZE 128
#define UPRAISE_TASK_PRIO 5

#define BARRIER_CARRY_TASK_SIZE 128 //�����ϰ���
#define BARRIER_CARRY_TASK_PRIO 5

#define SUPPLY_TASK_SIZE 128
#define SUPPLY_TASK_PRIO 5

#define CAN_MSG_SEND_TASK_SIZE 128
#define CAN_MSG_SEND_TASK_PRIO 6

#define MODE_SWITCH_TASK_SIZE 128
#define MODE_SWITCH_TASK_PRIO 4

#define INFO_GET_TASK_SIZE 128
#define INFO_GET_TASK_PRIO 4

#define DETECT_TASK_SIZE 128
#define DETECT_TASK_PRIO 4

#define IMU_TASK_SIZE 128
#define IMU_TASK_PRIO 4

#define JUDEG_TX_TASK_SIZE 256
#define JUDEG_TX_TASK_PRIO 4

#define JUDEG_RX_TASK_SIZE 256
#define JUDEG_RX_TASK_PRIO 4

#define PC_TX_TASK_SIZE 256
#define PC_TX_TASK_PRIO 4

#define PC_RX_TASK_SIZE 256
#define PC_RX_TASK_PRIO 4

#define SLIDE_TASK_SIZE 128
#define SLIDE_TASK_PRIO 4

#define MOTOR_TASK_SIZE 256
#define MOTOR_TASK_PRIO 5

#define COMM_8010_TASK_SIZE 256
#define COMM_8010_TASK_PRIO 5

TaskHandle_t start_Task_Handle;

TaskHandle_t gimbal_Task_Handle;
TaskHandle_t chassis_Task_Handle;

TaskHandle_t clamp_Task_Handle;
TaskHandle_t upraise_Task_Handle;
TaskHandle_t barrier_carry_Task_Handle;
TaskHandle_t supply_Task_Handle;
TaskHandle_t can_msg_send_Task_Handle;
TaskHandle_t mode_switch_Task_Handle;
TaskHandle_t info_get_Task_Handle;
TaskHandle_t detect_Task_Handle;
TaskHandle_t imu_Task_Handle;
TaskHandle_t slide_Task_Handle;

TaskHandle_t judge_tx_Task_Handle;
TaskHandle_t judge_rx_Task_Handle;
TaskHandle_t pc_tx_Task_Handle;
TaskHandle_t pc_rx_Task_Handle;

TaskHandle_t motor_Task_Handle;
TaskHandle_t comm_8010_task_Handle;

void start_task(void *parm)
{
	taskENTER_CRITICAL();
	
	{
		/*�������� -- �Զ�����һ��������ƿ�*/                     
    	xTaskCreate((TaskFunction_t)chassis_task,			//������
					(const char *)"chassis_task",  			//��������
					(uint16_t)CHASSIS_TASK_SIZE,   			//��ջ��С����̬�ڴ����룩
					(void * )NULL,                 			//�����������Ĳ���
					(UBaseType_t)CHASSIS_TASK_PRIO,			//���ȼ�
					(TaskHandle_t *) &chassis_Task_Handle );//������
							
    	xTaskCreate((TaskFunction_t)gimbal_task,
					(const char *)"gimbal_task",
					(uint16_t)GIMBAL_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)GIMBAL_TASK_PRIO,
					(TaskHandle_t *) &gimbal_Task_Handle );
							
		xTaskCreate((TaskFunction_t)Motor_task,
					(const char *)"motor_task",
					(uint16_t)MOTOR_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)MOTOR_TASK_PRIO,
					(TaskHandle_t *) &motor_Task_Handle );
					
		xTaskCreate((TaskFunction_t)coom_8010_task,
					(const char *)"coom_8010_task",
					(uint16_t)COMM_8010_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)COMM_8010_TASK_PRIO,
					(TaskHandle_t *) &comm_8010_task_Handle );
							
		xTaskCreate((TaskFunction_t)clamp_task,
					(const char *)"clamp_task",
					(uint16_t)CLAMP_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)CLAMP_TASK_PRIO,
					(TaskHandle_t *) &clamp_Task_Handle );
							
		xTaskCreate((TaskFunction_t)upraise_task,
					(const char *)"upraise_task",
					(uint16_t)UPRAISE_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)UPRAISE_TASK_PRIO,
					(TaskHandle_t *) &upraise_Task_Handle );		

    	xTaskCreate((TaskFunction_t)barrier_carry_task,
					(const char *)"barrier_carry_task",
					(uint16_t)BARRIER_CARRY_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)BARRIER_CARRY_TASK_PRIO,
					(TaskHandle_t *) &barrier_carry_Task_Handle );	
								
    	xTaskCreate((TaskFunction_t)supply_task,
					(const char *)"supply_task",
					(uint16_t)SUPPLY_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)SUPPLY_TASK_PRIO,
					(TaskHandle_t *) &supply_Task_Handle );		

    	xTaskCreate((TaskFunction_t)slide_task,
					(const char *)"slide_task",
					(uint16_t)SLIDE_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)SLIDE_TASK_PRIO,
					(TaskHandle_t *) &slide_Task_Handle );	
  	}        

  	{
		xTaskCreate((TaskFunction_t)can_msg_send_task,
					(const char *)"can_msg_send_task",
					(uint16_t)CAN_MSG_SEND_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)CAN_MSG_SEND_TASK_PRIO,
					(TaskHandle_t *) &can_msg_send_Task_Handle );
	}
	
	{						
		xTaskCreate((TaskFunction_t)mode_switch_task,
					(const char *)"mode_switch_task",
					(uint16_t)MODE_SWITCH_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)MODE_SWITCH_TASK_PRIO,
					(TaskHandle_t *) &mode_switch_Task_Handle );
							
		xTaskCreate((TaskFunction_t)info_get_task,
					(const char *)"info_get_task",
					(uint16_t)INFO_GET_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)INFO_GET_TASK_PRIO,
					(TaskHandle_t *) &info_get_Task_Handle );
								
		xTaskCreate((TaskFunction_t)detect_task,
					(const char *)"detect_task",
					(uint16_t)DETECT_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)DETECT_TASK_PRIO,
					(TaskHandle_t *) &detect_Task_Handle );
								
//		xTaskCreate((TaskFunction_t)imu_task,
//					(const char *)"imu_task",
//					(uint16_t)IMU_TASK_SIZE,
//					(void * )NULL,
//					(UBaseType_t)IMU_TASK_PRIO,
//					(TaskHandle_t *) &imu_Task_Handle );
	}						
	
  	{
    	xTaskCreate((TaskFunction_t)judge_tx_task,
					(const char *)"judge_tx_task",
					(uint16_t)JUDEG_TX_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)JUDEG_TX_TASK_PRIO,
					(TaskHandle_t *) &judge_tx_Task_Handle );
	
    	xTaskCreate((TaskFunction_t)judge_rx_task,
					(const char *)"judge_rx_task",
					(uint16_t)JUDEG_RX_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)JUDEG_RX_TASK_PRIO,
					(TaskHandle_t *) &judge_rx_Task_Handle );
	
    	xTaskCreate((TaskFunction_t)pc_tx_task,
					(const char *)"pc_tx_task",
					(uint16_t)PC_TX_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)PC_TX_TASK_PRIO,
					(TaskHandle_t *) &pc_tx_Task_Handle );
	
    	xTaskCreate((TaskFunction_t)pc_rx_task,
					(const char *)"pc_rx_task",
					(uint16_t)PC_RX_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)PC_RX_TASK_PRIO,
					(TaskHandle_t *) &pc_rx_Task_Handle );
	
  	}
  
	vTaskDelete(start_Task_Handle);//ɾ����ʼ�����������������ɾ������ ɾ����������Ҳ��ʹ��NULL��
	
	taskEXIT_CRITICAL();					 //�˳��ٽ���	
}

void TASK_START(void)
{
	xTaskCreate((TaskFunction_t)start_task,
				(const char *)"start_task",
				(uint16_t)START_TASK_SIZE,
				(void * )NULL,
				(UBaseType_t)START_TASK_PRIO,
				(TaskHandle_t *) &start_Task_Handle );
}

