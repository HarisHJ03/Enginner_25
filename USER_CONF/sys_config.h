#ifndef _sys_config_H
#define _sys_config_H

#include "stm32f4xx.h"

#define DEBUG

#define INFANTRY_1  1
#define INFANTRY_2  2
#define INFANTRY_3  3
#define INFANTRY_4  4
#define INFANTRY_5  5

/*����ģʽ С���ݻ���Сè��*/
#define LG 1
#define TWIST 2

#define  INFANTRY_NUM  INFANTRY_1
#define  DODGE_MODE LG //TWIST
/**********************ң�� ��������***************************/
/* ҡ�����ֵ */
#define RC_RESOLUTION     660.0f
/*************************���� ����*******************************/
/* ң��ģʽ  ���� �ٶ� ���� */
/* ǰ�� �ٶ� (mm/s) */
#define CHASSIS_RC_MAX_SPEED_X  3300.0f
#define CHASSIS_RC_MOVE_RATIO_X 1.0f
/* ���� �ٶ� (mm/s) */
#define CHASSIS_RC_MAX_SPEED_Y  3300.0f
#define CHASSIS_RC_MOVE_RATIO_Y 1.0f
/* ���� ��ת �ٶ� (deg/s) */
#define CHASSIS_RC_MAX_SPEED_R  480.0f
#define CHASSIS_RC_MOVE_RATIO_R 1.0f

/* ����ģʽ  ���� �ٶ� ���� */
/* ���� �ٶ� (mm/s) */
#define CHASSIS_KB_MAX_SPEED_X  1000.0f 
#define CHASSIS_KB_MOVE_RATIO_X 1.0f
/* ǰ�� �ٶ� (mm/s) */
#define CHASSIS_KB_MAX_SPEED_Y  1000.0f
#define CHASSIS_KB_MOVE_RATIO_Y 1.0f
/* ���� ��ת �ٶ� (deg/s) */
#define CHASSIS_KB_MAX_SPEED_R  600.0f
#define CHASSIS_KB_MOVE_RATIO_R 1.0f
/**************************��̨ ����*******************************/
/* ң��ģʽ ��̨�ٶȿ��� */
/* pitch �� �ٶ� */
#define GIMBAL_RC_MOVE_RATIO_PIT 1.0f
/* yaw �� �ٶ� */
#define GIMBAL_RC_MOVE_RATIO_YAW 1.0f

/* ң��ģʽ ��̨�ٶȿ��� */
/* pitch �� �ٶ� */
#define GIMBAL_PC_MOVE_RATIO_PIT 0.7f
/* yaw �� �ٶ� */
#define GIMBAL_PC_MOVE_RATIO_YAW 0.7f

/**************************��� ����********************************/
/* ���� */
#define DEFAULT_FRIC_WHEEL_SPEED 2000 //maximum value is 2500
/* ���̵������ */
#define TRIGGER_MOTOR_SPEED      -2000 //1500
/* ���̵������ */
#define C_TRIGGER_MOTOR_SPEED    -3000	//�²� /************************ ����Ӳ�� ���� ****************************/
/* ���ְ뾶(mm) */
#define RADIUS                 76
/* �����ܳ�(mm) */
#define PERIMETER              478

/*�־ࣨ���ң�*/
#define WHEELTRACK             375//415
/*��ࣨǰ��*/
#define WHEELBASE              365//406

/*��̨ƫ�Ƶ�������X�ᣨǰ�󣩾���*/
#define GIMBAL_X_OFFSET        -9//130
/*��̨ƫ�Ƶ�������Y�ᣨ���ң�����*/
#define GIMBAL_Y_OFFSET        0//0

/* ���̵�� 3508 */
/* ���̵���ļ��ٱ� */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* ��һ 3508 ����� ��� ת��, unit is rpm */
#define MAX_WHEEL_RPM        8500  //8347rpm = 3500mm/s
/* ���� ��� ƽ���ٶ� , unit is mm/s */
#define MAX_CHASSIS_VX_SPEED 3300  //8000rpm
#define MAX_CHASSIS_VY_SPEED 3300
/* ���� ��� ��ת�ٶ� , unit is degree/s */
#define MAX_CHASSIS_VR_SPEED 480   //5000rpm
/*С����ת��*/
#define LG_SPEED	300
/*С���ݻ�Сè�������VW*/
#define MAX_DODGE_SPEED	400

/************************** ��̨Ӳ�� ���� *****************************/
/* �������ֵ �� �Ƕȣ��ȣ� �ı��� */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)
/* pitch�� ��� �� ���ٱ� */
#define PIT_DECELE_RATIO       1.0f
/* yaw�� ��� �� ���ٱ� */
#define YAW_DECELE_RATIO       1.0f
/* pitch�� ��������� */
#define PIT_MOTO_POSITIVE_DIR  -1.0f
/* yaw�� ��������� */
#define YAW_MOTO_POSITIVE_DIR  1.0f
/* ���� ������������ */
#define TRI_MOTO_POSITIVE_DIR  1.0f




/*********************** ϵͳ �����ӿ� ���� ****************************/

/* CAN ��� */
#define CHASSIS_CAN       CAN1
#define RESCUE_CAN        CAN1
#define UPRAISE_CAN       CAN2
#define POWER_CAN         CAN1
#define SUPER_CAP_CAN     CAN1


#define CLAMP_EXCHANGE_CAN         	CAN2
#define DETECT_CAN        					CAN2
/* UART ��� */
/**
  * @attention
  * close usart DMA receive interrupt, so need add 
  * uart_receive_handler() before HAL_UART_IROHandler() in uart interrupt function
**/
#define DBUS_HUART         huart1 //ң�ؽ�����
#define JUDGE_HUART        huart3 //����ϵͳ�ӿ�
#define COMPUTER_HUART     huart6 //MINI���Խӿ�

/* ��̨ ��� */
//#define PIT_ANGLE_MAX      15//18//15
//#define PIT_ANGLE_MIN      -25//20//-25
#define YAW_ANGLE_MAX      50
#define YAW_ANGLE_MIN      -50

/* ������� ��� */
#define DEFAULT_TUNE       0	//����

/*���幦��*/
#define POWER_BUFFER			 60

/*�������*/
#define	SYSTEM_DELAY	0.3

/* IMU �¶ȿ��� */
#define DEFAULT_IMU_TEMP   50

/* ���� ��� */
/* ���� ϵ�� */
#define RADIAN_COEF        57.3f
/* Բ���� */
#define PI                 3.142f

//�ж������Сֵ
#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#endif


