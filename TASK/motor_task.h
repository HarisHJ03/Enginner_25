#ifndef _moto
#define _moto

#include "pid.h"
#include "fuzzy_pid.h"
#include "string.h"

#include "User.h"
/**************�����Ŀ����*************/
#define MOTOR_NUMBER 16
#define MODE_ANGLE_NUMBER 9
#define MOTOR_MAX_ID GIMBAL_YAW
/*************************************/
#define ON 1
#define OFF 0
#define ANGLE_MODE 2
#define SPEED_MODE 1

#ifdef BACK_DRIVE
#define NORMAL_LOOK 255
#define MODE_LOOK -193
#else
#define NORMAL_LOOK 62
#define MODE_LOOK 0
#endif

#define GO_8010_1   0
#define GO_8010_2   2
/**************���������*************/
typedef enum
{
//����ΪMOTOR_NAME �𼶵�����������
  CHASSIS_FR=0,
  CHASSIS_FL ,
  CHASSIS_BL ,
  CHASSIS_BR ,

  UPRISE_L ,
  UPRISE_R ,

  CLAMP_YAW ,
  CLAMP_PITCH ,
  CLAMP_ROLL ,

  SLIDE_L ,
  SLIDE_R ,
  joint_3_roll ,

  joint_1 ,
  joint_2 ,
  EXCHANGE_ROLL ,
	
	GIMBAL_YAW,
}motor_name_status;

typedef enum
{
  M6020=0 ,
  M3508 ,
  M2006 ,
	go_8010,
  SERVO ,
}motor_type_status;

//typedef enum
//{
//  M_UN_TURN=0  ,
//	M_TURN  ,
//}motor_turn_flag_status;

typedef struct
{
	uint8_t state;//��ʼ����ɣ� ɾ
	uint8_t offset_angle_init_flag;//��ʼ����־
	int16_t offset_angle_init_speed;//��ת��ʼ���ٶ�
	uint16_t err_count;

	uint8_t CAN_ID;
	uint8_t ESC_ID;//���ID
	uint8_t GO_ID;
	
	uint8_t Angle_to_Speed_mode;//�Ƿ��ڽǶȲ����ʱ�л����ٶȻ�
	uint8_t Speed_or_Angle_flag;//ѡ���ٶȻ�ǶȻ�
	int16_t speed;//�ǶȲ����ֱ�Ӹ������ٶ�
	
	int32_t ecd_fdb;
	float torque_fbd;
	float torque_ref;
  float angle_fdb;//�ǶȻ�get
	//float l_angle_fdb;
	int32_t angle_ref;//�ǶȻ�Ŀ��
  float spd_fdb;//�ٶȻ�get
  float spd_ref;//�ٶȻ�Ŀ��
	float current_read;//����
	
	int32_t current_send;//����
	
	
	
	
	
	pid_t angle_pid;
	pid_t speed_pid;
}Brushless_motor_t;

typedef struct
{
	TIM_TypeDef* TIM;
	uint8_t Compare;
	int16_t angle_ref;
	
	uint16_t Rotation_range;//��ת��Χ
}Servo_motor_t;

typedef struct
{
	int16_t init_angle;//��ʼ���õ��Ļ�׼
	int16_t offset_angle;//��ģʽ�Ƕ���Ա仯�û�׼
  int16_t normal_angle;
	int16_t exchange_angle;
	int16_t store_angle;
	int16_t exchange_pick_angle;
	int16_t exchange_pick_angle_2;
  int16_t bigisland_straight_angle;
	int16_t bigisland_slanted_angle;	
	int16_t smallisland_angle;
	int16_t smallisland_angle_2;
	int16_t ground_angle;
	int16_t check_angle;
	
	int16_t mode_angle;
}Motor_angle_t;

typedef struct
{
	motor_name_status MOTOR_NAME;//������궨�壬debug������
	motor_type_status MOTOR_TYPE;//�������
//	motor_turn_flag_status MOTOR_TURN_FLAG;
	
  Brushless_motor_t Brushless;
	Servo_motor_t Servo;
	Motor_angle_t Angle;
}motor_t;

typedef struct
{
	motor_name_status MOTOR_NAME;
	pid_parameter_t speed;
	pid_parameter_t angle;
}pid_motor_parameter_t;

extern motor_t Motor[MOTOR_NUMBER];

void Motor_task(void *parm);

void Motor_base_init(void);
void Motor_base_init_copy(uint8_t low,uint8_t hight);
void Motor_base_init_reversal(uint8_t ID);
void Motor_pid_init(INIT_STATUS init_status);
void Motor_angle_init(void);

void Motor_PID_Struct_Init(motor_t *Motor_recieve,pid_motor_parameter_t parameter_Struct,INIT_STATUS init_status);
uint8_t Motor_offset_angle_init(void);
void Motor_Servo_handler(uint8_t ID);
void Motor_pid_clac(uint8_t ID);
void Motor_current_into_CAN(uint8_t ID);
uint8_t Whether_Brushless_Motor(motor_t Motor);
uint8_t motor_8010_speed_get_limit(uint8_t ID);
#endif


