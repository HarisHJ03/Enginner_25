#ifndef _detect_task_H
#define _detect_task_H


#include "stm32f4xx.h"

#define Set_bit(data,bit) (data|=(1<<bit))
#define Reset_bit(data,bit) (data&=(~(1<<bit)))

typedef enum
{
  BOTTOM_DEVICE        = 0,
/*����*/
  CHASSIS_M1_OFFLINE   = 1,//�����豸ID��
  CHASSIS_M2_OFFLINE   = 2,
  CHASSIS_M3_OFFLINE   = 3,
  CHASSIS_M4_OFFLINE   = 4,
/*ң��*/	
  REMOTE_CTRL_OFFLINE  = 5,
/*��ȡ*/
  CLAMP_M1_OFFLINE     = 6,
	CLAMP_M2_OFFLINE     = 7,
/*��̬����*/
	CLAMP_ATTITUDE_ADJUSTMENT_M1_OFFLINE = 8,
/*��ʯ�н�*/          
//	CLAMP_CLAMPING_ORE_M1_OFFLINE   = 7,
//	CLAMP_CLAMPING_ORE_M2_OFFLINE   = 8,
/*̧��*/
  UPRAISE_M1_OFFLINE   = 9,
	UPRAISE_M2_OFFLINE   = 10,
/*��Ԯ*/
  RESCUE_M1_OFFLINE    = 11,
  RESCUE_M2_OFFLINE    = 12,
/*�һ�����*/
	EXCHANGE_OFFLINE		 = 13,

  JUDGE_SYS_OFFLINE    = 14,
/*С����*/
  PC_SYS_OFFLINE       = 15, 
/*��е��*/
	MANIPULATO_M0_OFFLINE= 16,
	MANIPULATO_M1_OFFLINE= 17,
	MANIPULATO_M2_OFFLINE= 18,
	MANIPULATO_M3_OFFLINE= 19,
	MANIPULATO_M4_OFFLINE= 20,
	MANIPULATO_M5_OFFLINE= 21,
	/*������*/
	IMU_OFFLINE					 = 22,
	/*�ܸ���*/
  ERROR_LIST_LENGTH    = 23,
}err_id;
typedef union
{
	uint32_t offline;
	struct//�ؼ���struct�ܶ���������͵ı������ϣ���Ϊ�ṹ(structure)������������Ϊһ����Ԫ�����ṹ�壩
	{
    uint32_t m0 : 1; //�ṹ���е�ð�ű�ʾλ�򡣴���֤��m0����Ϊ1
    uint32_t m1 : 1;
    uint32_t m2 : 1;
    uint32_t m3 : 1;
    uint32_t m4 : 1;
		uint32_t m5 : 1;
		uint32_t m6 : 1;
		uint32_t m7 : 1;
		uint32_t m8 : 1;
		uint32_t m9 : 1;
    uint32_t m10 : 1;
    uint32_t m11 : 1;
    uint32_t m12 : 1;
		uint32_t m13 : 1;
		uint32_t m14 : 1;
		uint32_t m15 : 1;
		uint32_t m16 : 1;
		uint32_t m17 : 1;
    uint32_t reserve : 14;
	}bit;
}State_t;
typedef struct
{
  uint32_t last_times;
  uint32_t delta_times;
  uint16_t set_timeout;
  
}detect_param_t;

typedef struct
{
  uint8_t err_exist;
  detect_param_t param; 
  
}err_status;

typedef struct
{
  uint8_t    err_now_id[ERROR_LIST_LENGTH];
  err_status list[ERROR_LIST_LENGTH];//�б�
  uint32_t   offline;
}global_err_t;

extern global_err_t global_err;
extern State_t state;

void detect_task(void *parm);
void detect_param_init(void);
void err_detector_hook(int err_id);
void module_offline_detect(void);
void module_offline_callback(void);


uint8_t chassis_is_controllable(void);
uint8_t rescue_is_controllable(void);
uint8_t clamp_is_controllable(void);
uint8_t barrier_carry_is_controllable(void);

#endif

