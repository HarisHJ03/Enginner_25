//#ifndef _rescue_task_H
//#define _rescue_task_H


//#include "stm32f4xx.h"

//#define RESCUE_OFF 				        GPIO_ResetBits(GPIOC,GPIO_Pin_2);
//#define RESCUE_ON									GPIO_SetBits(GPIOC,GPIO_Pin_2);
//typedef enum
//{
//  RECUSEING = 0,
//  RESCUEED,
//}recuse_flag_t;

//typedef enum
//{
//  OUTER = 0, //��Ԯ�������ڵ�������
//  INSIDE,    //��Ԯ�������ڵ�������
//}recuse_baffle_sequence_t; //��¼��Ԯ�͵���˳��

//typedef struct
//{
//	recuse_flag_t recuse_flag;              //���Ŀ��:���ɹ���Ԯ�󣬿��ܻ����ģʽ��ʹͼ������ǰ��,����Ԯ����Ӧ�ñ��ֲ���
//	recuse_baffle_sequence_t recuse_baffle_flag; //���ƾ�Ԯ�͵����λ˳�� ��֤�����ھ�Ԯ����
//	
//  uint8_t calibration_flag;
//  uint8_t state;//��¼�Ƿ��ʼ��
//  uint8_t last_state;
//  
//  uint8_t rescue_cmd;//��Ԯʹ�ܱ�־
//	uint8_t rescue_resurrection_cmd;//����ʹ�ܱ�־
//	
//	uint8_t upraise_updown_flag;//����̧�����̵�
//  
//  int16_t spd_ref[3];
//  int16_t spd_fdb[3];
//	
//  int16_t current[3];
//	
//  int32_t angle_ref[3];
//  int32_t angle_fdb[3];
//  
//  int32_t init_angle[3];//��ʼ���Ƕ�
//	
//	/*�������*/
//	int16_t baffle_right_ref; 
//	int16_t baffle_left_ref; 
//	/*��Ԯ����*/
//	int16_t rescue_right_ref;
//  int16_t rescue_left_ref;
//	
//}rescue_t;

//extern rescue_t rescue;

//void rescue_task(void *parm);
//void rescue_param_init(void);
//void rescue_init_handler(void);
//void rescue_enable_handler(void);
//void barrier_rescue_enable(void);

//#endif
