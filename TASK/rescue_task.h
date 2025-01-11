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
//  OUTER = 0, //救援机构还在挡板外面
//  INSIDE,    //救援机构已在挡板里面
//}recuse_baffle_sequence_t; //记录救援和挡板顺序

//typedef struct
//{
//	recuse_flag_t recuse_flag;              //添加目的:当成功救援后，可能会调整模式，使图传朝向前方,但救援机构应该保持不变
//	recuse_baffle_sequence_t recuse_baffle_flag; //控制救援和挡板归位顺序 保证挡板在救援外面
//	
//  uint8_t calibration_flag;
//  uint8_t state;//记录是否初始化
//  uint8_t last_state;
//  
//  uint8_t rescue_cmd;//救援使能标志
//	uint8_t rescue_resurrection_cmd;//复活使能标志
//	
//	uint8_t upraise_updown_flag;//控制抬升过程的
//  
//  int16_t spd_ref[3];
//  int16_t spd_fdb[3];
//	
//  int16_t current[3];
//	
//  int32_t angle_ref[3];
//  int32_t angle_fdb[3];
//  
//  int32_t init_angle[3];//初始化角度
//	
//	/*挡板给定*/
//	int16_t baffle_right_ref; 
//	int16_t baffle_left_ref; 
//	/*救援给定*/
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
