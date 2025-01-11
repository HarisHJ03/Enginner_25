#ifndef _bsp_can_H
#define _bsp_can_H

#include "stm32f4xx.h"
#include "User.h"

#define FILTER_BUF 5



/*3508的减速比 */
#define DECELE_RATIO_3508 (19.0f/1.0f)
/*大力3508  */
#define DECELE_RATIO_3508_2 (51.0f/1.0f)
/*2006的减速比*/
#define DECELE_RATIO_2006 (36.0f/1.0f)
/* 电机编码值 和 角度（度） 的比率 */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)

/* CAN send and receive ID */
typedef enum
{
	CAN_3508_M1_ID              = 0x201,//底盘 3508
	CAN_3508_M2_ID              = 0x202,
	CAN_3508_M3_ID              = 0x203,
	CAN_3508_M4_ID              = 0x204,

	CAN_UPRAISE_M1_ID           = 0x205, //抬升电机 3508 前
	CAN_UPRAISE_M2_ID           = 0x206,

	CAN_STORE_ID 								= 0x207,	//横移

  CAN_CHASSIS_ALL_ID          = 0x200,
	
	CAN_UPRAISE_ALL_ID   				= 0x1ff,
}
can1_msg_id_e;

typedef enum
{

 	CAN_SLIDE_M1_ID							= 0x201,	//平移伸出	3508 左
	CAN_SLIDE_M2_ID							= 0x202,	//平移伸出	3509 右

	CAN_MANIPULATO_M1_ID        = 0x203, 	//兑换pitch
	CAN_MANIPULATO_M2_ID        = 0x204, 	//
	
	CAN_MANIPULATO_M3_ID        = 0x205,  //机械臂6020电机
	CAN_MANIPULATO_M4_ID        = 0x206,  //机械臂
	CAN_MANIPULATO_M5_ID        = 0x207, 

	CAN_L_ID							 			= 0x200,
  CAN_H_ID             				= 0x1ff, //包含夹取
	
}can2_msg_id_e;

typedef struct
{
  uint16_t ecd;                  //编码位
  uint16_t last_ecd;
  
  int16_t  speed_rpm;            //转速
  int16_t  given_current;

  int32_t  round_cnt;
  int32_t  total_ecd;
  int32_t  total_angle;          //角度
  
  uint16_t offset_ecd;
  uint32_t msg_cnt;							//这什么玩意
  
  int32_t  ecd_raw_rate;
  int32_t  rate_buf[FILTER_BUF];
  uint8_t  buf_cut;
  int32_t  filter_rate;
	uint8_t  err_id;
} moto_measure_t;

//typedef struct
//{
//  float pit_angle;
//  float yaw_angle;
//  int16_t pit_spd;
//  int16_t yaw_spd;
//}mpu_data_t;

extern moto_measure_t moto_chassis[4];           	//底盘       3508
extern moto_measure_t moto_upraise[2];           	//抬升机构   3508
extern moto_measure_t moto_clamp_pit;						 	//夹取翻转		3508
extern moto_measure_t moto_slide[SLIDE_NUMBER];	
extern moto_measure_t moto_manipulator[6];				//机械臂电机（包括横移）
extern moto_measure_t moto_rescue[2];  
extern moto_measure_t Motor_CAN1_data[7];
extern moto_measure_t Motor_CAN2_data[7];


void encoder_data_handler(moto_measure_t* ptr, CanRxMsg *message);
void get_moto_offset(moto_measure_t* ptr, CanRxMsg *message);

void send_can1_low_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void send_can1_high_cur(int16_t iq1,int16_t iq2,int16_t iq3);
void send_can2_low_cur(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);
void send_can2_high_cur(int16_t iq1,int16_t iq2,int16_t iq3);

void send_rc_data1(void);
void send_rc_data2(void);
void send_rc_data3(void);
void send_detect_state(void);

#endif 

